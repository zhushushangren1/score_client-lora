#include <Arduino.h>
#include <Preferences.h>
#include <ScoreProtocol.h>

// 裁判机 E22-400T22D 串口接线：
// ESP32-S3 GPIO17 TX -> E22 RXD
// ESP32-S3 GPIO18 RX <- E22 TXD
// ESP32-S3 GPIO11     <- E22 AUX
// ESP32-S3 GPIO12     -> E22 M0
// ESP32-S3 GPIO13     -> E22 M1
//
// M0=LOW 且 M1=LOW 时，E22 进入普通透明传输模式。
// E22 模块出厂串口波特率通常是 9600。
constexpr int LORA_TX_PIN = 17;
constexpr int LORA_RX_PIN = 18;
constexpr int LORA_AUX_PIN = 11;
constexpr int LORA_M0_PIN = 12;
constexpr int LORA_M1_PIN = 13;
constexpr uint32_t LORA_UART_BAUD = 9600;

const int TEST_BATTERY_MV = 3800;

// NVS（Preferences）命名空间，用于持久化裁判机本地状态。
// 命名空间名长度限制 15 字节，"score_client" 足够短且语义清晰。
constexpr const char* NVS_NAMESPACE = "score_client";
// NVS 中存储 clientId 的键名。值为 "client1" / "client2" / "client3"；未绑定时该键不存在。
constexpr const char* NVS_KEY_CLIENT_ID = "clientId";
// 标识未绑定状态的字符串，与服务端协议字段一致。
constexpr const char* CLIENT_ID_UNASSIGNED = "UNASSIGNED";

// 本机 deviceId，由 setup() 读 MAC 后填充。
// 设计为全局 String，组帧时直接引用，避免每次发包都重新算 MAC 字符串。
// 选用低 32 位（8 位 hex）：
//   - 满足设计文档"显示后四位短码"的要求；
//   - 同型号开发板 MAC 后 32 位重复概率可忽略，作为唯一 ID 足够。
String deviceId;

// 当前绑定的 clientId（"client1" / "client2" / "client3"），未绑定时为 "UNASSIGNED"。
// setup() 从 NVS 加载；处理 ASSIGN/UNBIND 帧时同步更新内存值和 NVS。
// 不再是 const，因为绑定关系可在运行时由服务端下发改变。
String currentClientId;

// Preferences 实例，全局保留以便随时读写 NVS。
// 用 begin/end 包裹每次写入会增加 flash 延迟；这里在 setup() 中 begin 一次后长期持有。
Preferences prefs;

// 从 ESP32 efuse 读取 48 位 MAC，取低 32 位格式化成 8 位大写十六进制字符串。
// 调用时机：setup() 中一次，结果写入全局 deviceId。
// 返回：固定 8 字符长度的大写 hex，例如 "A1B2C3D4"。
// 说明：ESP.getEfuseMac() 返回 uint64_t，高 16 位为厂商信息，对裁判机标识没意义，丢弃。
String makeDeviceIdFromMac() {
    const uint64_t mac = ESP.getEfuseMac();
    const uint32_t low32 = static_cast<uint32_t>(mac & 0xFFFFFFFFULL);
    char buffer[9];
    snprintf(buffer, sizeof(buffer), "%08X", low32);
    return String(buffer);
}

// 从 NVS 读取 clientId 到全局 currentClientId。
// 调用时机：setup() 一次。
// 行为：键不存在或为空字符串时，currentClientId 被设为 "UNASSIGNED"。
// 说明：不返回任何值；调用方需要的话直接看 currentClientId。
void loadClientIdFromNvs() {
    // false=读写模式打开，方便后续 saveClientIdToNvs/clearClientIdFromNvs 复用同一个 prefs。
    prefs.begin(NVS_NAMESPACE, false);
    const String stored = prefs.getString(NVS_KEY_CLIENT_ID, "");
    if (stored.length() == 0) {
        currentClientId = CLIENT_ID_UNASSIGNED;
    } else {
        currentClientId = stored;
    }
}

// 把新的 clientId 写入 NVS 并同步更新全局变量。
// clientId：必须是 "client1" / "client2" / "client3" 之一（调用方自行校验）。
void saveClientIdToNvs(const String& clientId) {
    prefs.putString(NVS_KEY_CLIENT_ID, clientId);
    currentClientId = clientId;
}

// 清除 NVS 中的 clientId 并把全局变量设回 "UNASSIGNED"。
// 用于 UNBIND 流程或本地清除组合键（后续接入按键时使用）。
void clearClientIdFromNvs() {
    prefs.remove(NVS_KEY_CLIENT_ID);
    currentClientId = CLIENT_ID_UNASSIGNED;
}

// LoRa 接收行缓冲，handleLoraInput 按字节追加，遇到 '\n' 即视为一帧结束。
String loraLine;

// 等待 E22 进入空闲状态再发送，避免在模块忙时数据被吞掉。
// E22 的 AUX：HIGH=空闲可发送，LOW=正在收发或初始化。
// 1 秒超时是兜底，防止 AUX 接线错误（悬空或始终被拉低）让程序永久卡死。
void waitForLoraReady() {
    const unsigned long start = millis();
    while (digitalRead(LORA_AUX_PIN) == LOW && millis() - start < 1000) {
        delay(1);
    }
}

// 通过 Serial1 把一行已组好的协议数据写到 E22 进行无线发送，并在调试串口回显。
// text：已经包含 CRC 和末尾 '\n' 的完整协议行（通常来自 ScoreProtocol::buildFrame）。
void sendLoraLine(const String& text) {
    waitForLoraReady();
    Serial1.print(text);
    Serial.print("LoRa TX: ");
    Serial.print(text);
}

// 组装并发送一帧 HELLO，告诉服务端本机存在以及当前绑定状态。
// 帧格式：HELLO,deviceId,currentClientId,battMv,crc16（CRC 由 buildFrame 自动追加）。
// deviceId / currentClientId 来自全局变量；battMv 仍为联通测试用常量，等 ADC 接入后再换成真值。
void sendHello() {
    String fields[] = {
        "HELLO",
        deviceId,
        currentClientId,
        String(TEST_BATTERY_MV)
    };

    sendLoraLine(ScoreProtocol::buildFrame(fields, 4));
}

// 把已成功解析的协议帧打印到调试串口，便于联通测试时观察字段内容。
// frame：parseFrame 返回 true 时填好的结构体。
void printParsedFrame(const ScoreProtocol::ParsedFrame& frame) {
    Serial.print("Parsed type: ");
    Serial.println(ScoreProtocol::messageTypeToString(frame.type));
    Serial.print("Field count: ");
    Serial.println(frame.fieldCount);

    for (uint8_t i = 0; i < frame.fieldCount; i++) {
        Serial.print("  [");
        Serial.print(i);
        Serial.print("] ");
        Serial.println(frame.fields[i]);
    }
}

// 判断给定字符串是否是合法的 clientId。
// id：待校验文本。
// 返回：true=正好是 "client1"/"client2"/"client3" 之一；false=其他任何值。
// 用于 ASSIGN 帧的字段合法性校验，防止异常或恶意帧覆盖 NVS。
bool isValidClientId(const String& id) {
    return id == "client1" || id == "client2" || id == "client3";
}

// 组装并发送 ASSIGN_ACK 帧，作为对 ASSIGN 的应答。
// 帧格式：ASSIGN_ACK,deviceId,clientId,crc16。
// deviceId：本机 deviceId（与 ASSIGN 中收到的一致）。
// clientId：本机最终采纳的 clientId（已写入 NVS）。
void sendAssignAck(const String& clientId) {
    String fields[] = {
        "ASSIGN_ACK",
        deviceId,
        clientId
    };
    sendLoraLine(ScoreProtocol::buildFrame(fields, 3));
}

// 组装并发送 UNBIND_ACK 帧，作为对 UNBIND 的应答。
// 帧格式：UNBIND_ACK,deviceId,crc16。
// 调用前应已完成本机解绑（NVS 清除 + currentClientId 复位）。
void sendUnbindAck() {
    String fields[] = {
        "UNBIND_ACK",
        deviceId
    };
    sendLoraLine(ScoreProtocol::buildFrame(fields, 2));
}

// 处理收到的 ASSIGN 帧。
// 帧格式：ASSIGN,deviceId,clientId,crc16，fieldCount 必须为 3。
// frame：parseFrame 已成功解析的帧。
// 行为：
//   1) deviceId 不匹配本机 → 忽略（这是发给别人的）。
//   2) clientId 非法 → 打日志，不回 ACK，不写 NVS。
//   3) clientId 与当前已绑定值相同 → 仍然回 ACK（让服务端去重），但跳过 NVS 写入以减少 flash 损耗。
//   4) 合法且为新值 → 写 NVS，更新 currentClientId，回 ASSIGN_ACK。
void handleAssign(const ScoreProtocol::ParsedFrame& frame) {
    if (frame.fieldCount != 3) {
        Serial.println("ASSIGN: bad field count, ignored");
        return;
    }

    const String& targetDevice = frame.fields[1];
    const String& targetClient = frame.fields[2];

    if (targetDevice != deviceId) {
        // 该 ASSIGN 不是发给本机的（共享信道里别的裁判机的指令），静默丢弃。
        return;
    }

    if (!isValidClientId(targetClient)) {
        Serial.print("ASSIGN: invalid clientId '");
        Serial.print(targetClient);
        Serial.println("', ignored");
        return;
    }

    if (currentClientId == targetClient) {
        Serial.println("ASSIGN: clientId unchanged, ack only");
        sendAssignAck(targetClient);
        return;
    }

    saveClientIdToNvs(targetClient);
    Serial.print("ASSIGN: bound as ");
    Serial.println(targetClient);
    sendAssignAck(targetClient);
}

// 处理收到的 UNBIND 帧。
// 帧格式：UNBIND,deviceId,crc16，fieldCount 必须为 2。
// frame：parseFrame 已成功解析的帧。
// 行为：
//   1) deviceId 不匹配本机 → 忽略。
//   2) 本机当前已是未绑定状态 → 仍然回 ACK（让服务端确认收到），跳过 NVS 写入。
//   3) 已绑定 → 清 NVS、复位 currentClientId、回 UNBIND_ACK。
void handleUnbind(const ScoreProtocol::ParsedFrame& frame) {
    if (frame.fieldCount != 2) {
        Serial.println("UNBIND: bad field count, ignored");
        return;
    }

    const String& targetDevice = frame.fields[1];
    if (targetDevice != deviceId) {
        return;
    }

    if (currentClientId == CLIENT_ID_UNASSIGNED) {
        Serial.println("UNBIND: already unassigned, ack only");
        sendUnbindAck();
        return;
    }

    clearClientIdFromNvs();
    Serial.println("UNBIND: cleared local binding");
    sendUnbindAck();
}

// 从 Serial1（E22 透传口）按字节读入，遇到 '\n' 视为一行结束，然后做 CRC 校验、字段解析、类型识别、打印和业务处理。
// 行长度上限 120 字节，超长直接丢弃，防止异常输入把 String 无限撑大。
void handleLoraInput() {
    while (Serial1.available() > 0) {
        const char c = static_cast<char>(Serial1.read());
        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            if (loraLine.length() > 0) {
                Serial.print("LoRa RX: ");
                Serial.println(loraLine);

                ScoreProtocol::ParsedFrame frame;
                if (ScoreProtocol::parseFrame(loraLine, frame)) {
                    printParsedFrame(frame);

                    switch (frame.type) {
                        case ScoreProtocol::MessageType::Status:
                            // 联通测试：收到 STATUS 即认为往返链路通。
                            Serial.println("STATUS received successfully");
                            break;
                        case ScoreProtocol::MessageType::Assign:
                            handleAssign(frame);
                            break;
                        case ScoreProtocol::MessageType::Unbind:
                            handleUnbind(frame);
                            break;
                        default:
                            // 当前阶段其他类型（HELLO/HEARTBEAT/SUBMIT/...）裁判机不主动处理。
                            break;
                    }
                } else {
                    Serial.println("Invalid protocol frame");
                }

                loraLine = "";
            }
            continue;
        }

        if (loraLine.length() < 120) {
            loraLine += c;
        } else {
            // 丢弃过长数据，避免异常输入导致 String 无限增长。
            loraLine = "";
        }
    }
}

// Arduino 启动钩子，上电后只运行一次。
// 职责：初始化调试串口、读取 MAC 生成 deviceId、从 NVS 加载 clientId、把 E22 控制脚切到透传模式（M0=M1=LOW）、打开 Serial1 与 E22 通信、并打印启动横幅。
void setup() {
    Serial.begin(115200);
    const unsigned long serialWaitStart = millis();
    while (!Serial && millis() - serialWaitStart < 5000) {
        delay(10);
    }
    delay(500);

    // 读 MAC 生成本机唯一 deviceId。
    // 必须早于第一次 sendHello() 之前完成；放在串口准备好之后是为了能立刻打印结果。
    deviceId = makeDeviceIdFromMac();

    // 加载持久化的 clientId。loadClientIdFromNvs 内部会 prefs.begin()，从此 prefs 在整个 loop 期间保持打开。
    loadClientIdFromNvs();

    Serial.println();
    Serial.println("score_client-lora boot");
    Serial.print("Device ID: ");
    Serial.println(deviceId);
    Serial.print("Client ID: ");
    Serial.println(currentClientId);
    Serial.print("Millis: ");
    Serial.println(millis());

    pinMode(LORA_M0_PIN, OUTPUT);
    pinMode(LORA_M1_PIN, OUTPUT);
    pinMode(LORA_AUX_PIN, INPUT_PULLUP);

    // M0=LOW、M1=LOW：E22 进入正常透明传输模式（详见设计文档 4.3 节）。
    digitalWrite(LORA_M0_PIN, LOW);
    digitalWrite(LORA_M1_PIN, LOW);

    Serial1.begin(LORA_UART_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    delay(500);

    Serial.println("E22 UART transparent test ready");
    Serial.println("Client sends HELLO every 2 seconds and reacts to ASSIGN/UNBIND.");
}

// Arduino 主循环，会被反复调用。
// 每轮先处理收到的 LoRa 数据，再用静态变量按 2 秒间隔发一次 HELLO（非阻塞定时，不能用 delay 否则会丢接收）。
void loop() {
    handleLoraInput();

    static unsigned long lastPing = 0;
    if (millis() - lastPing >= 2000) {
        lastPing = millis();
        sendHello();
    }
}
