#include <Arduino.h>
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

const String DEVICE_ID = "TESTDEVICE01";
const String CURRENT_CLIENT_ID = "UNASSIGNED";
const int TEST_BATTERY_MV = 3800;

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
// 当前为联通测试，deviceId/clientId/battMv 都用文件顶部的常量代替；后续会换成 MAC + NVS + ADC。
void sendHello() {
    String fields[] = {
        "HELLO",
        DEVICE_ID,
        CURRENT_CLIENT_ID,
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

                    // 联通测试：收到 STATUS 即认为往返链路通。
                    if (frame.type == ScoreProtocol::MessageType::Status) {
                        Serial.println("STATUS received successfully");
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
// 职责：初始化调试串口、把 E22 控制脚切到透传模式（M0=M1=LOW）、打开 Serial1 与 E22 通信、并打印启动横幅。
void setup() {
    Serial.begin(115200);
    const unsigned long serialWaitStart = millis();
    while (!Serial && millis() - serialWaitStart < 5000) {
        delay(10);
    }
    delay(500);

    Serial.println();
    Serial.println("score_client-lora boot");
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
    Serial.println("Client sends HELLO every 2 seconds and waits for STATUS.");
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
