#include <Arduino.h>
#include <Preferences.h>
#include <ScoreProtocol.h>
#include <TM1637Display.h>

// 裁判机 E22-400T22D 串口接线（改到 ESP32-S3 DevKitC 右侧连续排针，减少洞洞板飞线）：
// ESP32-S3 GPIO41 TX -> E22 RXD
// ESP32-S3 GPIO40 RX <- E22 TXD
// ESP32-S3 GPIO42     <- E22 AUX
// ESP32-S3 GPIO38     -> E22 M0
// ESP32-S3 GPIO39     -> E22 M1
//
// M0=LOW 且 M1=LOW 时，E22 进入普通透明传输模式。
// E22 模块出厂串口波特率通常是 9600。
constexpr int LORA_TX_PIN = 41;
constexpr int LORA_RX_PIN = 40;
constexpr int LORA_AUX_PIN = 42;
constexpr int LORA_M0_PIN = 38;
constexpr int LORA_M1_PIN = 39;
constexpr uint32_t LORA_UART_BAUD = 9600;

// TM1637 数码管接线（设计文档 4.4 节）：
// CLK -> GPIO48，DIO -> GPIO47，VCC -> 3.3V，GND -> 共地
// 避开 GPIO35~GPIO37：N16R8 等带 OPI PSRAM 的板卡用这三个脚连八线 PSRAM，不能复用。
constexpr int TM1637_CLK_PIN = 48;
constexpr int TM1637_DIO_PIN = 47;

// 模块全局实例。构造时立刻把 CLK/DIO 设为 OUTPUT 拉高，所以即使 setup 还没跑也不会乱亮。
TM1637Display display(TM1637_CLK_PIN, TM1637_DIO_PIN);

// 5 个录分按键（设计文档 4.5 节）。
// 接法：GPIO ---- 按键 ---- GND，使用 INPUT_PULLUP，按下时 GPIO 被拉到 LOW。
// 顺序与下面 BUTTON_COUNT/handleButtonShort 的 switch 一一对应。
constexpr int BUTTON_RED_1_PIN  = 4;  // 红 +1 / 长按 -1
constexpr int BUTTON_RED_2_PIN  = 5;  // 红 +2 / 长按 -2
constexpr int BUTTON_BLUE_1_PIN = 6;  // 蓝 +1 / 长按 -1
constexpr int BUTTON_BLUE_2_PIN = 7;  // 蓝 +2 / 长按 -2
constexpr int BUTTON_SUBMIT_PIN = 8;  // 短按提交 / 长按本轮清零

constexpr uint8_t BUTTON_COUNT = 5;
constexpr int BUTTON_PINS[BUTTON_COUNT] = {
    BUTTON_RED_1_PIN,
    BUTTON_RED_2_PIN,
    BUTTON_BLUE_1_PIN,
    BUTTON_BLUE_2_PIN,
    BUTTON_SUBMIT_PIN
};

// 软件去抖时间：20ms 覆盖大多数轻触开关的弹跳窗口。
constexpr unsigned long BUTTON_DEBOUNCE_MS = 20;
// 长按判定阈值：600ms。低于这个值视为短按，达到这个时长仍未松手则触发一次长按事件。
// 选 600 是因为低于 500 误触多、高于 800 用户会感觉手感迟钝。
constexpr unsigned long BUTTON_LONG_PRESS_MS = 600;

// ===== 步骤 10：电池电压采样 + 低电量 LED =====
// 设计文档 4.6 节：电池正极经 100K+100K 分压后接 GPIO15，节点电压 = 电池电压 / 2。
// GPIO15 = ADC2_CH4；裁判机不开 WiFi，ADC2 可正常使用。
constexpr int BATTERY_ADC_PIN = 15;
// 分压比：节点电压 × 2 = 电池电压。
constexpr float BATTERY_DIVIDER_RATIO = 2.0f;
// 实测校准系数：用万用表量电池真实电压 ÷ 程序读出的电压，把比值填到这里，修正分压电阻与 ADC 偏差。
constexpr float BATTERY_CALIBRATION = 1.0f;
// 采样周期与每次平均样本数：5s 刷新一次够用，8 次平均抑制 ADC 抖动。
constexpr unsigned long BATTERY_SAMPLE_INTERVAL_MS = 5000;
constexpr int BATTERY_SAMPLE_COUNT = 8;
// 低电量阈值（设计文档第 9 节）：低于此值闪 LED 提示，但仍允许使用/提交。
constexpr int BATTERY_LOW_MV = 3700;

// 低电量指示 LED：GPIO2 -> 1K 限流电阻 -> LED 正极，LED 负极 -> GND（高电平点亮）。
// 换引脚改这里即可；GPIO2 是普通 GPIO，未占用启动/flash/PSRAM。
constexpr int BATTERY_LOW_LED_PIN = 2;
constexpr unsigned long BATTERY_LOW_BLINK_INTERVAL_MS = 400;

// 最近一次换算出的电池电压（毫伏）。0 表示尚未采样。
// setup() 首次采样在 sendHello() 之前完成，保证第一帧 HELLO 就带真实电压。
int batteryMv = 0;
unsigned long lastBatterySampleMs = 0;

// 低电量 LED 闪烁状态。
unsigned long lastBatteryBlinkMs = 0;
bool batteryLowLedOn = false;

// 读一次电池电压（毫伏）：多次采样取平均，再按分压比和校准系数换算。
// analogReadMilliVolts 返回引脚处已校准电压，乘 2（分压比）即电池电压。
int readBatteryMv() {
    uint32_t sum = 0;
    for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
        sum += analogReadMilliVolts(BATTERY_ADC_PIN);
    }
    const float nodeMv = static_cast<float>(sum) / BATTERY_SAMPLE_COUNT;
    return static_cast<int>(nodeMv * BATTERY_DIVIDER_RATIO * BATTERY_CALIBRATION + 0.5f);
}

// 到周期就刷新一次 batteryMv（非阻塞）。在 loop 中调用。
void sampleBatteryIfDue() {
    if (millis() - lastBatterySampleMs < BATTERY_SAMPLE_INTERVAL_MS) {
        return;
    }
    lastBatterySampleMs = millis();
    batteryMv = readBatteryMv();
}

// 电量低于阈值时闪烁 LED，正常时熄灭（非阻塞，loop 中调用）。
// 仅作低电量指示：不影响数码管显示，也不拦截提交（按需求只用 LED + 上报服务端）。
void updateBatteryLowLed() {
    const bool low = (batteryMv != 0 && batteryMv < BATTERY_LOW_MV);
    if (!low) {
        if (batteryLowLedOn) {
            batteryLowLedOn = false;
            digitalWrite(BATTERY_LOW_LED_PIN, LOW);
        }
        return;
    }
    if (millis() - lastBatteryBlinkMs >= BATTERY_LOW_BLINK_INTERVAL_MS) {
        lastBatteryBlinkMs = millis();
        batteryLowLedOn = !batteryLowLedOn;
        digitalWrite(BATTERY_LOW_LED_PIN, batteryLowLedOn ? HIGH : LOW);
    }
}

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

// ===== 步骤 7：SUBMIT/ACK 相关全局状态 =====

// 设计文档第 8 节定义的可靠性参数：
// 首次发送随机退避 0~300ms，未收 ACK 后等 300~1200ms 重发，最多重发 5 次。
constexpr unsigned long SUBMIT_INITIAL_BACKOFF_MAX_MS = 300;
constexpr unsigned long SUBMIT_RETRY_BACKOFF_MIN_MS = 300;
constexpr unsigned long SUBMIT_RETRY_BACKOFF_MAX_MS = 1200;
constexpr uint8_t SUBMIT_MAX_RETRIES = 5;

// 本机递增的 msgId 计数器。
// 设计成 unsigned long，单调递增；重启后从 1 开始（不持久化）。
// 第一版重启会丢失 msgId 历史，可能与服务端之前见过的 msgId 撞号，
// 但服务端去重以 (deviceId, roundId, msgId) 三元组为键，重启后 roundId 大概率已经推进，
// 即使 roundId 没变，msgId=1 的"新"提交也只会因为 roundSubmissions[slot].submitted 为 true
// 而被服务端返回 ERR_ALREADY_SUBMITTED，不会误计分。
unsigned long localMsgId = 0;

// 最近一次从服务端 STATUS 帧读到的 roundId。0 表示尚未收到任何 STATUS。
// 在没拿到这个值之前不允许提交（避免拿默认值乱猜导致 ERR_BAD_ROUND）。
unsigned long currentServerRoundId = 0;

// 本轮是否已被服务端确认（收到过 OK / OK_DUPLICATE / ERR_ALREADY_SUBMITTED 之一）。
// 锁定后再触发 submit 命令会被拒，必须等服务端推进到下一轮（STATUS 带新 roundId）才解锁。
bool lockedForCurrentRound = false;
unsigned long lockedRoundId = 0;

// 当前是否有挂起的 SUBMIT 在等 ACK / 等重传。
// pendingMsgId == 0 表示没有挂起；非 0 表示对应这次提交还没收到 ACK。
unsigned long pendingMsgId = 0;
unsigned long pendingRoundId = 0;
int pendingRed = 0;
int pendingBlue = 0;
uint8_t pendingRetries = 0;          // 已经发送了几次（首发 +1，重发 +1...）
unsigned long pendingNextSendMs = 0; // 下一次重发的 millis() 时刻

// ===== 步骤 8：TM1637 显示相关状态 =====

// 显示覆盖（hold overlay）机制：在普通状态映射之上短暂强制显示一段文字，
// 例如绑定成功后闪 2s "J1__"、提交失败后闪 3s " Err"。到期后自动回落到普通映射。
// displayHoldUntilMs == 0 表示当前没有覆盖在生效。
unsigned long displayHoldUntilMs = 0;
char displayHoldText[5] = {0};  // 4 字符 + '\0'

// 普通状态显示的最低刷新间隔。
// 状态变化点会主动调 updateDisplay；这里只是兜底，让覆盖到期等情况能尽快显现。
constexpr unsigned long DISPLAY_REFRESH_INTERVAL_MS = 200;
unsigned long lastDisplayRefreshMs = 0;

// ===== 步骤 9：按键 + 本地编辑分数 =====

// 编辑中的红蓝分数（设计文档 EDITING 状态）。
// 钳到 0..99；boot/绑定变更/换轮/长按清零 时被复位为 0。
// 不持久化到 NVS——未提交的分数没必要跨重启保留。
int localRed = 0;
int localBlue = 0;

// 每个按键的去抖与长按状态。
// stableLevel：去抖后的稳定电平（HIGH=未按、LOW=按下）。
// lastRawLevel / lastRawChangeMs：最近一次原始电平及其变化时刻，用来在 BUTTON_DEBOUNCE_MS 后采纳为稳定电平。
// pressedAtMs：最近一次稳定电平从 HIGH 转 LOW（按下沿）的时刻，用于长按判定。
// longPressFired：本次按下期间是否已经触发过长按事件，避免松手时再误触发短按。
struct ButtonState {
    bool stableLevel = true;        // 上电默认松开
    bool lastRawLevel = true;
    unsigned long lastRawChangeMs = 0;
    unsigned long pressedAtMs = 0;
    bool longPressFired = false;
};
ButtonState buttons[BUTTON_COUNT];

// 串口命令行缓冲。客户端串口命令暂时只用于模拟按键提交（按键属步骤 9）。
String serialLine;

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
// HELLO 语义：上电/重连时的一次性自报，不是周期保活。
// 周期保活由 sendHeartbeat 负责（10s/15s 间隔，详见设计文档第 9 节）。
// deviceId / currentClientId / batteryMv 均来自全局变量；batteryMv 由 ADC 周期采样（步骤 10）。
void sendHello() {
    String fields[] = {
        "HELLO",
        deviceId,
        currentClientId,
        String(batteryMv)
    };

    sendLoraLine(ScoreProtocol::buildFrame(fields, 4));
}

// 本机递增的 HEARTBEAT msgId 计数器，每次 sendHeartbeat 自增。
// 与 SUBMIT 的 localMsgId 完全独立——SUBMIT 的 msgId 用于服务端的 (deviceId, roundId, msgId)
// 去重；HEARTBEAT 的 msgId 当前只是为了让协议字段齐整、便于服务端日志区分包序，不参与去重。
unsigned long localHeartbeatMsgId = 0;

// 最近一次发 HEARTBEAT 的 millis() 时刻。
// 在 loop 中按周期触发，也可被 handleAssign/handleUnbind 拨回（设为 0 等价于"上次很久以前"），
// 让状态变化后能尽快发一帧 HEARTBEAT、拿回 STATUS 同步轮次。
unsigned long lastHeartbeatMs = 0;

// 触发下一次心跳尽快发送。
// 在绑定状态变化（ASSIGN/UNBIND 成功）后调用，避免客户端要等满 10s 才能拿到 STATUS。
void scheduleHeartbeatSoon() {
    lastHeartbeatMs = 0;
}

// 组装并发送一帧 HEARTBEAT。
// 帧格式：HEARTBEAT,deviceId,currentClientId,battMv,msgId,crc16（设计文档第 6 节）。
// 周期由 loop 中的定时器控制，频率：未锁定 10s、已锁定 15s。
void sendHeartbeat() {
    localHeartbeatMsgId++;
    String fields[] = {
        "HEARTBEAT",
        deviceId,
        currentClientId,
        String(batteryMv),
        String(localHeartbeatMsgId)
    };

    sendLoraLine(ScoreProtocol::buildFrame(fields, 5));
}

// 短暂强制显示一段文字（"hold overlay"），到期后自动回到 updateDisplay 的状态映射。
// text：最长 4 字符；多余截断；nullptr 时取消当前 hold。
// durationMs：覆盖持续时间。
// 适用：绑定成功瞬间显示 "J1__"、提交失败显示 " Err" 等。
void holdDisplay(const char* text, unsigned long durationMs) {
    if (text == nullptr) {
        displayHoldUntilMs = 0;
        displayHoldText[0] = '\0';
        return;
    }
    uint8_t i = 0;
    for (; i < 4 && text[i] != '\0'; i++) {
        displayHoldText[i] = text[i];
    }
    for (; i < 4; i++) {
        displayHoldText[i] = ' ';
    }
    displayHoldText[4] = '\0';
    displayHoldUntilMs = millis() + durationMs;
}

// 根据当前逻辑状态把内容推到 TM1637。
// 优先级（从高到低）：
//   1) 当前有未到期的 hold overlay → 直接显示 displayHoldText
//   2) 未绑定 → deviceId 后 4 位 hex
//   3) 正在发送中（pendingMsgId != 0）→ "SEND"
//   4) 锁定（本轮已被服务端确认）→ "----"
//   5) 已绑定空闲 → "00.00"（按键还没接，红蓝先固定为 0/0；步骤 9 接入按键后会替换为实时分数）
// 每次调用会向数码管下发一组 segment 字节，TM1637 协议时序合计约 0.5ms，调用代价低。
void updateDisplay() {
    lastDisplayRefreshMs = millis();

    if (displayHoldUntilMs != 0 && static_cast<long>(millis() - displayHoldUntilMs) < 0) {
        display.showText(displayHoldText);
        return;
    }
    if (displayHoldUntilMs != 0) {
        // 过期，清掉 hold 标记，继续走普通映射。
        displayHoldUntilMs = 0;
    }

    if (currentClientId == CLIENT_ID_UNASSIGNED) {
        // 显示 deviceId 后 4 位 hex 供操作员在网页/服务端串口里识别该机。
        const String tail = deviceId.length() >= 4 ? deviceId.substring(deviceId.length() - 4) : deviceId;
        display.showText(tail.c_str());
        return;
    }

    if (pendingMsgId != 0) {
        display.showText("SEND");
        return;
    }

    if (lockedForCurrentRound) {
        display.showText("----");
        return;
    }

    // 已绑定空闲：显示用户当前编辑的红蓝分数（步骤 9 按键加减后实时变化）。
    display.showScore(localRed, localBlue);
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

// 组装并发送一帧 SUBMIT。
// 帧格式：SUBMIT,deviceId,clientId,roundId,msgId,red,blue,battMv,crc16。
// 调用方负责保证 pendingMsgId/pendingRoundId/pendingRed/pendingBlue 已被填好。
// battMv 取自全局 batteryMv（GPIO15 ADC 周期采样，步骤 10）。
void sendSubmit() {
    String fields[] = {
        "SUBMIT",
        deviceId,
        currentClientId,
        String(pendingRoundId),
        String(pendingMsgId),
        String(pendingRed),
        String(pendingBlue),
        String(batteryMv)
    };
    sendLoraLine(ScoreProtocol::buildFrame(fields, 8));
}

// 清掉挂起的提交状态。
// 用于 ACK 到达成功匹配、或重传次数耗尽、或显式取消时。
void clearPendingSubmit() {
    pendingMsgId = 0;
    pendingRoundId = 0;
    pendingRed = 0;
    pendingBlue = 0;
    pendingRetries = 0;
    pendingNextSendMs = 0;
}

// 在 loop() 中按时机驱动 SUBMIT 重传。
// 行为：
//   - pendingMsgId == 0 → 没有挂起的提交，直接返回。
//   - 还没到 pendingNextSendMs → 等待。
//   - 已达 SUBMIT_MAX_RETRIES → 放弃，清挂起，打错误日志。
//   - 否则：发一帧 SUBMIT、retries++、安排下一次重发时刻（随机 300~1200ms）。
// 非阻塞，不能 delay，否则会丢 LoRa 接收。
void drivePendingSubmit() {
    if (pendingMsgId == 0) {
        return;
    }
    if (static_cast<long>(millis() - pendingNextSendMs) < 0) {
        return;
    }
    if (pendingRetries >= SUBMIT_MAX_RETRIES) {
        Serial.print("SUBMIT: gave up after ");
        Serial.print(pendingRetries);
        Serial.print(" attempts, msgId=");
        Serial.println(pendingMsgId);
        clearPendingSubmit();
        holdDisplay(" Err", 3000);
        updateDisplay();
        return;
    }

    sendSubmit();
    pendingRetries++;
    const unsigned long backoff = SUBMIT_RETRY_BACKOFF_MIN_MS +
        random(SUBMIT_RETRY_BACKOFF_MAX_MS - SUBMIT_RETRY_BACKOFF_MIN_MS + 1);
    pendingNextSendMs = millis() + backoff;

    Serial.print("SUBMIT attempt ");
    Serial.print(pendingRetries);
    Serial.print("/");
    Serial.print(SUBMIT_MAX_RETRIES);
    Serial.print(", next retry in ");
    Serial.print(backoff);
    Serial.println("ms");
}

// 检查 currentServerRoundId 是否已经超过 lockedRoundId，是则解锁本轮锁定。
// 每次从 STATUS 帧更新 currentServerRoundId 后调用。
void maybeUnlockOnRoundChange() {
    if (lockedForCurrentRound && lockedRoundId != currentServerRoundId) {
        Serial.print("Round changed (locked=");
        Serial.print(lockedRoundId);
        Serial.print(", current=");
        Serial.print(currentServerRoundId);
        Serial.println("), unlocked");
        lockedForCurrentRound = false;
        lockedRoundId = 0;
        // 新一轮回到 EDITING：清掉上一轮残留分数，否则数码管会停留在上一轮的最终值。
        localRed = 0;
        localBlue = 0;
        updateDisplay();
    }
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
    // 绑定关系变更后尽快发心跳，让服务端回 STATUS、客户端尽早拿到 currentServerRoundId，
    // 否则会有 ~10s 窗口期内无法通过 submit 命令。
    scheduleHeartbeatSoon();

    // 显示绑定确认 "J1__"/"J2__"/"J3__" 2 秒，给操作员视觉反馈。
    // targetClient 形如 "client1"，取末尾数字。
    char holdText[5] = "J?  ";
    if (targetClient.length() > 0) {
        holdText[1] = targetClient[targetClient.length() - 1];
    }
    holdDisplay(holdText, 2000);
    updateDisplay();
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
    // 绑定关系变更后尽快发心跳，让服务端尽早把这台设备登记回未绑定表。
    scheduleHeartbeatSoon();
    updateDisplay();
}

// 处理收到的 STATUS 帧。
// 帧格式：STATUS,deviceId,clientId,roundId,roundOpen,submitted,crc16，fieldCount 必须为 6。
// frame：parseFrame 已成功解析的帧。
// 行为：
//   - 校验 deviceId 是否指向本机；不是就丢弃（STATUS 是对特定裁判机的 HELLO 回应）。
//   - 解析 roundId，写入全局 currentServerRoundId。这是客户端发起 SUBMIT 时使用的轮号。
//   - 如果 lockedRoundId 与新轮号不一致，触发解锁。
//   - submitted 字段第一版只打日志，客户端自己维护 lockedForCurrentRound 状态。
void handleStatus(const ScoreProtocol::ParsedFrame& frame) {
    if (frame.fieldCount != 6) {
        Serial.println("STATUS: bad field count, ignored");
        return;
    }
    if (frame.fields[1] != deviceId) {
        // 不是发给本机的 STATUS（同信道里别的裁判机的应答），静默丢弃。
        return;
    }

    unsigned long parsedRound = 0;
    if (!ScoreProtocol::parseUnsignedLong(frame.fields[3], parsedRound)) {
        Serial.println("STATUS: bad roundId, ignored");
        return;
    }
    currentServerRoundId = parsedRound;
    maybeUnlockOnRoundChange();

    Serial.print("STATUS: round=");
    Serial.print(currentServerRoundId);
    Serial.print(" open=");
    Serial.print(frame.fields[4]);
    Serial.print(" submitted=");
    Serial.println(frame.fields[5]);
}

// 处理收到的 ACK 帧（对 SUBMIT 的回应）。
// 帧格式：ACK,deviceId,clientId,roundId,msgId,status,crc16，fieldCount 必须为 6。
// frame：parseFrame 已成功解析的帧。
// 行为：
//   - 校验 deviceId / roundId / msgId 与挂起的提交完全一致；任何一项不匹配 → 静默忽略
//     （可能是过期的重传 ACK、或是同信道里发给别人的 ACK 误入本机）。
//   - status == OK / OK_DUPLICATE / ERR_ALREADY_SUBMITTED → 视为"本轮已被服务端确认"，
//     进入锁定，清挂起。设计文档第 8 节明确这三种都是终止状态。
//   - status == ERR_BAD_ROUND 或其他错误 → 不锁定、清挂起，让用户看到失败原因再决定怎么办。
void handleAck(const ScoreProtocol::ParsedFrame& frame) {
    if (frame.fieldCount != 6) {
        Serial.println("ACK: bad field count, ignored");
        return;
    }
    if (frame.fields[1] != deviceId) {
        return;
    }
    if (pendingMsgId == 0) {
        Serial.println("ACK: no pending submit, ignored");
        return;
    }

    unsigned long ackRound = 0;
    unsigned long ackMsgId = 0;
    if (!ScoreProtocol::parseUnsignedLong(frame.fields[3], ackRound) ||
        !ScoreProtocol::parseUnsignedLong(frame.fields[4], ackMsgId)) {
        Serial.println("ACK: bad roundId/msgId, ignored");
        return;
    }

    if (ackRound != pendingRoundId || ackMsgId != pendingMsgId) {
        // 过期 ACK（比如前一轮的重复 ACK 在我们已经放弃后才到）；静默忽略。
        return;
    }

    const String& status = frame.fields[5];
    if (status == "OK" || status == "OK_DUPLICATE" || status == "ERR_ALREADY_SUBMITTED") {
        lockedForCurrentRound = true;
        lockedRoundId = pendingRoundId;
        Serial.print("ACK ");
        Serial.print(status);
        Serial.print(", locked for round ");
        Serial.println(lockedRoundId);
        clearPendingSubmit();
        updateDisplay();  // 进入 LOCKED 状态，显示 "----"
    } else {
        Serial.print("ACK ");
        Serial.print(status);
        Serial.print(", submit failed for round ");
        Serial.print(pendingRoundId);
        Serial.print(" msgId ");
        Serial.println(pendingMsgId);
        clearPendingSubmit();
        // 提交失败显示 " Err" 3 秒提示操作员，到期后回到普通映射。
        // 设计文档 ERROR 状态是持久的，但第一版没有按键来"清错"，先按短暂提示处理；
        // 步骤 9 接入按键后再改成"按任意键清错"的持久状态。
        holdDisplay(" Err", 3000);
        updateDisplay();
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

                    switch (frame.type) {
                        case ScoreProtocol::MessageType::Status:
                            handleStatus(frame);
                            break;
                        case ScoreProtocol::MessageType::Ack:
                            handleAck(frame);
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

// 把一行用 ' ' 拆分成最多 maxTokens 段，写入 tokens；连续空格按一个处理。
// line：原始行（不含末尾换行）。
// tokens：输出数组。
// maxTokens：tokens 容量。
// 返回：实际解析出的段数（0 到 maxTokens）。
// 与服务端 tokenizeBySpace 实现一致，复用为各自项目独立函数，避免在 shared 库里塞太多东西。
uint8_t tokenizeBySpace(const String& line, String tokens[], uint8_t maxTokens) {
    uint8_t count = 0;
    int start = 0;
    const int len = line.length();

    while (start < len && count < maxTokens) {
        while (start < len && line[start] == ' ') {
            start++;
        }
        if (start >= len) break;

        int end = start;
        while (end < len && line[end] != ' ') {
            end++;
        }
        tokens[count++] = line.substring(start, end);
        start = end;
    }
    return count;
}

// 打印客户端当前状态：deviceId / clientId / serverRoundId / locked / pending。
// 用于 show 命令和首次启动横幅。
void printClientState() {
    Serial.print("Device: ");
    Serial.print(deviceId);
    Serial.print("  Client: ");
    Serial.println(currentClientId);
    Serial.print("Server roundId: ");
    if (currentServerRoundId == 0) {
        Serial.println("<not yet received>");
    } else {
        Serial.println(currentServerRoundId);
    }
    Serial.print("Locked: ");
    if (lockedForCurrentRound) {
        Serial.print("yes (round ");
        Serial.print(lockedRoundId);
        Serial.println(")");
    } else {
        Serial.println("no");
    }
    Serial.print("Pending: ");
    if (pendingMsgId == 0) {
        Serial.println("none");
    } else {
        Serial.print("msgId=");
        Serial.print(pendingMsgId);
        Serial.print(" round=");
        Serial.print(pendingRoundId);
        Serial.print(" red=");
        Serial.print(pendingRed);
        Serial.print(" blue=");
        Serial.print(pendingBlue);
        Serial.print(" retries=");
        Serial.print(pendingRetries);
        Serial.print("/");
        Serial.println(SUBMIT_MAX_RETRIES);
    }

    // 显示状态：先描述应当显示的内容（与 updateDisplay 的优先级判断保持一致），
    // 再说明是否处于 hold overlay 期。便于联调时把"软件状态"与"硬件显示"做对照。
    Serial.print("Display: ");
    if (displayHoldUntilMs != 0 && static_cast<long>(millis() - displayHoldUntilMs) < 0) {
        Serial.print("hold '");
        Serial.print(displayHoldText);
        Serial.print("' remaining ");
        Serial.print(displayHoldUntilMs - millis());
        Serial.println("ms");
    } else if (currentClientId == CLIENT_ID_UNASSIGNED) {
        const String tail = deviceId.length() >= 4 ? deviceId.substring(deviceId.length() - 4) : deviceId;
        Serial.print("deviceId tail '");
        Serial.print(tail);
        Serial.println("'");
    } else if (pendingMsgId != 0) {
        Serial.println("SEND");
    } else if (lockedForCurrentRound) {
        Serial.println("----");
    } else {
        Serial.println("00.00 (placeholder)");
    }
}

// 尝试发起一次新的 SUBMIT。
// red / blue：本次提交的分数（不会再次钳位，调用方负责保证 0..99）。
// 返回：true=已入挂起队列等待发送；false=被前置条件拒绝（已 Serial.print 了具体原因）。
// 共享逻辑给串口命令和按键短按"提交"复用，避免分数提交流程分裂成两份代码。
bool tryQueueSubmit(int red, int blue) {
    if (currentClientId == CLIENT_ID_UNASSIGNED) {
        Serial.println("submit: not bound yet (still UNASSIGNED), wait for server ASSIGN");
        return false;
    }
    if (currentServerRoundId == 0) {
        Serial.println("submit: no STATUS received yet, cannot determine roundId");
        return false;
    }
    if (pendingMsgId != 0) {
        Serial.print("submit: previous submit still pending (msgId=");
        Serial.print(pendingMsgId);
        Serial.println("), wait or it will fail after 5 retries");
        return false;
    }
    if (lockedForCurrentRound && lockedRoundId == currentServerRoundId) {
        Serial.print("submit: already locked for round ");
        Serial.print(lockedRoundId);
        Serial.println(", wait for next-round");
        return false;
    }

    localMsgId++;
    pendingMsgId = localMsgId;
    pendingRoundId = currentServerRoundId;
    pendingRed = red;
    pendingBlue = blue;
    pendingRetries = 0;

    // 首次发送随机退避 0~300ms，错开多个裁判同时按提交时的信道碰撞。
    pendingNextSendMs = millis() + random(SUBMIT_INITIAL_BACKOFF_MAX_MS + 1);

    Serial.print("submit queued: round=");
    Serial.print(pendingRoundId);
    Serial.print(" msgId=");
    Serial.print(pendingMsgId);
    Serial.print(" red=");
    Serial.print(pendingRed);
    Serial.print(" blue=");
    Serial.print(pendingBlue);
    Serial.print(" first send in ");
    Serial.print(pendingNextSendMs - millis());
    Serial.println("ms");

    updateDisplay();  // 进入 SENDING 状态，显示 "SEND"
    return true;
}

// 把 localRed/localBlue 复位为 0/0 并刷新显示。
// 用途：长按提交键清零、绑定关系变化、轮次推进等。
void resetLocalScore() {
    localRed = 0;
    localBlue = 0;
    updateDisplay();
}

// 增减红/蓝分数，自动钳到 0..99。
// 调用方提供有符号的 delta，比如 +1、-2。
// 调用结束自动刷显示，避免操作员看不到反馈。
void adjustRed(int delta) {
    int v = localRed + delta;
    if (v < 0) v = 0;
    if (v > 99) v = 99;
    localRed = v;
    updateDisplay();
}
void adjustBlue(int delta) {
    int v = localBlue + delta;
    if (v < 0) v = 0;
    if (v > 99) v = 99;
    localBlue = v;
    updateDisplay();
}

// 按键事件分发：短按。
// idx：0..4，与 BUTTON_PINS 顺序一致。
// 设计文档第 3.4 节：
//   红+1=短按红+1，红+2=短按红+2，蓝同理；提交键短按 → 触发 SUBMIT（用当前 localRed/localBlue）。
void handleButtonShort(uint8_t idx) {
    switch (idx) {
        case 0: adjustRed(+1); break;
        case 1: adjustRed(+2); break;
        case 2: adjustBlue(+1); break;
        case 3: adjustBlue(+2); break;
        case 4:
            Serial.print("BUTTON: submit red=");
            Serial.print(localRed);
            Serial.print(" blue=");
            Serial.println(localBlue);
            tryQueueSubmit(localRed, localBlue);
            break;
        default: break;
    }
}

// 按键事件分发：长按。
// 设计文档第 3.4 节：
//   红+1 长按 → 红-1；红+2 长按 → 红-2；蓝同理；提交键长按 → 本轮清零（localRed/localBlue = 0）。
void handleButtonLong(uint8_t idx) {
    switch (idx) {
        case 0: adjustRed(-1); break;
        case 1: adjustRed(-2); break;
        case 2: adjustBlue(-1); break;
        case 3: adjustBlue(-2); break;
        case 4:
            Serial.println("BUTTON: long-submit → clear round (red/blue → 0)");
            resetLocalScore();
            break;
        default: break;
    }
}

// 在哪些状态下应该屏蔽按键？
// 设计文档第 9 节：
//   - UNASSIGNED：未绑定时编辑分数没意义，屏蔽。
//   - SENDING（有挂起提交）：屏蔽，避免一边重传一边改分。
//   - LOCKED：本轮已确认，屏蔽（需等服务端 next-round 解锁）。
// ERROR 状态目前用 hold overlay 表达、不持久，按键不在此屏蔽。
bool buttonsActive() {
    if (currentClientId == CLIENT_ID_UNASSIGNED) return false;
    if (pendingMsgId != 0) return false;
    if (lockedForCurrentRound) return false;
    return true;
}

// 扫描所有按键，做去抖与短按/长按事件分发。
// 在 loop 里每轮调用。每个按键独立维护去抖与长按计时，状态间互不影响。
// 事件触发时机：
//   - 短按事件：在释放沿（HIGH→LOW→稳定 HIGH 之后），如果这次按下期间没有触发过长按。
//   - 长按事件：稳定 LOW 持续达到 BUTTON_LONG_PRESS_MS 时一次性触发；标记 longPressFired，
//     避免释放沿再额外触发短按。
// 按键被屏蔽（!buttonsActive）时仍然继续扫描状态机以维持电平一致，但短/长按事件被丢弃，
// 不会污染计分。
void pollButtons() {
    const unsigned long now = millis();
    for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
        ButtonState& b = buttons[i];
        const bool raw = digitalRead(BUTTON_PINS[i]) != LOW;  // true=HIGH=松开，false=LOW=按下

        if (raw != b.lastRawLevel) {
            b.lastRawLevel = raw;
            b.lastRawChangeMs = now;
        }

        if (now - b.lastRawChangeMs >= BUTTON_DEBOUNCE_MS && raw != b.stableLevel) {
            b.stableLevel = raw;
            if (!raw) {
                // 按下沿（高→低）
                b.pressedAtMs = now;
                b.longPressFired = false;
            } else {
                // 释放沿（低→高）
                if (!b.longPressFired && buttonsActive()) {
                    handleButtonShort(i);
                }
            }
        }

        // 稳定 LOW 持续达到阈值，且本次按下还没触发过长按 → 触发长按事件。
        if (!b.stableLevel && !b.longPressFired &&
            now - b.pressedAtMs >= BUTTON_LONG_PRESS_MS) {
            b.longPressFired = true;
            if (buttonsActive()) {
                handleButtonLong(i);
            }
        }
    }
}

// 处理 "submit" 命令。两种形式：
//   submit                 → 使用当前 localRed/localBlue（与按键"短按提交"等价）
//   submit <red> <blue>    → 先把分数 set 到 localRed/localBlue，再触发提交（调试快捷方式）
// 任何形式下 red/blue 都必须在 0..99 范围内，否则提示用法不变更状态。
void handleSubmitCommand(const String args[], uint8_t argc) {
    int red = localRed;
    int blue = localBlue;

    if (argc == 2) {
        if (!ScoreProtocol::parseIntInRange(args[0], 0, 99, red) ||
            !ScoreProtocol::parseIntInRange(args[1], 0, 99, blue)) {
            Serial.println("submit: red/blue must be in 0..99");
            return;
        }
        localRed = red;
        localBlue = blue;
        updateDisplay();
    } else if (argc != 0) {
        Serial.println("Usage: submit                 # 使用当前红蓝分数提交");
        Serial.println("       submit <red> <blue>    # 先 set 再提交，0..99");
        return;
    }

    tryQueueSubmit(red, blue);
}

// 从 Serial（调试串口）按字节读入命令行，遇到 '\r' 或 '\n' 视为一行结束并解析。
// 命令格式：
//   submit <red 0-99> <blue 0-99>
//   show
// 终端兼容性与服务端一致：接受 CR/LF/CRLF，解析前 trim 整行，
// 解析前回显 "CMD: ..." 便于诊断异常输入。
// 输入超过 80 字节直接丢弃，避免缓冲区无限增长。
void handleSerialCommand() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());

        if (c == '\r' || c == '\n') {
            String line = serialLine;
            serialLine = "";

            line.trim();
            if (line.length() == 0) {
                continue;
            }

            Serial.print("CMD: ");
            Serial.println(line);

            constexpr uint8_t MAX_TOKENS = 4;
            String tokens[MAX_TOKENS];
            const uint8_t n = tokenizeBySpace(line, tokens, MAX_TOKENS);

            if (n == 0) {
                continue;
            }

            if (tokens[0] == "submit") {
                handleSubmitCommand(&tokens[1], static_cast<uint8_t>(n - 1));
            } else if (tokens[0] == "show") {
                printClientState();
            } else {
                Serial.print("Unknown command: ");
                Serial.println(tokens[0]);
                Serial.println("Available: submit <red> <blue> / show");
            }

            continue;
        }

        if (serialLine.length() < 80) {
            serialLine += c;
        } else {
            serialLine = "";
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

    // 用 MAC + millis 给 PRNG 播种。
    // 不同板的 MAC 不一样，所以多台裁判机同时发 SUBMIT 时各自的随机退避也会错开，
    // 设计文档第 8 节"随机退避"防碰撞机制依赖这个独立性。
    randomSeed(static_cast<unsigned long>(ESP.getEfuseMac() & 0xFFFFFFFFULL) ^ millis());

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

    // 5 个录分按键：INPUT_PULLUP，按下时由按键拉到 GND（设计文档 4.5 节）。
    for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }

    Serial1.begin(LORA_UART_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    delay(500);

    // 初始化数码管：满亮度（7）并按当前逻辑状态刷一次，让上电那一刻就有正确显示。
    display.setBrightness(7);
    display.clear();
    updateDisplay();

    // 低电量 LED 输出，默认熄灭。
    pinMode(BATTERY_LOW_LED_PIN, OUTPUT);
    digitalWrite(BATTERY_LOW_LED_PIN, LOW);

    // 电池 ADC：11dB 衰减让节点电压（满电约 2.1V）落在量程内；先采一次，保证首帧 HELLO 带真实电压。
    analogReadResolution(12);
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    batteryMv = readBatteryMv();
    Serial.print("Battery: ");
    Serial.print(batteryMv);
    Serial.println("mV");

    // 启动后立刻发一次 HELLO，告诉服务端本机已上线（无论已绑定还是 UNASSIGNED）。
    // 后续保活由 loop 的 HEARTBEAT 定时器承担。
    sendHello();

    Serial.println("E22 UART transparent ready");
    Serial.println("Serial commands: submit <red 0-99> <blue 0-99> / show");
}

// 设计文档第 9 节定义的心跳周期。
// 未提交时较密（10s），让服务端能较快感知掉线；
// 已提交锁定后稍疏（15s），因为锁定期间裁判机基本不参与业务，频繁通信浪费信道。
constexpr unsigned long HEARTBEAT_INTERVAL_IDLE_MS = 10000;
constexpr unsigned long HEARTBEAT_INTERVAL_LOCKED_MS = 15000;

// Arduino 主循环，会被反复调用。
// 每轮做这些事：
//   1) 处理 LoRa 入站帧（STATUS/ACK/ASSIGN/UNBIND）
//   2) 处理本机串口命令（submit/show）
//   3) 扫描按键
//   4) 按时机驱动挂起的 SUBMIT 重传
//   5) 周期采样电池电压，并按电量驱动低电量 LED
//   6) 按 10s（未锁定）或 15s（已锁定）间隔发 HEARTBEAT（非阻塞定时）
//   7) 兜底刷新 TM1637 显示：状态变化点已主动调 updateDisplay，这里只是兜住"hold 到期"
//      和偶发漏调用，保证显示与逻辑状态最多相差 DISPLAY_REFRESH_INTERVAL_MS。
// lastHeartbeatMs 是全局，scheduleHeartbeatSoon 把它清 0 即可在下个 loop 拿到立即触发。
void loop() {
    handleLoraInput();
    handleSerialCommand();
    pollButtons();
    drivePendingSubmit();
    sampleBatteryIfDue();
    updateBatteryLowLed();

    const unsigned long interval =
        lockedForCurrentRound ? HEARTBEAT_INTERVAL_LOCKED_MS : HEARTBEAT_INTERVAL_IDLE_MS;
    if (millis() - lastHeartbeatMs >= interval) {
        lastHeartbeatMs = millis();
        sendHeartbeat();
    }

    if (millis() - lastDisplayRefreshMs >= DISPLAY_REFRESH_INTERVAL_MS) {
        updateDisplay();
    }
}
