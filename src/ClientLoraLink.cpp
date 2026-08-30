// 裁判端 LoRa UART 链路模块实现。
// 负责 E22 透传串口初始化、按行收发协议帧，以及构造 HELLO/HEARTBEAT/SUBMIT 等上行帧。
//
// 收发分离设计：Serial1.flush() 会阻塞（9600 baud 下一帧约 40ms），若在 loop 里直接发送，
// 按键扫描会被打断、快速连按时丢按钮事件。因此所有 LoRa 发送都投递到一个 FreeRTOS 队列，
// 由独立的 loraTxTask 真正写 UART；主循环只负责按键/接收/组装帧，按钮保持最高优先级。
#include "ClientLoraLink.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <ScoreProtocol.h>

#include "ClientState.h"

namespace {

// E22-400T22D UART 透传接线。TX/RX 名称以 ESP32 Serial1 方向命名：
// LORA_TX_PIN 是 ESP32 发往 E22 RXD 的 TX 脚；LORA_RX_PIN 是 ESP32 接收 E22 TXD 的 RX 脚。
constexpr int LORA_TX_PIN = 40;
constexpr int LORA_RX_PIN = 41;
constexpr int LORA_AUX_PIN = 42;
constexpr int LORA_M0_PIN = 38;
constexpr int LORA_M1_PIN = 39;
// E22 出厂默认透传波特率为 9600；两端必须一致，否则会看到乱码或收不到完整帧。
constexpr uint32_t LORA_UART_BAUD = 9600;
// 单帧最长 120 字节，足够容纳当前 CSV+CRC 协议；超长通常说明串口乱码或丢换行。
constexpr size_t LORA_LINE_MAX = 120;

// 心跳周期：正常情况下客户端由服务端 POLL 驱动上报（见 handlePoll），
// 自主心跳只作兜底——当服务端停止轮询（重启/异常）时，5s 一次维持在线，
// 避免被误判离线。锁定后兜底降频到 10 秒。
constexpr unsigned long HEARTBEAT_INTERVAL_IDLE_MS = 5000;
constexpr unsigned long HEARTBEAT_INTERVAL_LOCKED_MS = 10000;
// 兜底心跳附加 0~100ms 随机抖动，用于对抗多台裁判长期运行后晶振漂移导致的相位重聚。
constexpr unsigned long HEARTBEAT_JITTER_MS = 100;
// 兜底心跳的相位偏移：client1/2/3 分别错开 0/333/666ms，让三台兜底心跳也不重叠。
constexpr unsigned long HEARTBEAT_SLOT_OFFSET_MS = 333;

unsigned long heartbeatPhaseOffsetMs() {
    if (currentClientId == "client2") {
        return HEARTBEAT_SLOT_OFFSET_MS;
    }
    if (currentClientId == "client3") {
        return HEARTBEAT_SLOT_OFFSET_MS * 2;
    }
    return 0;
}

// 发送队列长度和发送任务栈。队列满时丢弃新帧，丢失由心跳周期上报和 SUBMIT 重传兜底。
constexpr uint8_t LORA_TX_QUEUE_LENGTH = 8;
constexpr uint32_t LORA_TX_TASK_STACK = 4096;
constexpr UBaseType_t LORA_TX_TASK_PRIORITY = 1;

// 队列元素：一帧不含换行的完整文本（含末尾 CRC）。用定长 char 数组跨任务拷贝，避免 String 生命周期问题。
struct TxFrame {
    char text[LORA_LINE_MAX + 1];
};

QueueHandle_t loraTxQueue = nullptr;

// 配置模式进行期间置位，loraTxTask 看到后会暂停发送，避免把普通帧写进配置模式的 UART。
volatile bool loraConfigInProgress = false;

String loraLine;
bool loraDebugEnabled = false;
unsigned long lastLoraDebugPrintMs = 0;
uint32_t loraTxCount = 0;
uint32_t loraRxRawByteCount = 0;
uint32_t loraRxFrameCount = 0;
uint32_t loraRxOverflowCount = 0;
int lastAuxBeforeTx = -1;
int lastAuxAfterTx = -1;

void waitForLoraReady() {
    const unsigned long start = millis();
    // AUX 为 LOW 表示 E22 正忙。最多等 1 秒，避免硬件异常时发送任务永久卡死。
    while (digitalRead(LORA_AUX_PIN) == LOW && millis() - start < 1000) {
        delay(1);
    }
}

// 发送任务：从队列取帧并真正写 Serial1。阻塞在这里，不会影响主循环的按键扫描。
void loraTxTask(void* arg) {
    (void)arg;
    TxFrame frame;
    for (;;) {
        if (xQueueReceive(loraTxQueue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        // 配置模式改写寄存器期间暂停发送，避免普通帧被模块当成配置命令解析。
        while (loraConfigInProgress) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        waitForLoraReady();
        lastAuxBeforeTx = digitalRead(LORA_AUX_PIN);
        // buildFrame 已经带末尾 '\n'，这里用 print 而非 println，避免对端读到额外空行。
        Serial1.print(frame.text);
        Serial1.flush();
        loraTxCount++;
        lastAuxAfterTx = digitalRead(LORA_AUX_PIN);
        Serial.print("LoRa TX: ");
        Serial.print(frame.text);
    }
}

}  // namespace

void setupClientLoraLink() {
    pinMode(LORA_M0_PIN, OUTPUT);
    pinMode(LORA_M1_PIN, OUTPUT);
    // AUX 只读输入，开内部上拉可避免 AUX 未焊好/悬空时误判为一直忙。
    pinMode(LORA_AUX_PIN, INPUT_PULLUP);

    // M0=0、M1=0 是 E22 普通透明传输模式，串口写入内容会直接走 LoRa 发射。
    digitalWrite(LORA_M0_PIN, LOW);
    digitalWrite(LORA_M1_PIN, LOW);

    // Serial1.begin(rx, tx)：这里 RX=GPIO41 接 E22 TXD，TX=GPIO40 接 E22 RXD。
    Serial1.begin(LORA_UART_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

    // 创建发送队列和发送任务，把阻塞的 UART 写移出主循环。
    if (loraTxQueue == nullptr) {
        loraTxQueue = xQueueCreate(LORA_TX_QUEUE_LENGTH, sizeof(TxFrame));
    }
    if (loraTxQueue != nullptr) {
        xTaskCreate(loraTxTask, "loraTx", LORA_TX_TASK_STACK, nullptr, LORA_TX_TASK_PRIORITY, nullptr);
    }

    // 模式脚设置后给 E22 一个稳定时间，再开始发 HELLO。
    delay(500);
}

// 功能：从 E22 UART 读取一行完整协议帧。
// 参数 frameText：输出参数；成功时写入不含 \r/\n 的完整一行。
// 返回：true 表示读到一帧；false 表示当前没有完整帧。
bool readLoraFrame(String& frameText) {
    while (Serial1.available() > 0) {
        const char c = static_cast<char>(Serial1.read());
        loraRxRawByteCount++;
        if (c == '\r') {
            // 兼容 CRLF：协议帧以 '\n' 结束，'\r' 只作为行尾装饰直接忽略。
            continue;
        }

        if (c == '\n') {
            if (loraLine.length() == 0) {
                // 连续换行或空行不算协议帧，避免上层打印无意义的空 RX。
                continue;
            }
            // 收到换行才认为一帧完整；返回前清空缓冲，准备接下一帧。
            frameText = loraLine;
            loraLine = "";
            loraRxFrameCount++;
            return true;
        }

        if (loraLine.length() < LORA_LINE_MAX) {
            loraLine += c;
        } else {
            // 超长行大概率已经失去同步，丢弃整行等待下一个换行重新同步。
            loraLine = "";
            loraRxOverflowCount++;
        }
    }
    return false;
}

void setClientLoraDebugEnabled(bool enabled) {
    loraDebugEnabled = enabled;
    lastLoraDebugPrintMs = 0;
    Serial.print("LoRa debug ");
    Serial.println(enabled ? "ON" : "OFF");
}

void updateClientLoraDebug() {
    if (!loraDebugEnabled) {
        return;
    }

    const unsigned long now = millis();
    if (now - lastLoraDebugPrintMs < 1000) {
        return;
    }
    lastLoraDebugPrintMs = now;

    Serial.print("LoRa debug: tx=");
    Serial.print(loraTxCount);
    Serial.print(" rxRaw=");
    Serial.print(loraRxRawByteCount);
    Serial.print(" rxFrames=");
    Serial.print(loraRxFrameCount);
    Serial.print(" partialLen=");
    Serial.print(loraLine.length());
    Serial.print(" overflow=");
    Serial.print(loraRxOverflowCount);
    Serial.print(" avail=");
    Serial.print(Serial1.available());
    Serial.print(" AUX=");
    Serial.print(digitalRead(LORA_AUX_PIN));
    Serial.print(" lastAuxBeforeTx=");
    Serial.print(lastAuxBeforeTx);
    Serial.print(" lastAuxAfterTx=");
    Serial.print(lastAuxAfterTx);
    Serial.print(" M0=");
    Serial.print(digitalRead(LORA_M0_PIN));
    Serial.print(" M1=");
    Serial.print(digitalRead(LORA_M1_PIN));
    Serial.println();
}

void sendLoraLine(const String& text) {
    // 只负责把帧投递到发送队列，不在这里写 UART，保证主循环不被 Serial1.flush 阻塞。
    if (loraTxQueue == nullptr) {
        return;
    }
    TxFrame frame;
    const size_t len = text.length() < LORA_LINE_MAX ? text.length() : LORA_LINE_MAX;
    memcpy(frame.text, text.c_str(), len);
    frame.text[len] = '\0';
    // 队列满（发送过快）时丢弃，避免阻塞主循环；心跳周期上报和 SUBMIT 重传会兜底。
    if (xQueueSend(loraTxQueue, &frame, 0) != pdTRUE) {
        Serial.print("LoRa TX queue full, dropped: ");
        Serial.println(text);
    }
}

void sendHello() {
    // HELLO 字段：消息类型、设备号、当前绑定槽位、电池电压。
    // 未绑定时 currentClientId 为 UNASSIGNED，服务端会把 deviceId 放进未绑定列表。
    String fields[] = {
        "HELLO",
        deviceId,
        currentClientId,
        String(batteryMv)
    };
    sendLoraLine(ScoreProtocol::buildFrame(fields, 4));
}

void sendHeartbeat() {
    // 心跳 msgId 只用于日志观察，不参与服务端提交去重。
    localHeartbeatMsgId++;
    // HEARTBEAT 同时上报 clientId、电池和当前本地比分，让服务端用同一帧刷新
    // “在线状态”和“实时比分”，避免额外发 UPDATE 帧增加空中碰撞。
    String fields[] = {
        "HEARTBEAT",
        deviceId,
        currentClientId,
        String(batteryMv),
        String(localHeartbeatMsgId),
        String(localRed),
        String(localBlue)
    };
    sendLoraLine(ScoreProtocol::buildFrame(fields, 7));
}

void sendHeartbeatIfDue() {
    // 仅作兜底：服务端正常轮询时，handlePoll 会不断刷新 lastHeartbeatMs，
    // 这里 5s 周期不会触发。只有服务端停止轮询时才会自主发心跳维持在线。
    const unsigned long interval =
        lockedForCurrentRound ? HEARTBEAT_INTERVAL_LOCKED_MS : HEARTBEAT_INTERVAL_IDLE_MS;
    // 把本机相位折进虚拟时钟：client2/3 的兜底心跳比 client1 分别晚 333/666ms，
    // 让三台即使在服务端停止轮询后自主上报也保持错开。
    const unsigned long now = millis() + heartbeatPhaseOffsetMs();
    if (static_cast<long>(now - lastHeartbeatMs) >= static_cast<long>(interval)) {
        lastHeartbeatMs = now + random(HEARTBEAT_JITTER_MS + 1);
        sendHeartbeat();
    }
}

void sendAssignAck(const String& clientId) {
    // ASSIGN_ACK 只确认本机已经接受 clientId，服务端目前用于日志确认和现场排查。
    String fields[] = {
        "ASSIGN_ACK",
        deviceId,
        clientId
    };
    sendLoraLine(ScoreProtocol::buildFrame(fields, 3));
}

void sendUnbindAck() {
    // UNBIND_ACK 表示本机已经清除 NVS clientId，下次上电也会保持 UNASSIGNED。
    String fields[] = {
        "UNBIND_ACK",
        deviceId
    };
    sendLoraLine(ScoreProtocol::buildFrame(fields, 2));
}

void sendSubmit() {
    // SUBMIT 必须发送 pending 中保存的快照；ACK 匹配也依赖同一个 roundId/msgId。
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

// ---- E22 空中速率配置 ----
// 与服务端 LoraLink.cpp 中的逻辑一致：进配置模式改写 REG0 寄存器（0x03）低 3 位。
// 只改空中速率，保留 UART 波特率/校验和地址/信道/功率等其余设置不变。

constexpr uint8_t AIR_RATE_2400 = 0b010;
constexpr uint8_t AIR_RATE_4800 = 0b011;
constexpr uint8_t AIR_RATE_9600 = 0b100;
constexpr uint8_t AIR_RATE_19200 = 0b101;
constexpr uint8_t AIR_RATE_38400 = 0b110;
constexpr uint8_t AIR_RATE_62500 = 0b111;

uint8_t airRateBitsFromString(const String& s) {
    if (s == "2.4") return AIR_RATE_2400;
    if (s == "4.8") return AIR_RATE_4800;
    if (s == "9.6") return AIR_RATE_9600;
    if (s == "19.2") return AIR_RATE_19200;
    if (s == "38.4") return AIR_RATE_38400;
    if (s == "62.5") return AIR_RATE_62500;
    return 0xFF;
}

const char* airRateName(uint8_t bits) {
    switch (bits) {
        case AIR_RATE_2400: return "2.4k";
        case AIR_RATE_4800: return "4.8k";
        case AIR_RATE_9600: return "9.6k";
        case AIR_RATE_19200: return "19.2k";
        case AIR_RATE_38400: return "38.4k";
        case AIR_RATE_62500: return "62.5k";
        default: return "?";
    }
}

bool loraConfigWaitAux(unsigned long timeoutMs) {
    const unsigned long start = millis();
    while (digitalRead(LORA_AUX_PIN) == LOW && millis() - start < timeoutMs) {
        delay(1);
    }
    return digitalRead(LORA_AUX_PIN) == HIGH;
}

bool loraConfigReadBytes(uint8_t* buf, uint8_t len, unsigned long timeoutMs) {
    const unsigned long start = millis();
    uint8_t got = 0;
    while (got < len && millis() - start < timeoutMs) {
        while (got < len && Serial1.available() > 0) {
            buf[got++] = static_cast<uint8_t>(Serial1.read());
        }
        if (got < len) {
            delay(1);
        }
    }
    return got == len;
}

bool loraConfigReadSped(uint8_t& sped) {
    const uint8_t cmd[] = { 0xC1, 0x03, 0x01 };
    Serial1.write(cmd, sizeof(cmd));
    Serial1.flush();
    uint8_t resp[4];
    if (!loraConfigReadBytes(resp, 4, 300)) {
        Serial.println("airrate: read SPED timeout");
        return false;
    }
    if (resp[0] != 0xC1 || resp[1] != 0x03 || resp[2] != 0x01) {
        Serial.println("airrate: read SPED bad response");
        return false;
    }
    sped = resp[3];
    return true;
}

bool loraConfigWriteSped(uint8_t sped) {
    const uint8_t cmd[] = { 0xC2, 0x03, 0x01, sped };
    Serial1.write(cmd, sizeof(cmd));
    Serial1.flush();
    uint8_t resp[4];
    if (!loraConfigReadBytes(resp, 4, 300)) {
        Serial.println("airrate: write SPED timeout");
        return false;
    }
    if (resp[0] != 0xC1 || resp[1] != 0x03 || resp[2] != 0x01 || resp[3] != sped) {
        Serial.println("airrate: write SPED bad response");
        return false;
    }
    return true;
}

void configureClientLoraAirRate(const String& rateText) {
    // 置位保护标志，暂停 loraTxTask 发送，避免普通帧被模块当成配置命令。
    loraConfigInProgress = true;

    // 进配置模式前清空接收缓冲，避免透明模式残留字节干扰响应解析。
    while (Serial1.available() > 0) {
        Serial1.read();
    }

    // 进配置模式：E22-400T22D 配置模式 M1=1、M0=0。
    digitalWrite(LORA_M0_PIN, LOW);
    digitalWrite(LORA_M1_PIN, HIGH);

    if (!loraConfigWaitAux(2000)) {
        Serial.println("airrate: module did not enter config mode (AUX stuck LOW)");
    } else {
        delay(10);
        uint8_t curSped = 0;
        if (!loraConfigReadSped(curSped)) {
            // 读失败，不写入，直接回透传。
        } else {
            const uint8_t curAir = curSped & 0x07;
            Serial.print("airrate: current SPED=0x");
            Serial.print(curSped, HEX);
            Serial.print(" air=");
            Serial.println(airRateName(curAir));

            const uint8_t newAir = airRateBitsFromString(rateText);
            if (rateText.length() == 0) {
                // 无参数只读当前配置，不写入。
                Serial.println("airrate: read-only, no change");
            } else if (newAir == 0xFF) {
                Serial.println("airrate: unknown rate, use 2.4/4.8/9.6/19.2/38.4/62.5");
            } else if (newAir == curAir) {
                Serial.println("airrate: already at target, no change");
            } else {
                const uint8_t newSped = (curSped & 0xF8) | newAir;
                Serial.print("airrate: writing SPED=0x");
                Serial.print(newSped, HEX);
                Serial.print(" air=");
                Serial.println(airRateName(newAir));

                if (!loraConfigWriteSped(newSped)) {
                    // 写失败。
                } else {
                    uint8_t verifySped = 0;
                    if (loraConfigReadSped(verifySped) && (verifySped & 0x07) == newAir) {
                        Serial.println("airrate: OK, saved to module flash");
                    } else {
                        Serial.println("airrate: verify failed");
                    }
                }
            }
        }
    }

    // 回透传模式，并解除发送保护。
    digitalWrite(LORA_M1_PIN, LOW);
    digitalWrite(LORA_M0_PIN, LOW);
    loraConfigInProgress = false;
    delay(200);
    Serial.println("airrate: back to transparent mode (power-cycle module if link looks off)");
}

// 编译期目标空中速率，四台设备（1 服务端 + 3 客户端）必须完全一致。
// 上电时 ensureClientLoraAirRate() 会把它写入 E22 flash（仅当前值不同时才写）。
// 注意：首次写入后需断电重启一次，让 SX1268 射频芯片按新速率重新初始化。
constexpr const char* TARGET_AIR_RATE_KPBS = "19.2";

void ensureClientLoraAirRate() {
    configureClientLoraAirRate(TARGET_AIR_RATE_KPBS);
}
