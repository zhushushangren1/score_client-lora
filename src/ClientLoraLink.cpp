// 裁判端 LoRa UART 链路模块实现。
// 负责 E22 透传串口初始化、按行收发协议帧，以及构造 HELLO/HEARTBEAT/SUBMIT 等上行帧。
#include "ClientLoraLink.h"

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

// 心跳周期：未锁定时更频繁，已提交锁定后降低通信占用。
constexpr unsigned long HEARTBEAT_INTERVAL_IDLE_MS = 10000;
constexpr unsigned long HEARTBEAT_INTERVAL_LOCKED_MS = 15000;

String loraLine;

void waitForLoraReady() {
    const unsigned long start = millis();
    // AUX 为 LOW 表示 E22 正忙。最多等 1 秒，避免硬件异常时主循环永久卡死。
    while (digitalRead(LORA_AUX_PIN) == LOW && millis() - start < 1000) {
        delay(1);
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
    // 模式脚设置后给 E22 一个稳定时间，再开始发 HELLO。
    delay(500);
}

// 功能：从 E22 UART 读取一行完整协议帧。
// 参数 frameText：输出参数；成功时写入不含 \r/\n 的完整一行。
// 返回：true 表示读到一帧；false 表示当前没有完整帧。
bool readLoraFrame(String& frameText) {
    while (Serial1.available() > 0) {
        const char c = static_cast<char>(Serial1.read());
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
            return true;
        }

        if (loraLine.length() < LORA_LINE_MAX) {
            loraLine += c;
        } else {
            // 超长行大概率已经失去同步，丢弃整行等待下一个换行重新同步。
            loraLine = "";
        }
    }
    return false;
}

void sendLoraLine(const String& text) {
    // 发送前看 AUX，减少在 E22 忙时继续塞 UART 造成的丢字节风险。
    waitForLoraReady();
    // buildFrame 已经带末尾 '\n'，这里不能再 println，否则对端会读到额外空行。
    Serial1.print(text);
    Serial.print("LoRa TX: ");
    Serial.print(text);
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
    // HEARTBEAT 会持续上报 clientId，服务端可据此纠正客户端残留的旧绑定。
    String fields[] = {
        "HEARTBEAT",
        deviceId,
        currentClientId,
        String(batteryMv),
        String(localHeartbeatMsgId)
    };
    sendLoraLine(ScoreProtocol::buildFrame(fields, 5));
}

void sendHeartbeatIfDue() {
    // 已锁定时降低心跳频率，减少本轮已经提交后的无线占用。
    const unsigned long interval =
        lockedForCurrentRound ? HEARTBEAT_INTERVAL_LOCKED_MS : HEARTBEAT_INTERVAL_IDLE_MS;
    if (millis() - lastHeartbeatMs >= interval) {
        // 先更新时间戳再发送，避免发送过程短暂阻塞导致下一轮循环立即重复发送。
        lastHeartbeatMs = millis();
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
