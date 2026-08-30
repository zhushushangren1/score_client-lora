// 裁判端 LoRa UART 链路模块接口。
// 对上层隐藏 E22 串口细节，只暴露读一帧和发送各类协议帧的函数。
#pragma once

#include <Arduino.h>

// 初始化裁判端 E22 LoRa 透传串口。
// 设置 M0/M1/AUX 引脚，切入普通透传模式，并用 Serial1 打开 GPIO41/GPIO40。
void setupClientLoraLink();

// 从 E22 UART 读取一帧文本协议。
// frameText：输出参数，成功时为不含 CR/LF 的完整 CSV+CRC 行。
// 返回：true=读到完整帧；false=当前没有完整帧。非阻塞。
bool readLoraFrame(String& frameText);

// Enable or disable LoRa UART diagnostics printed from updateClientLoraDebug().
void setClientLoraDebugEnabled(bool enabled);

// Print one diagnostic snapshot each second while debug is enabled.
// TX counters and AUX samples help confirm whether the client E22 reacts to UART writes.
void updateClientLoraDebug();

// 发送一行已组好 CRC、并带换行的协议文本到 E22。
void sendLoraLine(const String& text);

// 发送 HELLO，告诉服务端本机 deviceId、当前 clientId 和电池电压。
void sendHello();

// 发送 HEARTBEAT，周期性上报在线状态、电池电压和当前 clientId。
void sendHeartbeat();

// 按当前锁定状态选择 10s/15s 周期，到了时间就发送 HEARTBEAT。
void sendHeartbeatIfDue();

// 回应 ASSIGN。clientId：本机已保存/确认的 client1/client2/client3。
void sendAssignAck(const String& clientId);

// 回应 UNBIND，表示本机已清除本地绑定。
void sendUnbindAck();

// 发送当前 pending SUBMIT。调用前 pendingRoundId/pendingMsgId/pendingRed/pendingBlue 必须已填好。
void sendSubmit();

// 改写 E22 空中速率（配置模式写 SPED 寄存器）。
// rateText：目标空中速率，取值 "2.4"/"4.8"/"9.6"/"19.2"/"38.4"/"62.5"（kbps）；
//           传空串则只读取并打印当前配置、不写入。
// 只修改 SPED 低 3 位（空中速率），保留 UART 波特率/校验和地址/信道/功率不变。
void configureClientLoraAirRate(const String& rateText);

// 上电自动把 E22 空中速率调整到编译期目标值（见 ClientLoraLink.cpp 中 TARGET_AIR_RATE_KPBS）。
// 与 configureClientLoraAirRate 共用同一套读-比较-写逻辑：当前值不同才写入并保存到 flash。
// 应在 setupClientLoraLink() 之后、发送任何协议帧之前调用。
void ensureClientLoraAirRate();
