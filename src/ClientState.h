#pragma once

#include <Arduino.h>

// 与服务端协议保持一致的“未绑定”标识。
// currentClientId 等于该值时，裁判端只会上报 HELLO/HEARTBEAT，不允许提交比分。
constexpr const char* CLIENT_ID_UNASSIGNED = "UNASSIGNED";

// 本机唯一设备号。setup() 中由 ESP32 efuse MAC 的低 32 位生成，格式为 8 位大写十六进制。
extern String deviceId;

// 当前服务端分配给本机的裁判槽位名。合法值为 client1/client2/client3；未绑定时为 UNASSIGNED。
extern String currentClientId;

// 最近一次 ADC 换算出的电池电压，单位毫伏。HELLO/HEARTBEAT/SUBMIT 都会上报这个值。
extern int batteryMv;

// 当前本地正在编辑的红蓝分数。按键增减只改这里；提交成功后由服务端 ACK 锁定当前轮。
extern int localRed;
extern int localBlue;

// 本机递增的 SUBMIT 消息号。每次新提交 +1，不持久化；服务端以 deviceId+roundId+msgId 去重。
extern unsigned long localMsgId;

// 最近一次从服务端 STATUS 帧同步到的轮号。为 0 表示还没收到 STATUS，此时禁止提交。
extern unsigned long currentServerRoundId;

// 本轮是否已经被服务端确认。锁定后禁止继续改分/提交，直到服务端发来新的 roundId。
extern bool lockedForCurrentRound;
extern unsigned long lockedRoundId;

// 当前挂起的提交。pendingMsgId == 0 表示没有提交在等待 ACK 或重传。
extern unsigned long pendingMsgId;
extern unsigned long pendingRoundId;
extern int pendingRed;
extern int pendingBlue;
extern uint8_t pendingRetries;
extern unsigned long pendingNextSendMs;

// HEARTBEAT 独立消息号和最近一次发送时间。心跳 msgId 只用于日志，不参与服务端去重。
extern unsigned long localHeartbeatMsgId;
extern unsigned long lastHeartbeatMs;

String makeDeviceIdFromMac();
void loadClientIdFromNvs();
void saveClientIdToNvs(const String& clientId);
void clearClientIdFromNvs();
bool isValidClientId(const String& id);

void scheduleHeartbeatSoon();
void clearPendingSubmit();
