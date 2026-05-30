#include "ClientState.h"

#include <Preferences.h>

namespace {

// Preferences 命名空间：ESP32 NVS 要求命名空间不超过 15 字节。
constexpr const char* NVS_NAMESPACE = "score_client";

// NVS 中保存服务端分配的 clientId。未绑定时删除该 key，而不是写入 UNASSIGNED。
constexpr const char* NVS_KEY_CLIENT_ID = "clientId";

// 全局 Preferences 实例。setup() 加载 clientId 时 begin，一直保持打开，后续 ASSIGN/UNBIND 可直接写。
Preferences prefs;

}  // namespace

String deviceId;
String currentClientId;
int batteryMv = 0;

int localRed = 0;
int localBlue = 0;

unsigned long localMsgId = 0;
unsigned long currentServerRoundId = 0;
bool lockedForCurrentRound = false;
unsigned long lockedRoundId = 0;

unsigned long pendingMsgId = 0;
unsigned long pendingRoundId = 0;
int pendingRed = 0;
int pendingBlue = 0;
uint8_t pendingRetries = 0;
unsigned long pendingNextSendMs = 0;

unsigned long localHeartbeatMsgId = 0;
unsigned long lastHeartbeatMs = 0;

// 功能：从 ESP32 efuse MAC 生成裁判端 deviceId。
// 返回值：8 位大写十六进制字符串，例如 "A1B2C3D4"。
// 说明：只取 MAC 低 32 位是为了让显示屏能展示后 4 位短码，同时保持足够低的重复概率。
String makeDeviceIdFromMac() {
    const uint64_t mac = ESP.getEfuseMac();
    // 只取低 32 位并格式化成 8 位十六进制，日志和网页都更容易手工比对。
    const uint32_t low32 = static_cast<uint32_t>(mac & 0xFFFFFFFFULL);
    char buffer[9];
    snprintf(buffer, sizeof(buffer), "%08X", low32);
    return String(buffer);
}

// 功能：从 NVS 读取已绑定的 clientId。
// 副作用：打开 Preferences，并写入全局 currentClientId。
// 缺省行为：如果 NVS 没有 clientId，则进入 UNASSIGNED 未绑定状态。
void loadClientIdFromNvs() {
    // false 表示读写模式；后续 ASSIGN/UNBIND 会继续通过同一个 prefs 写入。
    prefs.begin(NVS_NAMESPACE, false);
    const String stored = prefs.getString(NVS_KEY_CLIENT_ID, "");
    // NVS 中没有 clientId 时统一使用协议约定的 UNASSIGNED 文本。
    currentClientId = stored.length() == 0 ? String(CLIENT_ID_UNASSIGNED) : stored;
}

// 功能：保存服务端下发的新 clientId。
// 参数 clientId：调用方应先校验为 client1/client2/client3；本函数只负责持久化和同步内存。
void saveClientIdToNvs(const String& clientId) {
    // 先写 NVS 再同步内存，保证断电重启后还能保持服务端分配的裁判位。
    prefs.putString(NVS_KEY_CLIENT_ID, clientId);
    currentClientId = clientId;
}

// 功能：清除本地绑定状态。
// 用途：收到服务端 UNBIND，或服务端发现本机冒用旧 clientId 后要求纠正。
void clearClientIdFromNvs() {
    // 删除 key 比写入 UNASSIGNED 更干净，也避免以后合法值变更时产生歧义。
    prefs.remove(NVS_KEY_CLIENT_ID);
    currentClientId = CLIENT_ID_UNASSIGNED;
}

// 功能：判断字符串是否是服务端支持的裁判槽位。
// 参数 id：待校验的 clientId 字符串。
// 返回：true 表示 client1/client2/client3；false 表示非法或 UNASSIGNED。
bool isValidClientId(const String& id) {
    // 当前系统固定 3 个裁判位；新增裁判位时这里和服务端 BINDING_SLOT_NAMES 要同步改。
    return id == "client1" || id == "client2" || id == "client3";
}

// 功能：让下一次 loop 尽快发送 HEARTBEAT。
// 实现：把 lastHeartbeatMs 清零，下个 sendHeartbeatIfDue() 会认为已经超过周期。
void scheduleHeartbeatSoon() {
    // lastHeartbeatMs=0 对上电早期也安全；millis()-0 很快会超过心跳间隔。
    lastHeartbeatMs = 0;
}

// 功能：清掉当前挂起的 SUBMIT 状态。
// 用途：ACK 匹配成功、ACK 返回错误、重试耗尽、或其他流程需要取消 pending submit。
void clearPendingSubmit() {
    // pendingMsgId 是“是否有挂起提交”的哨兵，清零后按钮和新提交会重新可用。
    pendingMsgId = 0;
    pendingRoundId = 0;
    pendingRed = 0;
    pendingBlue = 0;
    pendingRetries = 0;
    pendingNextSendMs = 0;
}
