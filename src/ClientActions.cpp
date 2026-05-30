#include "ClientActions.h"

#include "ClientDisplay.h"
#include "ClientLoraLink.h"
#include "ClientState.h"

namespace {

// SUBMIT 可靠性参数：
// 首发前随机退避 0~300ms，减少多台裁判同时提交时的空中碰撞；
// 未收到 ACK 时每 300~1200ms 重发一次，最多 5 次。
constexpr unsigned long SUBMIT_INITIAL_BACKOFF_MAX_MS = 300;
constexpr unsigned long SUBMIT_RETRY_BACKOFF_MIN_MS = 300;
constexpr unsigned long SUBMIT_RETRY_BACKOFF_MAX_MS = 1200;
constexpr uint8_t SUBMIT_MAX_RETRIES = 5;

int clampScore(int value) {
    // 裁判端本地只允许编辑 0..99，和服务端 SUBMIT 校验范围保持一致。
    if (value < 0) {
        return 0;
    }
    if (value > 99) {
        return 99;
    }
    return value;
}

}  // namespace

bool tryQueueSubmit(int red, int blue) {
    // 未绑定时没有合法 clientId，服务端无法判断这是 client1/client2/client3 的哪一个。
    if (currentClientId == CLIENT_ID_UNASSIGNED) {
        Serial.println("submit: not bound yet (still UNASSIGNED), wait for server ASSIGN");
        return false;
    }
    // currentServerRoundId 只能来自服务端 STATUS。没收到 STATUS 前提交会有轮次歧义。
    if (currentServerRoundId == 0) {
        Serial.println("submit: no STATUS received yet, cannot determine roundId");
        return false;
    }
    // pendingMsgId 非 0 表示上一次 SUBMIT 仍在等待 ACK 或重传，不能并发发第二个提交。
    if (pendingMsgId != 0) {
        Serial.print("submit: previous submit still pending (msgId=");
        Serial.print(pendingMsgId);
        Serial.println("), wait or it will fail after 5 retries");
        return false;
    }
    // 一轮只允许接受一次提交。ACK 成功后会锁定，直到服务端 STATUS 宣告新 roundId。
    if (lockedForCurrentRound && lockedRoundId == currentServerRoundId) {
        Serial.print("submit: already locked for round ");
        Serial.print(lockedRoundId);
        Serial.println(", wait for next-round");
        return false;
    }

    // localMsgId 是本机递增消息号，服务端用 deviceId + roundId + msgId 区分重复包。
    localMsgId++;
    pendingMsgId = localMsgId;
    // 把当前提交快照写入 pending，后续重传必须重发同一个 round/msg/score。
    pendingRoundId = currentServerRoundId;
    pendingRed = red;
    pendingBlue = blue;
    pendingRetries = 0;
    // 首发也做随机退避，避免几台裁判在同一时刻按下提交造成 LoRa 空中碰撞。
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

    updateDisplay();
    return true;
}

void drivePendingSubmit() {
    // 没有 pending 提交时立即返回，让 loop 继续处理 LoRa/Web/按键等任务。
    if (pendingMsgId == 0) {
        return;
    }
    // 使用有符号差值比较 millis()，这样 millis 溢出回绕时判断仍然可靠。
    if (static_cast<long>(millis() - pendingNextSendMs) < 0) {
        return;
    }
    // 重试次数耗尽仍未收到 ACK，认为本次提交失败并给数码管显示 Err。
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

    // 发送的是 pending 快照，不读取 localRed/localBlue，避免用户后续改分影响重传内容。
    sendSubmit();
    pendingRetries++;
    // 每次重传间隔重新随机，降低持续碰撞的概率。
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

void maybeUnlockOnRoundChange() {
    // 锁定只针对 lockedRoundId。一旦服务端 STATUS 轮号变化，就可以开始编辑新一轮。
    if (lockedForCurrentRound && lockedRoundId != currentServerRoundId) {
        Serial.print("Round changed (locked=");
        Serial.print(lockedRoundId);
        Serial.print(", current=");
        Serial.print(currentServerRoundId);
        Serial.println("), unlocked");
        lockedForCurrentRound = false;
        lockedRoundId = 0;
        // 新一轮从 0:0 开始录入，避免误把上一轮本地显示值带到下一轮。
        localRed = 0;
        localBlue = 0;
        updateDisplay();
    }
}

void resetLocalScore() {
    // 长按提交键触发，只清本地编辑值，不影响已经被服务端 ACK 的历史轮次。
    localRed = 0;
    localBlue = 0;
    updateDisplay();
}

void adjustRed(int delta) {
    // delta 来自按键短按/长按，可能为正也可能为负，最终统一钳位。
    localRed = clampScore(localRed + delta);
    updateDisplay();
}

void adjustBlue(int delta) {
    // 修改后立即刷新数码管，让按键反馈不依赖周期性 refreshDisplayIfDue。
    localBlue = clampScore(localBlue + delta);
    updateDisplay();
}

bool buttonsActive() {
    // 未绑定时不允许实体按键改分/提交，避免拿不到合法轮次和裁判位。
    if (currentClientId == CLIENT_ID_UNASSIGNED) {
        return false;
    }
    // 等待 ACK 期间禁止继续改分，保证 pending SUBMIT 的画面和实际发送内容一致。
    if (pendingMsgId != 0) {
        return false;
    }
    // 当前轮已经提交成功后锁定，直到服务端进入下一轮。
    if (lockedForCurrentRound) {
        return false;
    }
    return true;
}
