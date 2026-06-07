// 裁判端 LoRa 入站协议处理实现。
// 负责解析服务端下发的 STATUS、ACK、ASSIGN、UNBIND，并更新本机绑定/提交/显示状态。
#include "ClientProtocolHandlers.h"

#include <Arduino.h>
#include <ScoreProtocol.h>

#include "ClientActions.h"
#include "ClientDisplay.h"
#include "ClientLoraLink.h"
#include "ClientState.h"

namespace {

void printParsedFrame(const ScoreProtocol::ParsedFrame& frame) {
    // 调试期保留完整字段打印，现场遇到 CRC 正确但业务不生效时可直接看字段顺序。
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

void handleAssign(const ScoreProtocol::ParsedFrame& frame) {
    if (frame.fieldCount != 3) {
        // ASSIGN,deviceId,clientId,CRC；字段数量不对说明协议版本或链路内容异常。
        Serial.println("ASSIGN: bad field count, ignored");
        return;
    }

    const String& targetDevice = frame.fields[1];
    const String& targetClient = frame.fields[2];

    if (targetDevice != deviceId) {
        // LoRa 是广播介质，别的裁判的 ASSIGN 本机也可能收到，必须按 deviceId 过滤。
        return;
    }
    if (!isValidClientId(targetClient)) {
        // 不接受 client1/client2/client3 之外的槽位，防止本机保存无法提交的非法身份。
        Serial.print("ASSIGN: invalid clientId '");
        Serial.print(targetClient);
        Serial.println("', ignored");
        return;
    }

    if (currentClientId == targetClient) {
        // 重复 ASSIGN 可能来自服务端重发；本机已是该身份时只回 ACK，不重复写 NVS。
        Serial.println("ASSIGN: clientId unchanged, ack only");
        sendAssignAck(targetClient);
        return;
    }

    saveClientIdToNvs(targetClient);
    Serial.print("ASSIGN: bound as ");
    Serial.println(targetClient);
    sendAssignAck(targetClient);
    scheduleHeartbeatSoon();

    // 数码管短暂显示 J1/J2/J3，便于现场确认服务端绑定到了哪一个裁判位。
    char holdText[5] = "J?  ";
    if (targetClient.length() > 0) {
        holdText[1] = targetClient[targetClient.length() - 1];
    }
    holdDisplay(holdText, 2000);
    updateDisplay();
}

void handleUnbind(const ScoreProtocol::ParsedFrame& frame) {
    if (frame.fieldCount != 2) {
        // UNBIND,deviceId,CRC；服务端不会附带 clientId，因为 deviceId 已能唯一定位本机。
        Serial.println("UNBIND: bad field count, ignored");
        return;
    }

    const String& targetDevice = frame.fields[1];
    if (targetDevice != deviceId) {
        // 忽略发给其他裁判端的解绑命令。
        return;
    }

    if (currentClientId == CLIENT_ID_UNASSIGNED) {
        // 幂等处理：本来就未绑定时仍回 ACK，让服务端知道命令已到达。
        Serial.println("UNBIND: already unassigned, ack only");
        sendUnbindAck();
        return;
    }

    clearClientIdFromNvs();
    // 解绑后保留本地分数显示逻辑会回到 deviceId 后四位，方便重新绑定。
    Serial.println("UNBIND: cleared local binding");
    sendUnbindAck();
    scheduleHeartbeatSoon();
    updateDisplay();
}

void handleStatus(const ScoreProtocol::ParsedFrame& frame) {
    if (frame.fieldCount != 6) {
        // STATUS,deviceId,clientId,roundId,open,submitted,CRC。
        Serial.println("STATUS: bad field count, ignored");
        return;
    }
    if (frame.fields[1] != deviceId) {
        // STATUS 是服务端点名某台设备的回包，广播收到的其他设备状态必须忽略。
        return;
    }

    unsigned long parsedRound = 0;
    if (!ScoreProtocol::parseUnsignedLong(frame.fields[3], parsedRound)) {
        // 轮号必须是纯数字；非法 STATUS 不应影响当前锁定状态。
        Serial.println("STATUS: bad roundId, ignored");
        return;
    }
    currentServerRoundId = parsedRound;
    // STATUS 是客户端获知“已经进入新一轮”的唯一来源，因此这里触发解锁检查。
    maybeUnlockOnRoundChange();

    const bool statusSubmitted = (frame.fields[5] == "1");
    bool displayNeedsRefresh = false;
    if (statusSubmitted) {
        // STATUS 的 submitted=1 表示服务端已经记录了本裁判本轮成绩。
        // 这是 ACK 丢包时的兜底确认：服务端日志已经收分，但裁判端没收到 ACK 时，
        // 不能继续重传到耗尽后显示 Err，而应该按服务端状态锁定本轮。
        if (pendingMsgId != 0 && pendingRoundId == currentServerRoundId) {
            Serial.println("STATUS: submitted=1 for pending submit, treating as accepted");
            clearPendingSubmit();
            displayNeedsRefresh = true;
        }
        if (!lockedForCurrentRound || lockedRoundId != currentServerRoundId) {
            lockedForCurrentRound = true;
            lockedRoundId = currentServerRoundId;
            displayNeedsRefresh = true;
        }
    }

    Serial.print("STATUS: round=");
    Serial.print(currentServerRoundId);
    Serial.print(" open=");
    Serial.print(frame.fields[4]);
    Serial.print(" submitted=");
    Serial.println(frame.fields[5]);

    if (displayNeedsRefresh) {
        updateDisplay();
    }
}

void handleAck(const ScoreProtocol::ParsedFrame& frame) {
    if (frame.fieldCount != 6) {
        // ACK,deviceId,clientId,roundId,msgId,status,CRC。
        Serial.println("ACK: bad field count, ignored");
        return;
    }
    if (frame.fields[1] != deviceId) {
        // 忽略其他裁判端的 ACK，防止误清本机 pending。
        return;
    }
    if (pendingMsgId == 0) {
        // 没有等待确认的提交时，ACK 可能是历史重包，不能改变当前状态。
        Serial.println("ACK: no pending submit, ignored");
        return;
    }

    unsigned long ackRound = 0;
    unsigned long ackMsgId = 0;
    if (!ScoreProtocol::parseUnsignedLong(frame.fields[3], ackRound) ||
        !ScoreProtocol::parseUnsignedLong(frame.fields[4], ackMsgId)) {
        // ACK 的轮号/消息号必须可解析，否则无法和 pending 精确匹配。
        Serial.println("ACK: bad roundId/msgId, ignored");
        return;
    }

    if (ackRound != pendingRoundId || ackMsgId != pendingMsgId) {
        // 只接受与当前 pending 完全一致的 ACK，避免旧 ACK 清掉新提交。
        return;
    }

    const String& status = frame.fields[5];
    if (status == "OK" || status == "OK_DUPLICATE" || status == "ERR_ALREADY_SUBMITTED") {
        // ERR_ALREADY_SUBMITTED 也锁定：服务端已经有该裁判本轮成绩，客户端不应再改分。
        lockedForCurrentRound = true;
        lockedRoundId = pendingRoundId;
        Serial.print("ACK ");
        Serial.print(status);
        Serial.print(", locked for round ");
        Serial.println(lockedRoundId);
        clearPendingSubmit();
        updateDisplay();
    } else {
        // ERR_BAD_ROUND 等错误说明本次提交没有被当前轮接受，清 pending 并提示用户重试/等状态同步。
        Serial.print("ACK ");
        Serial.print(status);
        Serial.print(", submit failed for round ");
        Serial.print(pendingRoundId);
        Serial.print(" msgId ");
        Serial.println(pendingMsgId);
        clearPendingSubmit();
        holdDisplay(" Err", 3000);
        updateDisplay();
    }
}

void dispatchFrame(const ScoreProtocol::ParsedFrame& frame) {
    // 客户端只处理服务端下发的四类消息；HELLO/HEARTBEAT/SUBMIT 这些上行消息忽略。
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
            break;
    }
}

}  // namespace

void handleLoraInput() {
    String line;
    while (readLoraFrame(line)) {
        // readLoraFrame 可能一次循环返回多帧，所以这里用 while 直到 Serial1 缓冲清空。
        Serial.print("LoRa RX: ");
        Serial.println(line);

        ScoreProtocol::ParsedFrame frame;
        if (ScoreProtocol::parseFrame(line, frame)) {
            // CRC 和消息类型都正确后才分发，业务层无需再重复校验 CRC。
            printParsedFrame(frame);
            dispatchFrame(frame);
        } else {
            // 乱码、CRC 不符、字段过长、未知消息类型都会走到这里。
            Serial.println("Invalid protocol frame");
        }
    }
}
