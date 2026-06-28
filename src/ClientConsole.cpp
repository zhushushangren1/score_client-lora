// 裁判端 USB 串口调试命令实现。
// 负责解析 submit/show/lora-debug 命令，让未焊按键时也能通过串口提交、查看状态和排查 LoRa。
#include "ClientConsole.h"

#include <Arduino.h>
#include <ScoreProtocol.h>

#include "ClientActions.h"
#include "ClientDisplay.h"
#include "ClientLoraLink.h"
#include "ClientState.h"

namespace {

String serialLine;

uint8_t tokenizeBySpace(const String& line, String tokens[], uint8_t maxTokens) {
    uint8_t count = 0;
    int start = 0;
    const int len = line.length();

    while (start < len && count < maxTokens) {
        while (start < len && line[start] == ' ') {
            // 跳过连续空格，允许用户输入 "submit   5   7"。
            start++;
        }
        if (start >= len) {
            break;
        }

        int end = start;
        while (end < len && line[end] != ' ') {
            // 找到当前 token 的右边界；本命令集不支持带空格参数。
            end++;
        }
        tokens[count++] = line.substring(start, end);
        start = end;
    }
    return count;
}

void printClientState() {
    // show 命令输出完整状态，用于没有网页/按钮时判断绑定、轮次、pending 和显示状态。
    Serial.println("Client state:");
    Serial.print("  deviceId: ");
    Serial.println(deviceId);
    Serial.print("  clientId: ");
    Serial.println(currentClientId);
    Serial.print("  battery: ");
    Serial.print(batteryMv);
    Serial.println("mV");
    Serial.print("  currentServerRoundId: ");
    Serial.println(currentServerRoundId);
    Serial.print("  local score: red=");
    Serial.print(localRed);
    Serial.print(" blue=");
    Serial.println(localBlue);
    Serial.print("  locked: ");
    if (lockedForCurrentRound) {
        Serial.print("yes (round ");
        Serial.print(lockedRoundId);
        Serial.println(")");
    } else {
        Serial.println("no");
    }

    Serial.print("  pending: ");
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
        Serial.println(pendingRetries);
    }

    Serial.print("  ");
    printDisplayState();
}

void handleSubmitCommand(const String args[], uint8_t argc) {
    // 不带参数时使用当前按键编辑出的 localRed/localBlue。
    int red = localRed;
    int blue = localBlue;

    if (argc == 2) {
        // 带参数时先校验并同步到本地显示，再进入和按键相同的提交流程。
        if (!ScoreProtocol::parseIntInRange(args[0], 0, 99, red) ||
            !ScoreProtocol::parseIntInRange(args[1], 0, 99, blue)) {
            Serial.println("submit: red/blue must be in 0..99");
            return;
        }
        localRed = red;
        localBlue = blue;
        updateDisplay();
    } else if (argc != 0) {
        // 只接受 submit 或 submit <red> <blue>，避免误把残缺命令提交出去。
        Serial.println("Usage: submit");
        Serial.println("       submit <red> <blue>");
        return;
    }

    tryQueueSubmit(red, blue);
}

}  // namespace

void handleSerialCommand() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());

        if (c == '\r' || c == '\n') {
            // 收到行结束符后，把缓冲拷贝出来处理，并立刻清空准备下一条命令。
            String line = serialLine;
            serialLine = "";

            line.trim();
            if (line.length() == 0) {
                // CRLF 会触发两次，这里过滤掉第二个空行。
                continue;
            }

            Serial.print("CMD: ");
            Serial.println(line);

            constexpr uint8_t MAX_TOKENS = 4;
            String tokens[MAX_TOKENS];
            // 最多拆 4 个 token，当前最长命令 submit <red> <blue> 只需要 3 个。
            const uint8_t n = tokenizeBySpace(line, tokens, MAX_TOKENS);

            if (n == 0) {
                continue;
            }

            if (tokens[0] == "submit") {
                // &tokens[1] 把命令名之后的参数数组传给处理函数。
                handleSubmitCommand(&tokens[1], static_cast<uint8_t>(n - 1));
            } else if (tokens[0] == "show") {
                printClientState();
            } else if (tokens[0] == "lora-debug") {
                const bool enabled = !(n >= 2 && tokens[1] == "off");
                setClientLoraDebugEnabled(enabled);
            } else {
                Serial.print("Unknown command: ");
                Serial.println(tokens[0]);
                Serial.println("Available: submit <red> <blue> / show / lora-debug [on|off]");
            }

            continue;
        }

        if (serialLine.length() < 80) {
            serialLine += c;
        } else {
            // 行太长通常是误粘贴或串口乱码，清空缓冲避免后续解析半截命令。
            serialLine = "";
        }
    }
}
