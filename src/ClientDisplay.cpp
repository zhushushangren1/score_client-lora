#include "ClientDisplay.h"

#include <TM1637Display.h>

#include "ClientState.h"

namespace {

// TM1637 四位数码管接线。CLK/DIO 与设计文档裁判端接线保持一致。
constexpr int TM1637_CLK_PIN = 48;
constexpr int TM1637_DIO_PIN = 47;

// 普通状态的兜底刷新周期。状态变化点会主动 updateDisplay，这里主要处理 hold 到期。
constexpr unsigned long DISPLAY_REFRESH_INTERVAL_MS = 200;

TM1637Display display(TM1637_CLK_PIN, TM1637_DIO_PIN);
// hold 到期时间点；0 表示当前没有临时覆盖显示。
unsigned long displayHoldUntilMs = 0;
// 4 位数码管覆盖文本，额外 1 字节用于 C 字符串结尾 '\0'。
char displayHoldText[5] = {0};
// 最近一次刷新时间，用于 refreshDisplayIfDue 非阻塞兜底刷新。
unsigned long lastDisplayRefreshMs = 0;

}  // namespace

void setupClientDisplay() {
    // 亮度 0..7，现场洞洞板供电正常时用最高亮度便于观察。
    display.setBrightness(7);
    display.clear();
    updateDisplay();
}

// 功能：短暂覆盖正常显示内容。
// 参数 text：最多 4 个字符，超过 4 个会截断；传 nullptr 表示取消当前覆盖。
// 参数 durationMs：覆盖持续时间，单位毫秒；到期后 updateDisplay 会自动回到普通状态。
void holdDisplay(const char* text, unsigned long durationMs) {
    if (text == nullptr) {
        // 允许调用方主动取消覆盖显示，下一次 updateDisplay 会回到普通状态。
        displayHoldUntilMs = 0;
        displayHoldText[0] = '\0';
        return;
    }

    uint8_t i = 0;
    for (; i < 4 && text[i] != '\0'; i++) {
        // TM1637 只有四位，超过四位的提示文本必须截断。
        displayHoldText[i] = text[i];
    }
    for (; i < 4; i++) {
        // 不足四位用空格补齐，避免残留上一条较长提示的字符。
        displayHoldText[i] = ' ';
    }
    displayHoldText[4] = '\0';
    displayHoldUntilMs = millis() + durationMs;
}

// 功能：根据当前客户端状态刷新 TM1637。
// 优先级：
// 1. holdDisplay 覆盖内容，例如 " Err"、"J1  "。
// 2. 未绑定：显示 deviceId 后 4 位，便于服务端控制页识别。
// 3. 正在提交：显示 "SEND"。
// 4. 已锁定：显示 "----"。
// 5. 正常编辑：显示本地红蓝分数，例如 05.07。
void updateDisplay() {
    // 无论显示内容是否变化，都刷新 lastDisplayRefreshMs，防止兜底刷新过密。
    lastDisplayRefreshMs = millis();

    if (displayHoldUntilMs != 0 && static_cast<long>(millis() - displayHoldUntilMs) < 0) {
        // 临时提示优先级最高，例如绑定成功 J1、发送失败 Err。
        display.showText(displayHoldText);
        return;
    }
    if (displayHoldUntilMs != 0) {
        // hold 已过期，清掉标记并继续向下显示普通状态。
        displayHoldUntilMs = 0;
    }

    if (currentClientId == CLIENT_ID_UNASSIGNED) {
        // 未绑定时显示 deviceId 后四位，服务端控制页也显示完整 deviceId，方便对照绑定。
        const String tail = deviceId.length() >= 4 ? deviceId.substring(deviceId.length() - 4) : deviceId;
        display.showText(tail.c_str());
        return;
    }

    if (pendingMsgId != 0) {
        // 等待 ACK/重传期间显示 SEND，提示用户不要继续改分或断电。
        display.showText("SEND");
        return;
    }

    if (lockedForCurrentRound) {
        // 已被服务端确认后显示 ----，表示本轮不能再修改。
        display.showText("----");
        return;
    }

    // 普通可编辑状态显示红蓝分数，格式由 TM1637Display::showScore 处理。
    display.showScore(localRed, localBlue);
}

void refreshDisplayIfDue() {
    if (millis() - lastDisplayRefreshMs >= DISPLAY_REFRESH_INTERVAL_MS) {
        // 主要用于 holdDisplay 到期后自动恢复，不负责高速刷新动画。
        updateDisplay();
    }
}

void printDisplayState() {
    // 串口 show 命令调用，打印的是“逻辑显示状态”，不直接读取 TM1637 硬件。
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
        Serial.print(localRed);
        Serial.print(".");
        Serial.println(localBlue);
    }
}
