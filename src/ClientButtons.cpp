#include "ClientButtons.h"

#include <Arduino.h>

#include "ClientActions.h"
#include "ClientState.h"

namespace {

constexpr int BUTTON_RED_1_PIN = 4;
constexpr int BUTTON_RED_2_PIN = 5;
constexpr int BUTTON_BLUE_1_PIN = 6;
constexpr int BUTTON_BLUE_2_PIN = 7;
constexpr int BUTTON_SUBMIT_PIN = 8;

constexpr uint8_t BUTTON_COUNT = 5;
constexpr int BUTTON_PINS[BUTTON_COUNT] = {
    BUTTON_RED_1_PIN,
    BUTTON_RED_2_PIN,
    BUTTON_BLUE_1_PIN,
    BUTTON_BLUE_2_PIN,
    BUTTON_SUBMIT_PIN
};

constexpr unsigned long BUTTON_DEBOUNCE_MS = 20;
constexpr unsigned long BUTTON_LONG_PRESS_MS = 600;

struct ButtonState {
    // 去抖后的稳定电平。INPUT_PULLUP 下 true=松开/HIGH，false=按下/LOW。
    bool stableLevel = true;
    // 最近一次直接 digitalRead 得到的原始电平，用来检测抖动边沿。
    bool lastRawLevel = true;
    // 原始电平最后一次变化的时间点；超过 BUTTON_DEBOUNCE_MS 才承认为稳定变化。
    unsigned long lastRawChangeMs = 0;
    // 稳定按下的起始时间，用于计算长按。
    unsigned long pressedAtMs = 0;
    // 本次按下期间是否已经触发过长按，防止长按动作在每个 loop 重复触发。
    bool longPressFired = false;
};

ButtonState buttons[BUTTON_COUNT];

void handleButtonShort(uint8_t idx) {
    // 短按：加分或提交。idx 与 BUTTON_PINS 数组顺序一一对应。
    switch (idx) {
        case 0: adjustRed(+1); break;   // 红方 +1。
        case 1: adjustRed(+2); break;   // 红方 +2。
        case 2: adjustBlue(+1); break;  // 蓝方 +1。
        case 3: adjustBlue(+2); break;  // 蓝方 +2。
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

void handleButtonLong(uint8_t idx) {
    // 长按：扣分或清空本地编辑分。扣分同样会被 clampScore 限制到不低于 0。
    switch (idx) {
        case 0: adjustRed(-1); break;   // 红方 -1。
        case 1: adjustRed(-2); break;   // 红方 -2。
        case 2: adjustBlue(-1); break;  // 蓝方 -1。
        case 3: adjustBlue(-2); break;  // 蓝方 -2。
        case 4:
            Serial.println("BUTTON: long-submit -> clear round (red/blue -> 0)");
            resetLocalScore();
            break;
        default: break;
    }
}

}  // namespace

void setupClientButtons() {
    for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
        // 使用内部上拉，按钮另一端接 GND；按下时读到 LOW。
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }
}

void pollButtons() {
    const unsigned long now = millis();
    for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
        ButtonState& button = buttons[i];
        // raw=true 表示松开，raw=false 表示按下；统一成布尔语义方便后面判断。
        const bool raw = digitalRead(BUTTON_PINS[i]) != LOW;

        if (raw != button.lastRawLevel) {
            // 原始电平一变化就重置去抖计时，直到稳定超过 BUTTON_DEBOUNCE_MS。
            button.lastRawLevel = raw;
            button.lastRawChangeMs = now;
        }

        if (now - button.lastRawChangeMs >= BUTTON_DEBOUNCE_MS &&
            raw != button.stableLevel) {
            // 到这里说明新电平已经稳定，正式更新稳定状态。
            button.stableLevel = raw;
            if (!raw) {
                // 稳定进入按下态：记录起点并允许本次按下触发一次长按。
                button.pressedAtMs = now;
                button.longPressFired = false;
            } else if (!button.longPressFired && buttonsActive()) {
                // 松开时若还没触发长按，就视为短按；禁用状态下直接丢弃事件。
                handleButtonShort(i);
            }
        }

        if (!button.stableLevel && !button.longPressFired &&
            now - button.pressedAtMs >= BUTTON_LONG_PRESS_MS) {
            // 按住超过阈值后立即触发长按，不需要等到松手。
            button.longPressFired = true;
            if (buttonsActive()) {
                handleButtonLong(i);
            }
        }
    }
}
