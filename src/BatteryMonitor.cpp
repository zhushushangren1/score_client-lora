#include "BatteryMonitor.h"

#include <Arduino.h>

#include "ClientState.h"

namespace {

// 电池采样脚：电池正极经 100K + 100K 分压后接 GPIO15，因此 ADC 读到的是电池电压的一半。
constexpr int BATTERY_ADC_PIN = 15;

// 分压比：节点电压 * 2 = 电池电压。若后续改分压电阻比例，只需要改这里。
constexpr float BATTERY_DIVIDER_RATIO = 2.0f;

// ADC 校准系数：用万用表量真实电池电压，再除以程序读数，得到这个修正倍率。
constexpr float BATTERY_CALIBRATION = 1.0f;

// 电池采样周期和单次平均样本数。5 秒足够用于上报和低电量提示，8 次平均降低 ADC 抖动。
constexpr unsigned long BATTERY_SAMPLE_INTERVAL_MS = 5000;
constexpr int BATTERY_SAMPLE_COUNT = 8;

// 低电量阈值：低于 3.7V 闪烁 LED 提示，但不拦截计分和提交。
constexpr int BATTERY_LOW_MV = 3700;

// 低电量 LED：GPIO2 -> 1K 限流电阻 -> LED 正极，LED 负极 -> GND，高电平点亮。
constexpr int BATTERY_LOW_LED_PIN = 2;
constexpr unsigned long BATTERY_LOW_BLINK_INTERVAL_MS = 400;

unsigned long lastBatterySampleMs = 0;
unsigned long lastBatteryBlinkMs = 0;
bool batteryLowLedOn = false;

// 功能：读取 GPIO15 当前 ADC 电压并换算成电池电压。
// 返回：电池电压，单位毫伏。
int readBatteryMv() {
    uint32_t sum = 0;
    for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
        // analogReadMilliVolts 返回的是分压节点电压，不是电池原始电压。
        sum += analogReadMilliVolts(BATTERY_ADC_PIN);
    }
    // 多次采样求平均，降低 ESP32 ADC 单次读数抖动对电量上报的影响。
    const float nodeMv = static_cast<float>(sum) / BATTERY_SAMPLE_COUNT;
    // +0.5f 是四舍五入到整数毫伏，避免直接截断带来系统性偏低。
    return static_cast<int>(nodeMv * BATTERY_DIVIDER_RATIO * BATTERY_CALIBRATION + 0.5f);
}

}  // namespace

void setupBatteryMonitor() {
    // 低电量 LED 默认熄灭，只有 updateBatteryLowLed 判断低电量后才闪烁。
    pinMode(BATTERY_LOW_LED_PIN, OUTPUT);
    digitalWrite(BATTERY_LOW_LED_PIN, LOW);

    // 12 位分辨率 + 11dB 衰减适合读取约 0..3.3V 的分压节点。
    analogReadResolution(12);
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    // 启动时立即采样一次，保证首个 HELLO 不会带 batteryMv=0。
    sampleBatteryNow();
}

int sampleBatteryNow() {
    // batteryMv 是全局状态，LoRa HELLO/HEARTBEAT/SUBMIT 都直接读取它。
    batteryMv = readBatteryMv();
    lastBatterySampleMs = millis();
    return batteryMv;
}

void sampleBatteryIfDue() {
    if (millis() - lastBatterySampleMs < BATTERY_SAMPLE_INTERVAL_MS) {
        // 未到周期时不读 ADC，减少 loop 中不必要的耗时。
        return;
    }
    sampleBatteryNow();
}

void updateBatteryLowLed() {
    const bool low = (batteryMv != 0 && batteryMv < BATTERY_LOW_MV);
    if (!low) {
        // 电量正常或尚未成功采样时，确保低电量灯保持熄灭。
        if (batteryLowLedOn) {
            batteryLowLedOn = false;
            digitalWrite(BATTERY_LOW_LED_PIN, LOW);
        }
        return;
    }

    if (millis() - lastBatteryBlinkMs >= BATTERY_LOW_BLINK_INTERVAL_MS) {
        // 低电量闪烁使用 millis，不使用 delay，避免影响 LoRa 收发和按键扫描。
        lastBatteryBlinkMs = millis();
        batteryLowLedOn = !batteryLowLedOn;
        digitalWrite(BATTERY_LOW_LED_PIN, batteryLowLedOn ? HIGH : LOW);
    }
}
