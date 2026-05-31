// 裁判端程序入口。
// 负责按顺序初始化串口、设备身份、LoRa、按键、数码管、电池检测，
// 并在 loop() 中调度各个非阻塞模块。
#include <Arduino.h>

#include "BatteryMonitor.h"
#include "ClientActions.h"
#include "ClientButtons.h"
#include "ClientConsole.h"
#include "ClientDisplay.h"
#include "ClientLoraLink.h"
#include "ClientProtocolHandlers.h"
#include "ClientState.h"

void setup() {
    // 打开 USB 调试串口。所有启动状态、LoRa 收发日志、串口命令回显都走这里输出。
    Serial.begin(115200);

    // 某些 ESP32-S3 板子的 USB CDC 需要一点枚举时间。
    // 最多等 5 秒，避免脱机供电时永远卡在 while (!Serial)。
    const unsigned long serialWaitStart = millis();
    while (!Serial && millis() - serialWaitStart < 5000) {
        delay(10);
    }
    delay(500);

    // deviceId 是本机永久身份，不写 NVS，而是每次从芯片 MAC 生成。
    // 这样即使擦除 flash，服务端仍能认出同一块板。
    deviceId = makeDeviceIdFromMac();

    // clientId 是服务端分配的裁判槽位，需要持久化。
    // 未绑定时 currentClientId 会被设为 UNASSIGNED。
    loadClientIdFromNvs();

    // 给随机退避播种。
    // 退避用于多台裁判同时提交时错开发包时间，减少 LoRa 同频碰撞。
    randomSeed(static_cast<unsigned long>(ESP.getEfuseMac() & 0xFFFFFFFFULL) ^ millis());

    Serial.println();
    Serial.println("score_client-lora boot");
    Serial.print("Device ID: ");
    Serial.println(deviceId);
    Serial.print("Client ID: ");
    Serial.println(currentClientId);
    Serial.print("Millis: ");
    Serial.println(millis());

    setupClientLoraLink();   // 必须早于 sendHello()，否则 HELLO 无法通过 E22 发出。
    setupClientButtons();    // 只初始化 GPIO，不触发任何业务动作。
    setupClientDisplay();    // 先显示当前绑定/未绑定状态，让上电后立刻有可见反馈。
    setupBatteryMonitor();   // 必须早于 sendHello()，保证 HELLO 携带真实电池电压。

    Serial.print("Battery: ");
    Serial.print(batteryMv);
    Serial.println("mV");

    // 上电后主动自报一次。
    // 服务端收到 HELLO 后会回 STATUS；如果未绑定，控制页会出现这台设备。
    sendHello();

    Serial.println("E22 UART transparent ready");
    Serial.println("Serial commands: submit <red 0-99> <blue 0-99> / show");
}

void loop() {
    // 先处理 LoRa 入站帧，保证 ACK/ASSIGN/STATUS 尽快生效。
    // 这能降低 pending 提交不必要重传，也能让绑定/解绑尽快刷新显示。
    handleLoraInput();

    // USB 串口调试命令优先级仅次于 LoRa，便于现场无按键时手工 submit/show。
    handleSerialCommand();

    // 扫描实体按键。内部做去抖和长按判断，不会 delay 阻塞。
    pollButtons();

    // 如果有 pending SUBMIT，这里按随机退避时间点发首包或重传。
    // 该函数非阻塞；没到发送时间会立即返回。
    drivePendingSubmit();

    // 电池采样周期较慢，sampleBatteryIfDue 只有到 5s 周期才真正 analogRead。
    sampleBatteryIfDue();

    // 低电量 LED 闪烁也不能 delay，靠 millis 做非阻塞翻转。
    updateBatteryLowLed();

    // 根据是否锁定选择 10s/15s 心跳周期。心跳用于在线状态、电量和轮次同步。
    sendHeartbeatIfDue();

    // 兜底刷新显示，主要处理 holdDisplay 到期后恢复普通显示。
    refreshDisplayIfDue();
}
