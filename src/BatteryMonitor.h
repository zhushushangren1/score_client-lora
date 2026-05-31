// 裁判端电池监测模块接口。
// 向主循环提供电池采样、低电量 LED 更新和立即采样函数。
#pragma once

// 初始化 GPIO15 ADC 和低电量 LED。
void setupBatteryMonitor();

// 到采样周期时刷新全局 batteryMv，未到周期则立即返回。
void sampleBatteryIfDue();

// 根据 batteryMv 驱动低电量 LED。低电量时闪烁，电量正常时熄灭。
void updateBatteryLowLed();

// 立即采样一次电池电压，返回毫伏值，并同步写入全局 batteryMv。
int sampleBatteryNow();
