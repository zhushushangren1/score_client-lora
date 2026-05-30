#pragma once

#include <Arduino.h>

// 初始化 TM1637 数码管，设置亮度并按当前状态刷新一次。
void setupClientDisplay();

// 短暂覆盖数码管显示。
// text：最多 4 字符，nullptr 表示取消覆盖。
// durationMs：覆盖持续时间，单位毫秒。
void holdDisplay(const char* text, unsigned long durationMs);

// 根据当前绑定、pending、锁定、本地分数状态刷新数码管。
void updateDisplay();

// 到 DISPLAY_REFRESH_INTERVAL_MS 时兜底刷新显示，主要处理 hold 到期。
void refreshDisplayIfDue();

// 把当前数码管逻辑状态打印到串口，供 show 命令调试。
void printDisplayState();
