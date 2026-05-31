// 裁判端实体按键模块接口。
// 主程序只需要初始化按键并在 loop() 中周期性扫描。
#pragma once

// 初始化 5 个录分按键 GPIO，均为 INPUT_PULLUP，按下为 LOW。
void setupClientButtons();

// 扫描按键并做去抖、短按、长按事件分发。
// 非阻塞；loop 中高频调用。按键是否有效由 buttonsActive() 判断。
void pollButtons();
