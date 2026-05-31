// 裁判端本地业务动作接口。
// 封装按钮和串口命令都会复用的录分、提交、锁定判断等操作。
#pragma once

#include <Arduino.h>

// 尝试把一次本地分数提交放入 pending 队列。
// red/blue：要提交的红蓝分数，调用方应保证范围为 0..99。
// 返回：true=提交已排队等待发送/ACK；false=未绑定、未同步轮号、已有 pending、已锁定等前置条件不满足。
bool tryQueueSubmit(int red, int blue);

// 驱动 pending SUBMIT 的首发和重传状态机。
// 非阻塞；根据 pendingNextSendMs 判断是否该发，重试耗尽后自动清 pending 并显示 Err。
void drivePendingSubmit();

// 当 STATUS 更新 currentServerRoundId 后调用。
// 如果服务端轮号已经不同于 lockedRoundId，说明进入新一轮，解除锁定并清本地分数。
void maybeUnlockOnRoundChange();

// 清空本地正在编辑的红蓝分数并刷新数码管。
void resetLocalScore();

// 调整红方本地分数。delta 可为正或负，结果自动钳到 0..99。
void adjustRed(int delta);

// 调整蓝方本地分数。delta 可为正或负，结果自动钳到 0..99。
void adjustBlue(int delta);

// 判断当前是否允许按键操作。
// 未绑定、正在发送、已锁定时返回 false，按键扫描仍运行但事件会被丢弃。
bool buttonsActive();
