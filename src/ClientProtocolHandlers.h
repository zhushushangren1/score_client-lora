#pragma once

// 处理 LoRa 入站协议帧。
// 从 ClientLoraLink 读取完整帧，解析 CRC/字段，并分发 STATUS/ACK/ASSIGN/UNBIND。
void handleLoraInput();
