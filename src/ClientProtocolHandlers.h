// 裁判端 LoRa 入站协议处理接口。
// 主循环调用该入口即可持续消化 E22 收到的完整协议帧。
#pragma once

// 处理 LoRa 入站协议帧。
// 从 ClientLoraLink 读取完整帧，解析 CRC/字段，并分发 STATUS/ACK/ASSIGN/UNBIND。
void handleLoraInput();
