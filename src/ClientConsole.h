#pragma once

// 处理裁判端 USB 串口命令。
// 支持：submit、submit <red> <blue>、show。
// 内部按字节维护行缓冲，接受 CR/LF/CRLF。
void handleSerialCommand();
