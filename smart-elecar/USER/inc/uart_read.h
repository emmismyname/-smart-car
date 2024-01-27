#ifndef UART_READ_H
#define UART_READ_H

#include "common.h"
#include "uart_send.h"
#include "SEEKFREE_WIRELESS.h"
#include "motor.h"

extern float base_speed;
double myatof(const char *str); // 字符串转浮点数函数
void wireless_read(uint8 flag); // 无线转串口读取调参函数

#endif