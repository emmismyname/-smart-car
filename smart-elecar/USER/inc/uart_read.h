#ifndef UART_READ_H
#define UART_READ_H

#include "common.h"
#include "uart_send.h"
#include "SEEKFREE_WIRELESS.h"
#include "motor.h"
#include "car_state.h"
extern float base_speed;//�����ٶ�����
double myatof(const char *str); // �ַ���ת����������
void wireless_read(uint8 flag); // ����ת���ڶ�ȡ���κ���

#endif