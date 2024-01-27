#ifndef UART_SEND_H
#define UART_SEND_H

#include "common.h"
#include "adc_aquire.h"
#include "motor.h"
#include "imu.h"
#include "SEEKFREE_WIRELESS.h"
#include "zf_delay.h"
#include "element.h"

extern int8 Element_flag;
extern int16 RoundAbout_small_left;
extern int16 RoundAbout_small_right;
extern int16 RoundAbout_big_left;
extern int16 RoundAbout_big_right;
extern int16 UpSlope;
extern int uart_mode;
extern uint32 isr_count_flag;
extern int basespeed;
void floatToByte(float fnum, unsigned char byte[]); // ������ת��������
void wireless_uart_send(float num[], uint8 cnt);    // ���ڷ���
void wireless_send(int mode, int delayTime);        // ���ߴ��ڷ��ͺ���

#endif
