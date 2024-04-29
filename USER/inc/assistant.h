#ifndef _ASSISTANT_H_
#define _ASSISTANT_H_

#include "headfile.h"
#include "mycode.h"

#define WORK_LED P01


void motor_init();//电机初始化
void LED_init();  //led初始化
void WORK_LED_on();// 工作LED亮
void WORK_LED_off();      // 工作LED灭


#endif