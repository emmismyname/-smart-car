#include "assistant.h"



void motor_init()
{
  gpio_mode(P3_6, GPO_PP);									// P64��������Ϊ�������
	gpio_mode(P5_1, GPO_PP);									// P60��������Ϊ�������
	pwm_init(PWM_1, 17000, 1000);								// ��ʼ��PWM1  ʹ��P60����  ��ʼ��Ƶ��Ϊ17Khz
	pwm_init(PWM_2, 17000, 1000);								// ��ʼ��PWM2  ʹ��P62����  ��ʼ��Ƶ��Ϊ17Khz 
}
