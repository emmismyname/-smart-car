#include "assistant.h"



void motor_init()
{
  gpio_mode(P3_6, GPO_PP);									// P64引脚设置为推挽输出
	gpio_mode(P5_1, GPO_PP);									// P60引脚设置为推挽输出
	pwm_init(PWM_1, 17000, 1000);								// 初始化PWM1  使用P60引脚  初始化频率为17Khz
	pwm_init(PWM_2, 17000, 1000);								// 初始化PWM2  使用P62引脚  初始化频率为17Khz 
}
