#ifndef _CAR_STATE_H_
#define _CAR_STATE_H_
#include "motor.h"
#include "eeprom.h"

#define switch_1 P14
#define switch_2 P47
#define switch_3 P40
#define switch_4 P17



//extern car jiangxing_No_1;//车体发车状态

typedef struct // 定义pid常量参数及输出变量
{
	int start_flag;//发车标志位
	int stop_flag;//停车标志位
	int start_mode;//停车标志位
	float car_speed;
	float left_target_speed;
	float right_target_speed;
	int8 uart_mode;
	uint8 trace_flag;
	uint8 motor_flag;
	uint8 running_mode;
} car_states;



extern car_states jiangxing_No_1;//车体发车状态
extern uint8 SWITCH1_CURRENTSTATE;
extern uint8 SWITCH2_CURRENTSTATE;
extern uint8 SWITCH3_CURRENTSTATE;
extern uint8 SWITCH4_CURRENTSTATE;
extern uint8 SWITCH1_FLAG;
extern uint8 SWITCH2_FLAG;
extern uint8 SWITCH3_FLAG;
extern uint8 SWITCH4_FLAG;

int car_init(car_states *name);
void parameter_init();
void get_switch();//获取按键状态
void switch_init();//按键初始化



#endif