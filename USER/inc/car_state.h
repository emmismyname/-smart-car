#ifndef _CAR_STATE_H_
#define _CAR_STATE_H_
#include "motor.h"
#include "eeprom.h"

#define switch_1 P14
#define switch_2 P47
#define switch_3 P40
#define switch_4 P17



//extern car jiangxing_No_1;//���巢��״̬

typedef struct // ����pid�����������������
{
	int start_flag;//������־λ
	int stop_flag;//ͣ����־λ
	int start_mode;//ͣ����־λ
	float car_speed;
	float left_target_speed;
	float right_target_speed;
	int8 uart_mode;
	uint8 trace_flag;
	uint8 motor_flag;
	uint8 running_mode;
} car_states;



extern car_states jiangxing_No_1;//���巢��״̬
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
void get_switch();//��ȡ����״̬
void switch_init();//������ʼ��



#endif