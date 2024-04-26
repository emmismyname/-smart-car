#ifndef _CARCONTROL_H_
#define _CARCONTROL_H_

#include "headfile.h"
#include "motor.h"
#include "adc_aquire.h"
#include "imu.h"
#include "car_state.h"
// #include "eeprom.h"
// #include "element.h"


typedef struct // ����pid�����������������
{
	 uint32 start_time; //����ʱ���־λ
	 uint32 end_time;   //ͣ��ʱ���־λ
   uint32 running_time;   //ͣ��ʱ���־λ
   uint8 work_flag;//������־λ 0��ֹͣ����1��ʼ����
} time_run;



// /****************************�˶�״̬���********************************/




void Element_Idef(); // Ԫ��ʶ��
void mode_choose(int move_mode); // �˶�ģʽ�л�
void car_start();//����
void car_strategy(uint8 s_mode);//����ģʽ


#endif