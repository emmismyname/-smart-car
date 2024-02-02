#ifndef _ADC_AQUIRE_H_
#define _ADC_AQUIRE_H_

#include "common.h"
#include "headfile.h"
#include "uart_send.h"
#include "motor.h"
#include "car_state.h"
struct ADC
{
	int16 data_0;	// �����ֵ
	int16 data_max; // ������ֵ
	int16 data_min; // �����Сֵ
	float data_result;
};
extern struct ADC adc[11];
extern float adc_L1, adc_L2, adc_M, adc_R2, adc_R1;
extern float adc_2_L1, adc_2_L2, adc_2_M, adc_2_R1, adc_2_R2; // �����ֵ : ���� ���� ���� �м� ���� ���� ����
extern float Err_Hori, Err_Vert, Err_2_Hori, Err_2_Vert, Err_1, Err_2,Err_Inclined_left,Err_Inclined_right,compensation_left,compensation_right;
extern float err;	   // Ѱ�����
extern float err_last; // ��һ�ε�Ѱ�����
extern uint8 resetelec_flag;
extern int resetelec_count;
extern int myabs(int number);	   // ����ȡ����ֵ
extern float myfabs(float number); // ������ȡ����ֵ
extern void ADC_channel_quire();   // ��ȡ��е���ֵ
extern float Diff_Ratio_Sum_Diff(float adc1, float adc2, float adc3, float adc4, float adc5, float adc6, float a, float b, float c, int Limit);
extern float Diff_Vert_Sum_Diff(float adc1, float adc2, float a, int Limit);
extern float Diff_Hori_Sum_Diff(float adc1, float adc2, float a, int Limit);
extern void resetelec_Init(); // ����ѧϰ��ֵ��е������Сֵ
extern void ERR_process();	  // �����ȺͲ�
extern int8 Swerve_flag;

#endif
