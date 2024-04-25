
/*********************************************************************************************************************
 * @file       		adc_acquire
 * @date       		2024-03-06
 * @note   
 ********************************************************************************************************************/


#ifndef _ADC_ACQUIRE_H_
#define _ADC_ACQUIRE_H_

#include "headfile.h"

struct ADC
{
	int16 data_0;	// �����ֵ
	int16 data_max; // ������ֵ
	int16 data_min; // �����Сֵ
	float data_result;
};

extern float err;	   // Ѱ�����
extern float err_last; // ��һ�ε�Ѱ�����
extern struct ADC adc[11];
extern float adc1_L1_V, adc1_L2_H, adc1_M_H, adc1_R2_H, adc1_R1_V;
extern float adc2_L1_V, adc2_L2_H, adc2_M_H, adc2_R1_V, adc2_R2_H; // �����ֵ : ���� ���� ���� �м� ���� ���� ����
extern float Err_Hori, Err_Vert, Err_2_Hori, Err_2_Vert, Err_1, Err_2,Err_Inclined_left,Err_Inclined_right,compensation_left,compensation_right;
extern uint8 resetelec_flag;
extern int resetelec_count;
extern int8 Swerve_flag;
extern uint8 ADC_OUTOFRANGE_LIMIT; 
extern float adc1_L2H_withL1V ,adc1_MH_withL1V  ,adc1_R2H_withR1V ,adc1_MH_withR1V  ,
						 adc2_L2H_withL1V ,adc2_MH_withL1V  ,adc2_R2H_withR1V ,adc2_MH_withR1V	;
extern float vector_ratio[2][2] ;
extern float straight_convert_ratio[2][3];
extern float curve_convert_ratio[2][4];

extern int myabs(int number);	   // ����ȡ����ֵ
void quickSort(int array[], int low, int high) reentrant;   // ��������
int partition(int array[], int low, int high);    // ���ŷ���                   // ��������
extern float myfabs(float number); // ������ȡ����ֵ
extern void ADC_channel_quire();   // ��ȡ��е���ֵ
extern float Diff_Ratio_Sum_Diff(float adc1, float adc2, float adc3, float adc4, float adc5, float adc6, float a, float b, float c, int Limit);
extern float Diff_Vert_Sum_Diff(float adc1, float adc2, float a, int Limit);
extern float Diff_Hori_Sum_Diff(float adc1, float adc2, float a, int Limit);
extern void resetelec_Init(); // ����ѧϰ��ֵ��е������Сֵ
extern void ERR_process();	  // �����ȺͲ�

#endif