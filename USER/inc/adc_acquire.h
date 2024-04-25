
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
	int16 data_0;	// 电感数值
	int16 data_max; // 电感最大值
	int16 data_min; // 电感最小值
	float data_result;
};

extern float err;	   // 寻迹误差
extern float err_last; // 上一次的寻迹误查
extern struct ADC adc[11];
extern float adc1_L1_V, adc1_L2_H, adc1_M_H, adc1_R2_H, adc1_R1_V;
extern float adc2_L1_V, adc2_L2_H, adc2_M_H, adc2_R1_V, adc2_R2_H; // 电感数值 : 左外 左竖 左内 中间 右内 右竖 右外
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

extern int myabs(int number);	   // 整数取绝对值
void quickSort(int array[], int low, int high) reentrant;   // 快速排序
int partition(int array[], int low, int high);    // 快排分区                   // 交换函数
extern float myfabs(float number); // 浮点数取绝对值
extern void ADC_channel_quire();   // 获取电感的数值
extern float Diff_Ratio_Sum_Diff(float adc1, float adc2, float adc3, float adc4, float adc5, float adc6, float a, float b, float c, int Limit);
extern float Diff_Vert_Sum_Diff(float adc1, float adc2, float a, int Limit);
extern float Diff_Hori_Sum_Diff(float adc1, float adc2, float a, int Limit);
extern void resetelec_Init(); // 重新学习赋值电感的最大最小值
extern void ERR_process();	  // 计算差比和差

#endif