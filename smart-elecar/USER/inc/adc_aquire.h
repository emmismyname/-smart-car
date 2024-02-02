#ifndef _ADC_AQUIRE_H_
#define _ADC_AQUIRE_H_

#include "common.h"
#include "headfile.h"
#include "uart_send.h"
#include "motor.h"
#include "car_state.h"
struct ADC
{
	int16 data_0;	// 电感数值
	int16 data_max; // 电感最大值
	int16 data_min; // 电感最小值
	float data_result;
};
extern struct ADC adc[11];
extern float adc_L1, adc_L2, adc_M, adc_R2, adc_R1;
extern float adc_2_L1, adc_2_L2, adc_2_M, adc_2_R1, adc_2_R2; // 电感数值 : 左外 左竖 左内 中间 右内 右竖 右外
extern float Err_Hori, Err_Vert, Err_2_Hori, Err_2_Vert, Err_1, Err_2,Err_Inclined_left,Err_Inclined_right,compensation_left,compensation_right;
extern float err;	   // 寻迹误差
extern float err_last; // 上一次的寻迹误查
extern uint8 resetelec_flag;
extern int resetelec_count;
extern int myabs(int number);	   // 整数取绝对值
extern float myfabs(float number); // 浮点数取绝对值
extern void ADC_channel_quire();   // 获取电感的数值
extern float Diff_Ratio_Sum_Diff(float adc1, float adc2, float adc3, float adc4, float adc5, float adc6, float a, float b, float c, int Limit);
extern float Diff_Vert_Sum_Diff(float adc1, float adc2, float a, int Limit);
extern float Diff_Hori_Sum_Diff(float adc1, float adc2, float a, int Limit);
extern void resetelec_Init(); // 重新学习赋值电感的最大最小值
extern void ERR_process();	  // 计算差比和差
extern int8 Swerve_flag;

#endif
