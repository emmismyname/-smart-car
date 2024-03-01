#include "adc_aquire.h"

struct ADC adc[11];
int32 sum[11];
float adc_L1, adc_L2, adc_M, adc_R2, adc_R1;// 电感数值 : 左外 左竖 左内 中间 右内 右竖 右外
float adc_2_L1, adc_2_L2, adc_2_M, adc_2_R1, adc_2_R2; // 电感数值 : 左外 左竖 左内 中间 右内 右竖 右外
float forsight_Left=0,forsight_Right=0;
float turn_Left=0,turn_Right=0;//转弯变量
float Err_Hori, Err_Vert, Err_2_Hori, Err_2_Vert, Err_1, Err_2,forward_err=0,turn_err=0,
compensation_left=40,compensation_right=40;
int resetelec_count = 0;
uint8 resetelec_flag = 0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      绝对值函数
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
int myabs(int number) // 整数取绝对值
{
	if (number < 0)
		return -number;
	else
		return number;
}
float myfabs(float number) // 浮点数取绝对值
{
	if (number < 0)
		return -1.0 * number;
	else
		return number;
}
float squre_sum(float number1,float number2) // 浮点数取绝对值
{
	float result;
  result=number1*number1+number2*number2;
  result=sqrt(result);
  return result;

}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ADC均值滤波
//
//-------------------------------------------------------------------------------------------------------------------
// uint16 adc_mean_filter(ADCN_enum adcn, uint8 count)
// {
// 	uint8 i;
// 	uint32 sum;
// 	sum = 0;
// 	for (i = 0; i < count; i++)
// 	{
// 		sum += adc_once(adcn, ADC_12BIT);
// 	}
// 	return sum / count;
// }

//-------------------------------------------------------------------------------------------------------------------
// @brief		ADC均值滤波
//
//-------------------------------------------------------------------------------------------------------------------
void adc_aquire(int count)
{
	static int i = 0;

	for (i = 0; i < count; i++)
	{
		adc[1].data_0 = adc_once(ADC_P01, ADC_12BIT);
		adc[2].data_0 = adc_once(ADC_P14, ADC_12BIT);
		adc[3].data_0 = adc_once(ADC_P06, ADC_12BIT);
		adc[4].data_0 = adc_once(ADC_P13, ADC_12BIT);
		adc[5].data_0 = adc_once(ADC_P16, ADC_12BIT);

		adc[6].data_0 = adc_once(ADC_P10, ADC_12BIT);
		adc[7].data_0 = adc_once(ADC_P11, ADC_12BIT);
		adc[8].data_0 = adc_once(ADC_P00, ADC_12BIT);
		adc[9].data_0 = adc_once(ADC_P05, ADC_12BIT);
		adc[10].data_0 = adc_once(ADC_P17, ADC_12BIT);

		// adc[1].data_0 = adc_once(ADC_P01, ADC_12BIT);
		// adc[2].data_0 = adc_once(ADC_P14, ADC_12BIT);
		// adc[3].data_0 = adc_once(ADC_P06, ADC_12BIT);
		// adc[4].data_0 = adc_once(ADC_P13, ADC_12BIT);
		// adc[5].data_0 = adc_once(ADC_P16, ADC_12BIT);

		// adc[6].data_0 = adc_once(ADC_P10, ADC_12BIT);
		// adc[7].data_0 = adc_once(ADC_P11, ADC_12BIT);
		// adc[8].data_0 = adc_once(ADC_P00, ADC_12BIT);
		// adc[9].data_0 = adc_once(ADC_P05, ADC_12BIT);
		// adc[10].data_0 = adc_once(ADC_P17, ADC_12BIT);
		sum[1] += adc[1].data_0;
		sum[2] += adc[2].data_0;
		sum[3] += adc[3].data_0;
		sum[4] += adc[4].data_0;
		sum[5] += adc[5].data_0;

		sum[6] += adc[6].data_0;
		sum[7] += adc[7].data_0;
		sum[8] += adc[8].data_0;
		sum[9] += adc[9].data_0;
		sum[10] += adc[10].data_0;
	}

	adc[1].data_result = sum[1] / count;
	adc[2].data_result = sum[2] / count;
	adc[3].data_result = sum[3] / count;
	adc[4].data_result = sum[4] / count;
	adc[5].data_result = sum[5] / count;

	adc[6].data_result = sum[6] / count;
	adc[7].data_result = sum[7] / count;
	adc[8].data_result = sum[8] / count;
	adc[9].data_result = sum[9] / count;
	adc[10].data_result = sum[10] / count;

	sum[1] = 0;
	sum[2] = 0;
	sum[3] = 0;
	sum[4] = 0;
	sum[5] = 0;

	sum[6] = 0;
	sum[7] = 0;
	sum[8] = 0;
	sum[9] = 0;
	sum[10] = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获得处理后电感值归一化处理和 获得差比和
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void ADC_channel_quire() // 获得处理后电感值归一化处理和 获得差比和
{
	adc_aquire(10);

	// adc[1].data_0 = adc_mean_filter(ADC_P10,8);		//采集并处理ADC_P17电压
	// adc[2].data_0 = adc_mean_filter(ADC_P11,8);		//采集并处理ADC_P13电压
	// adc[3].data_0 = adc_mean_filter(ADC_P14,8);		//采集并处理ADC_P01电压
	// adc[4].data_0 = adc_mean_filter(ADC_P17,8);		//采集并处理ADC_P05电压
	// adc[5].data_0 = adc_mean_filter(ADC_P16,8);		//采集并处理ADC_P06电压
	// adc[6].data_0 = adc_mean_filter(ADC_P06,8);		//采集并处理ADC_P14电压
	// adc[7].data_0 = adc_mean_filter(ADC_P05,8);		//采集并处理ADC_P16电压
	// adc[8].data_0 = adc_mean_filter(ADC_P01,8);		//采集并处理ADC_P01电压
	// adc[9].data_0 = adc_mean_filter(ADC_P00,8);		//采集并处理ADC_P01电压
	// adc[10].data_0 = adc_mean_filter(ADC_P13,8);		//采集并处理ADC_P01电压
	// 归一化处理电感值，各偏移量归一化到0--100以内
	//  adc_L1 = adc[1].data_0;
	//  adc_L2 = adc[2].data_0;
	//  adc_M  = adc[3].data_0;
	//  adc_R2 = adc[4].data_0;
	//  adc_R1 = adc[5].data_0;
	//  adc_2_L1 = 	adc[6].data_0;
	//  adc_2_L2 = 	adc[7].data_0;
	//  adc_2_M  = 	adc[8].data_0;
	//  adc_2_R2 = 	adc[9].data_0;
	//  adc_2_R1 =	adc[10].data_0;

	adc_R1 = (adc[1].data_result - adc[1].data_min) * 100.0 / (adc[1].data_max - adc[1].data_min);
	adc_R2 = (adc[2].data_result - adc[2].data_min) * 100.0 / (adc[2].data_max - adc[2].data_min);
	adc_M = (adc[3].data_result - adc[3].data_min) * 100.0 / (adc[3].data_max - adc[3].data_min);
	adc_L2 = (adc[4].data_result - adc[4].data_min) * 100.0 / (adc[4].data_max - adc[4].data_min);
	adc_L1 = (adc[5].data_result - adc[5].data_min) * 100.0 / (adc[5].data_max - adc[5].data_min);

	adc_2_R1 = (adc[6].data_result - adc[6].data_min) * 100.0 / (adc[6].data_max - adc[6].data_min);
	adc_2_R2 = (adc[7].data_result - adc[7].data_min) * 100.0 / (adc[7].data_max - adc[7].data_min);
	adc_2_M = (adc[8].data_result - adc[8].data_min) * 100.0 / (adc[8].data_max - adc[8].data_min);
	adc_2_L2 = (adc[9].data_result - adc[9].data_min) * 100.0 / (adc[9].data_max - adc[9].data_min);
	adc_2_L1 = (adc[10].data_result - adc[10].data_min) * 100.0 / (adc[10].data_max - adc[10].data_min);

	if (adc_L1 < 0)
		adc_L1 = 0;
	if (adc_L2 < 0)
		adc_L2 = 0;
	if (adc_M < 0)
		adc_M = 0;
	if (adc_R1 < 0)
		adc_R1 = 0;
	if (adc_R2 < 0)
		adc_R2 = 0;
	if (adc_2_L1 < 0)
		adc_2_L1 = 0;
	if (adc_2_L2 < 0)
		adc_2_L2 = 0;
	if (adc_2_M < 0)
		adc_2_M = 0;
	if (adc_2_R1 < 0)
		adc_2_R1 = 0;
	if (adc_2_R2 < 0)
		adc_2_R2 = 0;
	// 得到电感的差比和差
	//  Err_1=Diff_Ratio_Sum_Diff(adc_L1,adc_L2,0,0,adc_R2,adc_R1,0,0.2,0.6,20);
	//  Err_2=Diff_Ratio_Sum_Diff(adc_2_L1,adc_2_L2,0,0,adc_2_R2,adc_2_R1,0,0.4,0.6,20);
	ERR_process(); // 误查处理赋值
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      差比和差公式
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
float Diff_Ratio_Sum_Diff(float adc1, float adc2, float adc3, float adc4, float adc5, float adc6, float a, float b, float c, int Limit)
{

	float ans, fz, fm;
	int16 Diff_Inner, Diff_Mid, Diff_Outer;
	int16 Sum_Inner, Sum_Mid, Sum_Outer;

	Diff_Inner = adc3 - adc4;
	Diff_Mid = adc2 - adc5;
	Diff_Outer = adc1 - adc6;

	Sum_Inner = adc3 + adc4;
	Sum_Mid = adc2 + adc5;
	Sum_Outer = adc1 + adc6;

	fz = a * Diff_Inner + b * Diff_Mid + c * Diff_Outer;
	fm = a * Sum_Inner + b * Sum_Mid + c * Sum_Outer;

	ans = (float)(Limit * (fz * 1.0 / fm)); // 差比和差
	return ans;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      差比和差公式
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
float Diff_Hori_Sum_Diff(float adc1, float adc2, float a, int Limit)
{

	float ans, fz, fm;
	int16 Diff_Mid;
	int16 Sum_Mid;
	if (adc1 > 5 || adc2 > 5)
	{
		Diff_Mid = adc1 - adc2;
		Sum_Mid = adc1 + adc2;
		fz = a * Diff_Mid;
		fm = a * Sum_Mid;
		ans = (float)(Limit * (fz * 1.0 / fm)); // 差比和差
	}
	else
	{
		ans = 0;
	}

	return ans;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      差比和差公式
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
float Diff_turn_Sum_Diff(float adc1, float adc2, float a, int Limit)
{

	float ans, fz, fm;
	int16 Diff_Mid;
	int16 Sum_Mid;
	if (adc1 > 1 || adc2 > 1)
	{
		Diff_Mid = adc1 - adc2;
		Sum_Mid = adc1 + adc2+0.001;
		fz = a * Diff_Mid;
		fm = a * Sum_Mid;
		ans = (float)(Limit * (fz * 1.0 / fm)); // 差比和差
	}
	else
	{
		ans = 0;
	}

	return ans;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电感最大最小值获取
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void resetelec_Init() // 重新学习赋值电感的最大最小值
{
	if (resetelec_flag == 1)
	{
		uint16 i;
		resetelec_count = 0;
		// trace_flag = 1;
		// brusheless_flag = 1;
		// brusheless_duty = 800;
		// delay_ms(1200);
		if (resetelec_count == 0)
			ips114_clear(WHITE);
		{
			adc[1].data_max = 0;
			adc[1].data_min = 5000;
			adc[2].data_max = 0;
			adc[2].data_min = 5000;
			adc[3].data_max = 0;
			adc[3].data_min = 5000;
			adc[4].data_max = 0;
			adc[4].data_min = 5000;
			adc[5].data_max = 0;
			adc[5].data_min = 5000;
			adc[6].data_max = 0;
			adc[6].data_min = 5000;
			adc[7].data_max = 0;
			adc[7].data_min = 5000;
			adc[8].data_max = 0;
			adc[8].data_min = 5000;
			adc[9].data_max = 0;
			adc[9].data_min = 5000;
			adc[10].data_max = 0;
			adc[10].data_min = 5000;
		}

		while (resetelec_count < 300)
		{
			for (i = 1; i <= 10; i++)
			{
				if (adc[i].data_result > adc[i].data_max)
					adc[i].data_max = adc[i].data_result;
				if (adc[i].data_result < adc[i].data_min)
					adc[i].data_min = adc[i].data_result;
			}
			wireless_send(2, 25);
			resetelec_count++;
			ips114_showuint16(0, 0, resetelec_count);
		}
		resetelec_count = 0;
		resetelec_flag = 0;
		ips114_clear(WHITE);
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      误差处理
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void ERR_process() // 误查处理
{
	turn_Left=squre_sum(adc_2_L1,adc_2_L2);
	turn_Right=squre_sum(adc_2_R1,adc_2_R2);
	forsight_Left=adc_L1+adc_L2;
	forsight_Right=adc_R1+adc_R2;
  
	forward_err=Diff_turn_Sum_Diff(forsight_Left-forsight_Right,1,30);
	turn_err=Diff_turn_Sum_Diff(turn_Left,turn_Right,1,30);
  Forward_PID.err=forward_err;
	Swerve_PID.err=turn_err;

	// if (Swerve_flag == 0 && ((adc_L2 > 80 || adc_R2 > 80) && adc_M > 60&&adc_2_M<120)) // 当数值电感到达一定数值的时候直道转换的时候
	// {
	// 	Swerve_flag = 1;
	// 	err = 1.1 * Err_Hori + 0.5 * Err_Vert + 0.8 * Err_2_Hori + 0.5 * Err_2_Vert;
	// }
	// else if (Swerve_flag == 1)
	// {
	// 	err = 1.1 * Err_Hori + 0.5 * Err_Vert + 0.8 * Err_2_Hori + 0.5 * Err_2_Vert;
	// }
	// if (Swerve_flag == 1 && (adc_2_R2 < 15 && adc_2_L2 < 15 && adc_2_M > 30 && adc_M > 30)) // 直道的误差数值
	// {
	// 	Swerve_flag = 0;
	// 	err = 0.6 * Err_Hori + 0.6 * Err_Vert + 0.8 * Err_2_Hori;
	// }
	// if (Swerve_flag == 0)
	// {
	// 	err = 0.6 * Err_Hori + 0.6 * Err_Vert + 0.8 * Err_2_Hori;
	// }

}
