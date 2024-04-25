
/*********************************************************************************************************************
 * @file       		adc_acquire
 * @date       		2024-03-06
 * @note   
 ********************************************************************************************************************/

#include "adc_acquire.h"


#define ADC_CHANNEL 10				// ADCͨ����
#define ADC_CONVERT_TIMES 14	// ADCת���ܴ���
#define ADC_CONVERT_DIV 7			// ADCת���Ӵ���
#define ADC_RESET_TIMES 500 	// �����ֵ��������
#define ADC_ACQUIRE_DELAY 25  // �����ֵ�������
uint8 ADC_OUTOFRANGE_LIMIT = 50; // �����ж�ֵ

struct ADC adc[11];						// ADC�ṹ��

float err = 0;						   	// Ѱ�����
float err_last = 0;						// ���ǰֵ

float adc1_L2_H, adc1_L1_V, adc1_M_H, adc1_R1_V, adc1_R2_H;	// �����ֵ : ���� ���� �м� ���� ����
float adc2_L2_H, adc2_L1_V, adc2_M_H, adc2_R1_V, adc2_R2_H; // �����ֵ : ���� ���� �м� ���� ����

float Err_Hori, Err_Vert, Err_2_Hori, Err_2_Vert,Err_1, Err_2,	// ��Ⱥ����
Err_Inclined_left=0,Err_Inclined_right=0,												// Ԫ���ж����
compensation_left=40,compensation_right=40;											// Ԫ���ж�ϵ��

int resetelec_count = 0;																				// ɨ��м���λ
uint8 resetelec_flag = 0;																				// ɨ��б�־λ

float vector_ratio[2][2] = {{0.60,0.80},												// ��������ں�ϵ����ǰ��
														{0.70,0.90}};												// ��������ں�ϵ������

float straight_convert_ratio[2][3] = {{0.6,0.8,0.6},						// ֱ��������ϵ����1��
														          {0.6,0.8,0.6}};						// ֱ��������ϵ����2��

float curve_convert_ratio[2][4] = {{1.1,0.5,0.8,0.5},						// ֱ��������ϵ����1��
														       {1.1,0.5,0.8,0.5}};					// ֱ��������ϵ����2��

float adc1_L2H_withL1V ,adc1_MH_withL1V  ,adc1_R2H_withR1V ,adc1_MH_withR1V ,		// ����ƫ�����
			adc2_L2H_withL1V ,adc2_MH_withL1V  ,adc2_R2H_withR1V ,adc2_MH_withR1V	;


const ADCN_enum adc_pin[ADC_CHANNEL+1]=													// ��������
{
	ADC_P00,	ADC_P05,	ADC_P03,	ADC_P01,	ADC_P00, ADC_P02,
						ADC_P15,	ADC_P14,	ADC_P17,	ADC_P16, ADC_P13,
};

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����������
//  @param      array[]      ����������  
//  @param      low          ��������
//  @param      high      	 ��������
//  Sample usage:				quickSort(data, 0, ARRAY_SIZE - 1);    
//-------------------------------------------------------------------------------------------------------------------
//void quickSort(int array[], int low, int high) reentrant //�ɵݹ�ؼ��ʱ�ʶ
//{
//    if (low < high)
//		{
//        int pivotIndex = partition(array, low, high);    // ѡ����׼
//        quickSort(array, low, pivotIndex - 1);					 // ��������������еݹ�����
//        quickSort(array, pivotIndex + 1, high);
//    }
//}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ֺ���
//  @param      array[]      ����������  
//  @param      low          ��������
//  @param      high      	 ��������
//  Sample usage:				partition(data, 0, ARRAY_SIZE - 1);    
//-------------------------------------------------------------------------------------------------------------------
//int partition(int array[], int low, int high)
//{
//    int pivot = array[high];  									// ѡ�����һ��Ԫ����Ϊ��׼
//    int i = low - 1, j;

//    for (j = low; j < high; j++) {
//        if (array[j] < pivot) {
//            i++;
//            swap(&array[i], &array[j]);					// ����Ԫ��
//        }
//    }
//    swap(&array[i + 1], &array[high]); 					// ��׼�ع�
//    return i + 1;
//}



////-------------------------------------------------------------------------------------------------------------------
//// @brief		ADC��ֵ�˲�
////
////-------------------------------------------------------------------------------------------------------------------
void adc_acquire(uint8 count)
{
	uint8 adc_i,adc_j;
	uint32 sum[11];								
	for(adc_i = 0; adc_i < count; adc_i++)
	{
			for(adc_j = 1; adc_j <= ADC_CHANNEL; adc_j++)
			{
				adc[adc_j].data_0  = adc_once(adc_pin[adc_j], ADC_12BIT);
				sum[adc_j] += adc[adc_j].data_0;
			}	
	}
	for(adc_i = 1; adc_i <= ADC_CHANNEL; adc_i++)
	{
		adc[adc_i].data_result = sum[adc_i] / count;
		sum[adc_i] = 0;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ô������ֵ��һ������� ��ò�Ⱥ�
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void ADC_channel_quire(void) // ��ô������ֵ��һ������� ��ò�Ⱥ�
{
	float adc_tmp = 0.0;
	adc_acquire(ADC_CONVERT_TIMES);
//	adc_acquire(ADC_CONVERT_TIMES);

	// adc[1].data_0 = adc_mean_filter(ADC_P10,8);		//�ɼ�������ADC_P17��ѹ
	// adc[2].data_0 = adc_mean_filter(ADC_P11,8);		//�ɼ�������ADC_P13��ѹ
	// adc[3].data_0 = adc_mean_filter(ADC_P14,8);		//�ɼ�������ADC_P01��ѹ
	// adc[4].data_0 = adc_mean_filter(ADC_P17,8);		//�ɼ�������ADC_P05��ѹ
	// adc[5].data_0 = adc_mean_filter(ADC_P16,8);		//�ɼ�������ADC_P06��ѹ
	// adc[6].data_0 = adc_mean_filter(ADC_P06,8);		//�ɼ�������ADC_P14��ѹ
	// adc[7].data_0 = adc_mean_filter(ADC_P05,8);		//�ɼ�������ADC_P16��ѹ
	// adc[8].data_0 = adc_mean_filter(ADC_P01,8);		//�ɼ�������ADC_P01��ѹ
	// adc[9].data_0 = adc_mean_filter(ADC_P00,8);		//�ɼ�������ADC_P01��ѹ
	// adc[10].data_0 = adc_mean_filter(ADC_P13,8);		//�ɼ�������ADC_P01��ѹ
	// ��һ��������ֵ����ƫ������һ����0--100����
	//  adc1_L1_V = adc[1].data_0;
	//  adc1_L2_H = adc[2].data_0;
	//  adc1_M_H  = adc[3].data_0;
	//  adc1_R2_H = adc[4].data_0;
	//  adc1_R1_V = adc[5].data_0;
	//  adc2_L1_V = adc[6].data_0;
	//  adc2_L2_H = adc[7].data_0;
	//  adc2_M_H  = adc[8].data_0;
	//  adc2_R2_H = adc[9].data_0;
	//  adc2_R1_V =	adc[10].data_0;
	
	adc1_L1_V = (adc[1].data_result - adc[1].data_min) * 100.0 / (adc[1].data_max - adc[1].data_min);
	adc1_L2_H = (adc[2].data_result - adc[2].data_min) * 100.0 / (adc[2].data_max - adc[2].data_min);
	adc1_M_H = (adc[3].data_result - adc[3].data_min) * 100.0 / (adc[3].data_max - adc[3].data_min);
	adc1_R1_V = (adc[4].data_result - adc[4].data_min) * 100.0 / (adc[4].data_max - adc[4].data_min);
	adc1_R2_H = (adc[5].data_result - adc[5].data_min) * 100.0 / (adc[5].data_max - adc[5].data_min);

	 adc2_L1_V = (adc[6].data_result - adc[6].data_min) * 100.0 / (adc[6].data_max - adc[6].data_min);
	adc2_L2_H = (adc[7].data_result - adc[7].data_min) * 100.0 / (adc[7].data_max - adc[7].data_min);
	adc2_M_H = (adc[8].data_result - adc[8].data_min) * 100.0 / (adc[8].data_max - adc[8].data_min);
	adc2_R1_V = (adc[9].data_result - adc[9].data_min) * 100.0 / (adc[9].data_max - adc[9].data_min);
	adc2_R2_H = (adc[10].data_result - adc[10].data_min) * 100.0 / (adc[10].data_max - adc[10].data_min);

	if (adc1_L2_H < 0)	adc1_L2_H = 0;
	if (adc1_L1_V < 0)	adc1_L1_V = 0;
	if (adc1_M_H < 0)		adc1_M_H = 0;
	if (adc1_R1_V < 0)	adc1_R1_V = 0;
	if (adc1_R2_H < 0)	adc1_R2_H = 0;
	
	if (adc2_L2_H < 0)	adc2_L2_H = 0;
	if (adc2_L1_V < 0)	adc1_L1_V = 0;
	if (adc2_M_H < 0)		adc2_M_H = 0;
	if (adc2_R1_V < 0)	adc2_R1_V = 0;
	if (adc2_R2_H < 0)	adc2_R2_H = 0;

	// �õ���еĲ�ȺͲ�
	//  Err_1=Diff_Ratio_Sum_Diff(adc1_L1_V,adc1_L2_H,0,0,adc1_R2_H,adc1_R1_V,0,0.2,0.6,20);
	//  Err_2=Diff_Ratio_Sum_Diff(adc2_L1_V,adc2_L2_H,0,0,adc2_R2_H,adc2_R1_V,0,0.4,0.6,20);
	if(	adc1_L2_H + adc1_L1_V + adc1_M_H + adc1_R1_V + adc1_R2_H 
		+ adc2_L2_H + adc2_L1_V + adc2_M_H + adc2_R1_V + adc2_R2_H < ADC_OUTOFRANGE_LIMIT)
	err = 800;
	else 
ERR_process(); // ��鴦��ֵ
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȺͲʽ
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

	ans = (float)(Limit * (fz * 1.0 / (fm+0.1))); // ��ȺͲ�
	return ans;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȺͲʽ
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
		ans = (float)(Limit * (fz * 1.0 / (fm+0.1))); // ��ȺͲ�
	}
	else	ans = 0;
	return ans;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȺͲʽ
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
float Diff_Vert_Sum_Diff(float adc1, float adc2, float a, int Limit)
{

	float ans, fz, fm;
	int16 Diff_Mid;
	int16 Sum_Mid;
	if (adc1 > 1 || adc2 > 1)
	{
		Diff_Mid = adc1 - adc2;
		Sum_Mid = adc1 + adc2;
		fz = a * Diff_Mid;
		fm = a * Sum_Mid;
		ans = (float)(Limit * (fz * 1.0 / (fm+0.1))); // ��ȺͲ�
	}
	else	ans = 0;
	return ans;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��������Сֵ��ȡ
//  @param
//  @param
//  @return
//------------------------------------------------------------------------------------------------------------------
void resetelec_Init() // ����ѧϰ��ֵ��е������Сֵ
{
	if (resetelec_flag == 1)
	{
		uint16 i;
		resetelec_count = 0;
		// trace_flag = 1;
		// brusheless_flag = 1;
		// brusheless_duty = 800;
		// delay_ms(1200);
		delay_ms(6000);
		P50 = 0;
		printf("ready!\n");
		delay_ms(3000);
		P50 = 1;
		printf("start!\n");
		if (resetelec_count == 0)
		{
			for(i=1;i<=ADC_CHANNEL;i++)
			{
				adc[i].data_max = 0;
				adc[i].data_min = 5000;
			}
		}
		while (resetelec_count < ADC_RESET_TIMES)
		{
			adc_acquire(ADC_CONVERT_TIMES);
			for (i = 1; i <= 10; i++)
			{
				if (adc[i].data_result > adc[i].data_max)
					adc[i].data_max = adc[i].data_result;
				if (adc[i].data_result < adc[i].data_min)
					adc[i].data_min = adc[i].data_result;
				
			}
				printf("%d\n",resetelec_count);
				delay_ms(ADC_ACQUIRE_DELAY);
			resetelec_count++;
		}
		P50 = 0;
		printf("stop!\n");
		delay_ms(3000);
		P50 = 1;
		printf("go!\n");
		resetelec_count = 0;
		resetelec_flag = 0;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void ERR_process() // ����
{
	adc1_L2H_withL1V = sqrt(vector_ratio[0][0] * adc1_L2_H*adc1_L2_H + (1-vector_ratio[0][0]) * adc1_L1_V*adc1_L1_V);			// ��������ƫ�����
	adc1_R2H_withR1V = sqrt(vector_ratio[0][0] * adc1_R2_H*adc1_R2_H + (1-vector_ratio[0][0]) * adc1_R1_V*adc1_R1_V);
	
	adc1_MH_withL1V  = sqrt(vector_ratio[0][1] * adc1_M_H *adc1_M_H  + (1-vector_ratio[0][1]) * adc1_L1_V*adc1_L1_V);
	adc1_MH_withR1V  = sqrt(vector_ratio[0][1] * adc1_M_H *adc1_M_H  + (1-vector_ratio[0][1]) * adc1_R1_V*adc1_R1_V);
	
	adc2_L2H_withL1V = sqrt(vector_ratio[1][0] * adc2_L2_H*adc2_L2_H + (1-vector_ratio[1][0]) * adc2_L1_V*adc2_L1_V);
	adc2_R2H_withR1V = sqrt(vector_ratio[1][0] * adc2_R2_H*adc2_R2_H + (1-vector_ratio[1][0]) * adc2_R1_V*adc2_R1_V);
	
	adc2_MH_withL1V  = sqrt(vector_ratio[1][1] * adc2_M_H *adc2_M_H  + (1-vector_ratio[1][1]) * adc2_L1_V*adc2_L1_V);
	adc2_MH_withR1V  = sqrt(vector_ratio[1][1] * adc2_M_H *adc2_M_H  + (1-vector_ratio[1][1]) * adc2_R1_V*adc2_R1_V);
	
	
//	Err_Hori = Diff_Hori_Sum_Diff(adc1_L2_H, adc1_R2_H, 1, 20);
//	Err_2_Hori = Diff_Hori_Sum_Diff(adc2_L2_H, adc2_R2_H, 1, 20);
//	
//	Err_Vert = Diff_Vert_Sum_Diff(adc1_L1_V, adc1_R1_V, 1, 20);
//	Err_2_Vert = Diff_Vert_Sum_Diff(adc2_L1_V, adc2_R1_V, 1, 20);
//	
//	Err_Inclined_left=Diff_Vert_Sum_Diff(adc1_L2_H+compensation_left,adc1_M_H, 1, 20)*4;
//	Err_Inclined_right=Diff_Vert_Sum_Diff(adc1_M_H, adc1_R2_H+compensation_right, 1, 20)*4;
	
	Err_Hori = Diff_Hori_Sum_Diff(adc1_L2H_withL1V, adc1_R2H_withR1V, 1, 20);											// �����Ⱥ����
	Err_2_Hori = Diff_Hori_Sum_Diff(adc2_L2H_withL1V, adc2_R2H_withR1V, 1, 20);
	
	Err_Vert = Diff_Vert_Sum_Diff(adc1_L1_V, adc1_R1_V, 1, 20);
	Err_2_Vert = Diff_Vert_Sum_Diff(adc2_L1_V, adc2_R1_V, 1, 20);
	
	Err_Inclined_left=Diff_Vert_Sum_Diff(adc1_L2_H+compensation_left,adc1_M_H, 1, 20)*4;
	Err_Inclined_right=Diff_Vert_Sum_Diff(adc1_M_H, adc1_R2_H+compensation_right, 1, 20)*4;
	
	// Err_Inclined_left=(Diff_Vert_Sum_Diff(adc1_L2_H, adc1_R2_H, 1, 20)+Diff_Vert_Sum_Diff(adc2_L2_H, adc2_R2_H, 1, 20));
	// Err_Inclined_right=(Diff_Vert_Sum_Diff(adc1_L2_H, adc1_R2_H, 1, 20)+Diff_Vert_Sum_Diff(adc2_L2_H, adc2_R2_H, 1, 20));
	//  Err_Hori=Diff_Ratio_Sum_Diff(adc1_L1_V,0,0,0,0,adc1_R1_V,0,0,1,20);
	//  Err_2_Hori=Diff_Ratio_Sum_Diff(adc2_L1_V,0,0,0,0,adc2_R1_V,0,0,1,20);
	//  Err_Vert=Diff_Ratio_Sum_Diff(0,adc1_L2_H,0,0,adc1_R2_H,0,0,1,0,20);
	//  Err_2_Vert=Diff_Ratio_Sum_Diff(0,adc2_L2_H,0,0,adc2_R2_H,0,0,1,0,20);
	
	err_last = err;
	if (Swerve_flag == 0 && ((adc1_L2_H > 80 || adc1_R2_H > 80) && adc1_M_H > 60&&adc2_M_H<120)) // ����ֵ��е���һ����ֵ��ʱ��ֱ��ת����ʱ��
	{
		Swerve_flag = 1;
		err = curve_convert_ratio[0][0] * Err_Hori   + curve_convert_ratio[0][1] * Err_Vert 				// ������
				+ curve_convert_ratio[0][2] * Err_2_Hori + curve_convert_ratio[0][3] * Err_2_Vert;
	}
	else if (Swerve_flag == 1)
	{
		err = curve_convert_ratio[0][0] * Err_Hori   + curve_convert_ratio[0][1] * Err_Vert 
				+ curve_convert_ratio[0][2] * Err_2_Hori + curve_convert_ratio[0][3] * Err_2_Vert;
	}
	
	if (Swerve_flag == 1 && (adc2_R2_H < 15 && adc2_L2_H < 15 && adc2_M_H > 30 && adc1_M_H > 30)) // ֱ���������ֵ
	{
		Swerve_flag = 0;
		err = straight_convert_ratio[0][0] * Err_Hori  + straight_convert_ratio[0][1] * Err_2_Hori 
			  + straight_convert_ratio[0][2] * Err_Vert;
	}
	if (Swerve_flag == 0)
	{
		err = straight_convert_ratio[1][0] * Err_Hori  + straight_convert_ratio[1][1] * Err_2_Hori 
			  + straight_convert_ratio[1][2] * Err_Vert;
	}
	
	// if (Swerve_flag == 0 && ((adc1_L2_H > 80 || adc1_R2_H > 80)&&adc2_M_H<120)) // ����ֵ��е���һ����ֵ��ʱ��ֱ��ת����ʱ��
	// {
	// 	Swerve_flag = 1;
	// 	BEEP=1;
	// 	err = 1.1 * Err_Hori + 0.5 * Err_Vert + 0.8 * Err_2_Hori + 0.5 * Err_2_Vert;
	// }
	// else if (Swerve_flag == 1)
	// {
	// 	BEEP=1;
	// 	err = 1.1 * Err_Hori + 0.5 * Err_Vert + 0.8 * Err_2_Hori + 0.5 * Err_2_Vert;
	// }
	// if (Swerve_flag == 1 && (adc2_R2_H < 15 && adc2_L2_H < 15 && adc2_M_H > 30)) // ֱ���������ֵ
	// {
	// 	BEEP=0;
	// 	Swerve_flag = 0;
	// 	err = 0.6 * Err_Hori + 0.6 * Err_Vert + 0.8 * Err_2_Hori;
	// }
	// if (Swerve_flag == 0)
	// {
	// 	BEEP=0;
	// 	err = 0.6 * Err_Hori + 0.6 * Err_Vert + 0.8 * Err_2_Hori;
	// }
	
	
	//	if(RoundAbout==3)
	//		{
	//       if(err>6)
	//			{err=6;}
	//			else if(err<-6)
	//			{err=-6;}
	//	}
	// else if(Swerve_flag==1)
	//	err=0.6*Err_Hori+0.6*Err_Vert;
	// else if(adc1_R2_H>40||adc1_L2_H>40)//ֱ���������ֵ
	// err=0.6*Err_Hori+0.5*Err_Vert;
	// err = Err_1 + Err_2;
	// 	 if(Err_2_Hori>6&&Err_Hori>8)
	//  {
	//  		err=1.1*Err_Hori+0.5*Err_Vert+0.8*Err_2_Hori+0.5*Err_2_Vert;
	//  }
	//  else if(Err_2_Hori<-4&&Err_Hori<-6)
	//  {
	//  		err=1.1*Err_Hori+0.5*Err_Vert+0.8*Err_2_Hori+0.5*Err_2_Vert;
	//  }
	//  	else
	//  {

	//  }
}
