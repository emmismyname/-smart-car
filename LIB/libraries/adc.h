
/*********************************************************************************************************************
 * @file       		adc
 * @date       		2024-03-06
 * @note    
 ********************************************************************************************************************/


#ifndef _ADC_H
#define _ADC_H

#include "common.h"

//��ö�ٶ��岻�����û��޸�
typedef enum
{
	ADC_P10 = 0     , 
	ADC_P11         , // 0x01
	ADC_P12         ,	//STC32ϵ�к�STC16Fû��������ţ�����ռλʹ��
	ADC_P13         , // 0x03
	ADC_P14         , // 0x04
	ADC_P15         , // 0x05
	ADC_P16         , // 0x06
	ADC_P17         , // 0x07
	
	ADC_P00         , // 0x08
	ADC_P01         , // 0x09
	ADC_P02        	, // 0x0a
	ADC_P03        	, // 0x0b
	ADC_P04        	, // 0x0c
	ADC_P05        	, // 0x0d
	ADC_P06        	, // 0x0e
	ADC_POWR = 0x0f	, //�ڲ�AD 1.19V
} ADCN_enum;


//��ö�ٶ��岻�����û��޸�
typedef enum
{
	ADC_SYSclk_DIV_2 = 0,
	ADC_SYSclk_DIV_4,
	ADC_SYSclk_DIV_6,
	ADC_SYSclk_DIV_8,
	ADC_SYSclk_DIV_10,
	ADC_SYSclk_DIV_12,
	ADC_SYSclk_DIV_14,
	ADC_SYSclk_DIV_16,
	ADC_SYSclk_DIV_18,
	ADC_SYSclk_DIV_20,
	ADC_SYSclk_DIV_22,
	ADC_SYSclk_DIV_24,
	ADC_SYSclk_DIV_26,
	ADC_SYSclk_DIV_28,
	ADC_SYSclk_DIV_30,
	ADC_SYSclk_DIV_32,
} ADC_SPEED_enum;


//��ö�ٶ��岻�����û��޸�
typedef enum    // ö��ADCͨ��
{

  ADC_12BIT=0,    //12λ�ֱ���
	ADC_11BIT,		//11λ�ֱ���
	ADC_10BIT,		//10λ�ֱ���
	ADC_9BIT,    	//9λ�ֱ���
	ADC_8BIT,     	//8λ�ֱ���

}ADCRES_enum;

void adc_init(ADCN_enum adcn,ADC_SPEED_enum speed);
uint16 adc_once(ADCN_enum adcn,ADCRES_enum resolution);
//uint16 adc_mean_filter(ADCN_enum adcn, uint8 count);


#endif