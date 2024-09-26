
/*********************************************************************************************************************
 * @file       		adc
 * @date       		2024-03-06
 * @note    
 ********************************************************************************************************************/


#ifndef _ADC_H
#define _ADC_H

#include "common.h"

//此枚举定义不允许用户修改
typedef enum
{
	ADC_P10 = 0     , 
	ADC_P11         , // 0x01
	ADC_P12         ,	//STC32系列和STC16F没有这个引脚，仅做占位使用
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
	ADC_POWR = 0x0f	, //内部AD 1.19V
} ADCN_enum;


//此枚举定义不允许用户修改
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


//此枚举定义不允许用户修改
typedef enum    // 枚举ADC通道
{

  ADC_12BIT=0,    //12位分辨率
	ADC_11BIT,		//11位分辨率
	ADC_10BIT,		//10位分辨率
	ADC_9BIT,    	//9位分辨率
	ADC_8BIT,     	//8位分辨率

}ADCRES_enum;

void adc_init(ADCN_enum adcn,ADC_SPEED_enum speed);
uint16 adc_once(ADCN_enum adcn,ADCRES_enum resolution);
//uint16 adc_mean_filter(ADCN_enum adcn, uint8 count);


#endif