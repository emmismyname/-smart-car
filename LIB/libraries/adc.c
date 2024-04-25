
/*********************************************************************************************************************
 * @file       		adc
 * @date       		2024-03-06
 * @note    
 ********************************************************************************************************************/


#include "adc.h"
#include "intrins.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC��ʼ��
//  @param      adcn            ѡ��ADCͨ��
//  @param      speed      		ADCʱ��Ƶ��
//  @return     void
//  Sample usage:               adc_init(ADC_P10,ADC_SYSclk_DIV_2);//��ʼ��P1.0ΪADC����,ADCʱ��Ƶ�ʣ�SYSclk/2
//-------------------------------------------------------------------------------------------------------------------
void adc_init(ADCN_enum adcn,ADC_SPEED_enum speed)
{
	ADC_CONTR |= 1<<7;				//1 ���� ADC ��Դ
	
	ADC_CONTR &= (0xF0);			//���ADC_CHS[3:0] �� ADC ģ��ͨ��ѡ��λ
	ADC_CONTR |= adcn;
	
	if((adcn >> 3) == 1) //P0.0
	{
		//IO����Ҫ����Ϊ��������
		P0M0 &= ~(1 << (adcn & 0x07));
		P0M1 |= (1 << (adcn & 0x07));
	}
	else if((adcn >> 3) == 0) //P1.0	
	{
		//IO����Ҫ����Ϊ��������
		P1M0 &= ~(1 << (adcn & 0x07));
	  P1M1 |= (1 << (adcn & 0x07));
	}

	ADCCFG |= speed&0x0F;			//ADCʱ��Ƶ��SYSclk/2/speed&0x0F;
	
	ADCCFG |= 1<<5;						//ת������Ҷ��롣 ADC_RES �������ĸ� 2 λ�� ADC_RESL �������ĵ� 8 λ��
	
//	ADCTIM = 0x3C; 						//CSSETUP:0(׼��ʱ��) CSHOLD:01(����ʱ��) SMPDUTY:01110(����ʱ��)
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADCת��һ��
//  @param      adcn            ѡ��ADCͨ��
//  @param      resolution      �ֱ���
//  @return     void
//  Sample usage:               adc_once(ADC_P10, ADC_10BIT);
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_once(ADCN_enum adcn,ADCRES_enum resolution)
{
	uint16 adc_value;
	
	ADC_CONTR &= (0xF0);			//���ADC_CHS[3:0] �� ADC ģ��ͨ��ѡ��λ
	ADC_CONTR |= adcn;
	
	ADC_CONTR |= 0x40;  			// ���� AD ת��
//	while (!(ADC_CONTR & 0x20));  	// ��ѯ ADC ��ɱ�־
	ADC_CONTR &= ~0x20;  			// ����ɱ�־
	
	adc_value = ADC_RES;  			//�洢 ADC �� 12 λ����ĸ� 4 λ
	adc_value <<= 8;
	adc_value |= ADC_RESL;  		//�洢 ADC �� 12 λ����ĵ� 8 λ
	
	ADC_RES = 0;
	ADC_RESL = 0;
	
	adc_value >>= resolution;		//ȡ����λ
	
	return adc_value;
}