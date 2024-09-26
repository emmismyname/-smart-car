
/*********************************************************************************************************************
 * @file       		init
 * @date       		2024-03-06
 * @note    
 ********************************************************************************************************************/

#include "init.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC口初始化
//  @param      void
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void allADC_init(void)
{
	adc_init(ADC_P00,ADC_SYSclk_DIV_2);
	adc_init(ADC_P01,ADC_SYSclk_DIV_2);
	adc_init(ADC_P02,ADC_SYSclk_DIV_2);
	adc_init(ADC_P03,ADC_SYSclk_DIV_2);
	adc_init(ADC_P04,ADC_SYSclk_DIV_2);
	adc_init(ADC_P05,ADC_SYSclk_DIV_2);
	adc_init(ADC_P06,ADC_SYSclk_DIV_2);
	adc_init(ADC_P13,ADC_SYSclk_DIV_2);
	adc_init(ADC_P14,ADC_SYSclk_DIV_2);
	adc_init(ADC_P15,ADC_SYSclk_DIV_2);
	adc_init(ADC_P16,ADC_SYSclk_DIV_2);
	adc_init(ADC_P17,ADC_SYSclk_DIV_2);
}


void allPins_init(void)
{
	gpio_mode(P5_0,GPO_PP);					// 将P5.0设置为推挽输出
	
}

void nvic_init(void)
{
	NVIC_SetPriority(SPI_IRQn,3);
//	NVIC_SetPriority(SPI_IRQn,3);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi初始化
//  @param      无				
//  @param      
//  @return     无
//-------------------------------------------------------------------------------------------------------------------
void spi_master_init()
{
		PSPIH = 1;PSPI = 1;					//SPI中断设置为最高优先级
		spi_init(SPI_CH2, SPI_CH2_SCLK_P25 , SPI_CH2_MOSI_P23, SPI_CH2_MISO_P24, 0, MASTER, SPI_SYSclk_DIV_16);	
		SS_2=1;
		spi_busy=0;
		SPSTAT = 0xC0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi接收初始化
//  @param      无				
//  @param      
//  @return     无
//-------------------------------------------------------------------------------------------------------------------
void spi_slave_init()
{
//		PSPIH = 1;PSPI = 1;					//SPI中断设置为最高优先级
		spi_init(SPI_CH2, SPI_CH2_SCLK_P25 , SPI_CH2_MOSI_P23, SPI_CH2_MISO_P24, 0, SLAVE, SPI_SYSclk_DIV_16);	
		SPSTAT = 0xC0;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      使能SPI中断
//  @param      无				
//  @param      
//  @return     无
//-------------------------------------------------------------------------------------------------------------------
void EnableSPIIRQ(void)
{	
	SS_2 = 0;
	ESPI = 1;
}