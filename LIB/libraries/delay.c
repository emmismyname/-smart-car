
/*********************************************************************************************************************
 * @file       		delay
 * @date       		2024-03-06
 * @note    
 ********************************************************************************************************************/


#include "delay.h"
#include "board.h"
#include "intrins.h"

vuint16 the_delay_ms = 0;
vuint16 the_delay_us = 0;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      软件延时函数初始化
//  @param      void         
//  @return     void
//  Sample usage:               无需用户调用，用户请使用h文件中的宏定义
//-------------------------------------------------------------------------------------------------------------------
void delay_init(void)
{
	the_delay_ms = sys_clk / 15000;
	the_delay_us = sys_clk / 17500000;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      软件延时函数(这是一个不准确的延时)
//  @param      x            	需要延时的时间（ms）
//  @return     void
//  Sample usage:               无需用户调用，用户请使用h文件中的宏定义
//-------------------------------------------------------------------------------------------------------------------
void delay_ms(uint16 ms)
{
	vuint16 i;
	do {
			i = the_delay_ms;
			while(--i);
	   }while(--ms);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      软件延时函数(这是一个不准确的延时)
//  @param      x            	需要延时的时间（us）
//  @return     void
//  Sample usage:               无需用户调用，用户请使用h文件中的宏定义
//-------------------------------------------------------------------------------------------------------------------
void delay_us(uint32 us)
{
	uint16 i;
	do {
			i = the_delay_us;
			while(--i);
	   }while(--us);
}
