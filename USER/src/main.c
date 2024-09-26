
/*********************************************************************************************************************
 * @file       		main (Signal_Board)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32F12K
 * @date       		2024-03-04  Ver0.0.0
									2024-03-08  Ver0.1.1
									2024-03-XX  Ver0.1.2

 ********************************************************************************************************************/
#include "headfile.h"

/*
 *关于内核频率的设定，可以查看board.h文件
 *在board_init中,已经将P54引脚设置为复位
 *如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */
 
	
void main()
{
	clock_init(SYSTEM_CLOCK_52M);		// 初始化系统频率
	board_init();			 							// 初始化寄存器
	
	allPins_init();									// 初始化引脚
	allADC_init();                  // 初始化ADC
	pit_timer_ms(TIM_3,5);					// 初始化定时器
	pit_timer_ms(TIM_0,5);					// 初始化定时器
	wireless_uart_init();						// 初始化无线串口
//	dl1a_init();										// 初始化TOF
	spi_master_init();							// 初始化SPI(作为主机)
	
	EnableSPIIRQ();									// 使能SPI中断
	EnableGlobalIRQ();            	// 打开总中断
				
	resetelec_flag = 1;							// 上电扫电感
	resetelec_Init();

  while(1)
	 {
			resetelec_Init();
		 wireless_send_SBtoPC();			// 无线串口发送数据至上位机
		 wireless_read(1);						// 读取指令
		 spi_send_float(err);					// SPI发送err至主板
//		 delay_ms(10);
//		 printf("%f",err);
   }
}

