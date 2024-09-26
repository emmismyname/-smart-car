
/*********************************************************************************************************************
 * @file       		isr
 * @date       		2024-03-06
 * @note   
 ********************************************************************************************************************/


#include "isr.h"

uint16 tim3_cnt = 0;

//UART1中断
void UART1_Isr() interrupt 4
{
    uint8 res;
	static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
				res = SBUF;
				if(wireless_type == WIRELESS_SI24R1)
        {
            wireless_uart_callback();           //无线转串口回调函数
        }
        UART1_CLEAR_RX_FLAG;
        //程序自动下载
        if(res == 0x7F)
        {
            if(dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

//UART2中断
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		//接收数据寄存器为：S2BUF

	}
}


//UART3中断
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		//接收数据寄存器为：S3BUF

	}
}


//UART4中断
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG)
	{
        UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;

		//接收数据寄存器为：S4BUF;
	}
}

void INT0_Isr() interrupt 0
{
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //清除中断标志
}


void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //清除中断标志
	
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //清除中断标志
}

void TM0_Isr() interrupt 1
{

}
void TM1_Isr() interrupt 3
{

}
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //清除中断标志
	
}

extern bit re_flag;

void TM3_Isr() interrupt 19
{	
	TIM3_CLEAR_FLAG; //清除中断标志
	if(!resetelec_flag) ADC_channel_quire();
	dl1a_get_distance();

	if(!resetelec_flag) tim3_cnt++;
	if(tim3_cnt==100)
	{
		tim3_cnt = 0;
		LED = !LED;
	}
}
extern void pit_callback(void);
void TM4_Isr() interrupt 20
{


	TIM4_CLEAR_FLAG; //清除中断标志
//	ccd_collect();	 //CCD采集数据

}

//SPI发
void  SPI_Isr()   interrupt 9  //SPI中断发送
{
	SPIF=1;//清除中断标志
	SS_2=1;//拉高ss的管脚
	spi_busy=0;
}

//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;

//void  CAN1_Isr()  interrupt 28;
//void  CAN2_Isr()  interrupt 29;
//void  LIN_Isr()   interrupt 30;
//void  RTC_Isr()   interrupt 36;
//void  P0_Isr()    interrupt 37;
//void  P1_Isr()  	interrupt 38;
//void  P2_Isr()  	interrupt 39;
//void  P3_Isr()  	interrupt 40;
//void  P4_Isr()  	interrupt 41;
//void  P5_Isr()  	interrupt 42;
//void  P6_Isr()  	interrupt 43;
//void  P7_Isr()  	interrupt 44;
//void  P8_Isr()  	interrupt 45;
//void  P9_Isr()  	interrupt 46;
//void  M2M_DMA_Isr() 	interrupt 47;
//void  ADC_DMA_Isr() 	interrupt 48;
//void  SPI_DMA_Isr()   interrupt 49;
//void  UR1T_DMA_Isr()  interrupt 50;
//void  UR2R_DMA_Isr()  interrupt 51;
//void  UR2T_DMA_Isr()  interrupt 52;
//void  UR2R_DMA_Isr()  interrupt 53;
//void  UR3T_DMA_Isr()  interrupt 54;
//void  UR3R_DMA_Isr()  interrupt 55;
//void  UR4T_DMA_Isr()  interrupt 56;
//void  UR4R_DMA_Isr()  interrupt 57;
//void  LCM_DMA_Isr()   interrupt 58;
//void  LCM_Isr()  			interrupt 59;
//void  I2CT_DMA_Isr()  interrupt 60;
//void  I2CR_DMA_Isr()  interrupt 61;
//void  I2ST_DMA_Isr()  interrupt 62;
//void  I2SR_DMA_Isr()  interrupt 63;
//void  CAN1_Isr()  		interrupt 64;