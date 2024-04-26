///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020,��ɿƼ�
// * All rights reserved.
// * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
// *
// * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
// * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
// *
// * @file       		isr
// * @company	   		�ɶ���ɿƼ����޹�˾
// * @author     		��ɿƼ�(QQ790875685)
// * @version    		�鿴doc��version�ļ� �汾˵��
// * @Software 			MDK FOR C251 V5.60
// * @Target core		STC32F12K
// * @Taobao   			https://seekfree.taobao.com/
// * @date       		2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"
#include "mycode.h"

float get_error1=0;
uint8 spi_num_type = 0;
uint32  counter_timer= 0;
bit spi_w=0;//infer work

//UART1�ж�
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
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        //�����Զ�����
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

//UART2�ж�
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
		//�������ݼĴ���Ϊ��S2BUF

	}
}


//UART3�ж�
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
		//�������ݼĴ���Ϊ��S3BUF

	}
}


//UART4�ж�

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
		//�������ݼĴ���Ϊ��S4BUF;
		if(wireless_type == WIRELESS_SI24R1)
        {
            wireless_uart_callback();           //����ת���ڻص�����
        }
        else if(wireless_type == WIRELESS_CH9141)
        {
            bluetooth_ch9141_uart_callback();   //����ת���ڻص�����
        }

	}
}

#define LED P52

void INT0_Isr() interrupt 0
{
	LED = 0;	//����LED
}

void INT1_Isr() interrupt 2
{

}

void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //����жϱ�־
}

void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //����жϱ�־
}


void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //����жϱ�־
}

volatile uint32 SysTickFlag = 0; // ��������ʼÿ1ms��1
uint32 Tick5ms_Flag = 0; // ��������ʼÿ1ms��1


void TM0_Isr() interrupt 1
{
	SysTickFlag++;
}

void TM1_Isr() interrupt 3
{
	Tick5ms_Flag++;
	if(Tick5ms_Flag%4==0)
	{
   get_switch();
	}
	get_error(&Forward_PID,SPI_float);
  get_error1=SPI_float;
	counte_quire();  // ����������LR
	motor_output();  // ������PWM 
	MPU_Process();
	Element_Idef();
}



void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //����жϱ�־
	
}

void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; //����жϱ�־
	
}

extern void pit_callback(void);
void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //����жϱ�־
//	ccd_collect();	 //CCD�ɼ�����
//	pit_callback();
}



// SPI��
void  SPI_Isr()   interrupt 9
{
	if(SWITCH4_FLAG==0)//���ڷ����׶���ȡ������Ϊ�������
  {
    spi_w=1;
		SPDAT = SPDAT;
		Receive_buff[++Receive_ptr] = SPDAT;
		if(Receive_buff[1] == SPIFrameFloat[0])	spi_num_type = 1;
		else if(Receive_buff[1] == SPIFrameUint[0])	spi_num_type = 2;
		else if(Receive_buff[1] == SPIFrameUchar[0])	spi_num_type = 3;
		else	Receive_ptr = 0;
		if(Receive_ptr == 6 && spi_num_type==1)
		{
			if(Receive_buff[Receive_ptr]==SPIFrameFloat[1])	
			{
				FloatLongType spi_tmp;
				spi_tmp.ldata = (unsigned long int)Receive_buff[2]
												|(unsigned long int)Receive_buff[3] << 8
												|(unsigned long int)Receive_buff[4] << 16
												|(unsigned long int)Receive_buff[5] << 24;
				SPI_float = spi_tmp.fdata;
			}
			Receive_ptr=0;
		}
		else if(Receive_ptr == 4 && spi_num_type==2)
		{
			if(Receive_buff[4]==SPIFrameUint[1]) 
			{
				SPI_uint = (unsigned int)Receive_buff[2]
									|(unsigned int)Receive_buff[3] << 8;
			}
			Receive_ptr = 0; 
		}
		else if(Receive_ptr == 3 && spi_num_type==3)
		{
			if(Receive_buff[3]!=SPIFrameUchar[1])	SPI_uchar = Receive_buff[2];
			Receive_ptr = 0;
		}
		SPIF = 1;
	}



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