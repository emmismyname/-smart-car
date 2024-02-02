///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020,��ɿƼ�
// * All rights reserved.
// * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
// *
// * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
// * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
// *
// * @file       		isr
// * @company	   		�ɶ���ɿƼ����޹�˾
// * @author     		��ɿƼ�(QQ790875685)
// * @version    		�鿴doc��version�ļ� �汾˵��
// * @Software 			MDK FOR C251 V5.60
// * @Target core		STC32G12K128
// * @Taobao   			https://seekfree.taobao.com/
// * @date       		2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"

// UART1�ж�
void UART1_Isr() interrupt 4
{
    uint8 res;
    static uint8 dwon_count;
    if (UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if (UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        // �����Զ�����
        if (res == 0x7F)
        {
            if (dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

// UART2�ж�
void UART2_Isr() interrupt 8
{
    if (UART2_GET_TX_FLAG)
    {
        UART2_CLEAR_TX_FLAG;
        busy[2] = 0;
    }
    if (UART2_GET_RX_FLAG)
    {
        UART2_CLEAR_RX_FLAG;
        // �������ݼĴ���Ϊ��S2BUF
    }
}

// UART3�ж�
void UART3_Isr() interrupt 17
{
    if (UART3_GET_TX_FLAG)
    {
        UART3_CLEAR_TX_FLAG;
        busy[3] = 0;
    }
    if (UART3_GET_RX_FLAG)
    {
        UART3_CLEAR_RX_FLAG;
        // �������ݼĴ���Ϊ��S3BUF
    }
}

// UART4�ж�
void UART4_Isr() interrupt 18
{
    if (UART4_GET_TX_FLAG)
    {
        UART4_CLEAR_TX_FLAG;
        busy[4] = 0;
    }
    if (UART4_GET_RX_FLAG)
    {
        UART4_CLEAR_RX_FLAG;

        // �������ݼĴ���Ϊ��S4BUF;
        if (wireless_type == WIRELESS_SI24R1)
        {
            wireless_uart_callback(); // ����ת���ڻص�����
        }
        else if (wireless_type == WIRELESS_CH9141)
        {
            bluetooth_ch9141_uart_callback(); // ����ת���ڻص�����
        }
        else if (wireless_type == WIRELESS_CH573)
        {
//            wireless_ch573_callback(); // CH573����ģ��ص�����
        }
    }
}

#define LED P52
void INT0_Isr() interrupt 0
{
    LED = 0; // ����LED
}
void INT1_Isr() interrupt 2
{
}
void INT2_Isr() interrupt 10
{   //�������
    // if(isr_count_flag > isr_counter_start + 800)
    // {HRTRIG_flag=0;}
    // BEEP=1;
    // delay_ms(20);
    // BEEP=0;
    INT2_CLEAR_FLAG; // ����жϱ�־
}
void INT3_Isr() interrupt 11//
{   //�������
    // if(isr_count_flag > isr_counter_start + 800)
    // {HRTRIG_flag=0;}
    // BEEP=1;
    // delay_ms(20);
    // BEEP=0;
    INT3_CLEAR_FLAG; // ����жϱ�־
}

void INT4_Isr() interrupt 16
{
    INT4_CLEAR_FLAG; // ����жϱ�־
}

void TM0_Isr() interrupt 1
{
}
void TM1_Isr() interrupt 3
{
}
void TM2_Isr() interrupt 12
{

    TIM2_CLEAR_FLAG; // ����жϱ�־
}
void TM3_Isr() interrupt 19
{
    TIM3_CLEAR_FLAG; // ����жϱ�־
}

uint32 isr_count_flag=0;    // ������ʾʱ��
extern uint32 isr_counter_start;
extern float Target_Speed_L; // ����Ŀ���ٶ�
extern float Target_Speed_R; // ����Ŀ���ٶ�
extern uint8 Dir_car;        // ����dir_car=1 L=1 R=0  ;dir_car=0 ���� L=0 R=1
extern int OUTP_flag;        // ��ʱ����������
extern int running_mode;
int iu = 50;

void TM4_Isr() interrupt 20//20ms
{
    counte_quire();  // ����������LR
    motor_output();  // ������PWM
    // Hall_detection();//�������
    if (isr_count_flag % 2 == 1)//10ms
    {
        MPU_Process();
        ADC_channel_quire(); // ��õ��ֵ  
        // Element_Idef();      // ģʽ����
        // if (count_flag % 600 == 1)
        // {
        //     distance_L = 0;
        //     distance_R = 0;
        //     yaw = 0.0f;
        // }
        // mode_choose(running_mode);
    }
    // if(running_mode==8||running_mode==15||running_mode==16)
    // {
    //     err_angle_rate=PID_Loc_angle_Control(&Pout_PID, yaw_target - yaw_output)-(z_gyro-5);
    // }
    // if (isr_count_flag % 2 == 1)//10ms
    // {
    //     TOF_detection();//������
    // }
    if (isr_count_flag % 100 == 1)//100ms
    {
        ADC_battery_quire(); // ��õ�ص�ѹ
    }

    // if (OUTP_flag != 0)//������ʱ�򷢳�����
    // {
    //     outp(outp_angle,outp_distance,outp_radiu);
    // }

    // if (isr_count_flag % 200 == 1)
    // {
    //     iu++;
    //     if (iu % 5 == 1)
    //     {
    //         duty_L = 1000;
    //         duty_R = 0;
    //     }
    //     else if (iu % 5 == 2)
    //     {
    //         duty_L = -1000;
    //         duty_R = 0;
    //     }
    //     else if (iu % 5 == 3)
    //     {
    //         duty_L = 0;
    //         duty_R = 1000;
    //     }
    //     else if (iu % 5 == 4)
    //     {
    //         duty_L = 0;
    //         duty_R = -1000;
    //     }
    // }

    // ccd_collect();	 //CCD�ɼ�����
    // ״̬����ȡ����״̬
    isr_count_flag++; // ÿ5ms����һ��
    TIM4_CLEAR_FLAG; // ����жϱ�־
}

// void  INT0_Isr()  interrupt 0;
// void  TM0_Isr()   interrupt 1;
// void  INT1_Isr()  interrupt 2;
// void  TM1_Isr()   interrupt 3;
// void  UART1_Isr() interrupt 4;
// void  ADC_Isr()   interrupt 5;
// void  LVD_Isr()   interrupt 6;
// void  PCA_Isr()   interrupt 7;
// void  UART2_Isr() interrupt 8;
// void  SPI_Isr()   interrupt 9;
// void  INT2_Isr()  interrupt 10;
// void  INT3_Isr()  interrupt 11;
// void  TM2_Isr()   interrupt 12;
// void  INT4_Isr()  interrupt 16;
// void  UART3_Isr() interrupt 17;
// void  UART4_Isr() interrupt 18;
// void  TM3_Isr()   interrupt 19;
// void  TM4_Isr()   interrupt 20;
// void  CMP_Isr()   interrupt 21;
// void  I2C_Isr()   interrupt 24;
// void  USB_Isr()   interrupt 25;
// void  PWM1_Isr()  interrupt 26;
// void  PWM2_Isr()  interrupt 27;