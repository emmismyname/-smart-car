/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2023-07-27

 ********************************************************************************************************************/
#include "headfile.h"
#include "mycode.h"


// �����ں�Ƶ�ʵ��趨�����Բ鿴board.h�ļ�
// ��board_init��,�Ѿ���P54��������Ϊ��λ
// �����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����


void main()
{ 
		//���س�ʼ��������
		EAXFR=1;//ʹ�ܷ���XFR
		clock_init(SYSTEM_CLOCK_52M);	// ��ʼ��ϵͳƵ��,��ɾ���˾���롣
		board_init();					// ��ʼ���Ĵ���,��ɾ���˾���롣
		DisableGlobalIRQ(); // 	
		ctimer_count_init(SPEEDR_PLUSE);					// ��ʼ����ʱ��4��Ϊ�ⲿ����
		ctimer_count_init(SPEEDL_PLUSE);					// ��ʼ����ʱ��3��Ϊ�ⲿ����
		spi_slave_init();							// ��ʼ��SPI(��Ϊ�ӻ�)		
		wireless_uart_init();						// ����ת����ģ���ʼ��
	
		pit_timer_ms(TIM_1, 5);										// ʹ�ö�ʱ��4�������жϣ�ʱ��5msһ�Ρ��������
			// �˴���д�û����� ���������ʼ�������
	//  gpio_mode(P0_5, GPO_PP);									// P64��������Ϊ�������
		motor_init();
		switch_init();                //������ʼ��
		iap_init();				// ��ʼ��EEPROM
		EnableSPIIRQ();								// ʹ��SPI�ж�
		EnableGlobalIRQ();						// ʹ�����ж�
		//20  0.6  40
	  pid_all_init();//��ʼ��
		disenablemotor();//�رյ��
		car_init(&jiangxing_No_1);//��״̬��ʼ��
		
		Speed_PID_L.KP=33.0f;
		Speed_PID_L.KI=3.4f;
		Speed_PID_L.KD=0.0f;	
		Speed_PID_R.KP=33.0f;
		Speed_PID_R.KI=3.4f;
		Speed_PID_R.KD=0.0f;
		Forward_PID.KP=-3;
		Forward_PID.KI=-0.01;
		Forward_PID.KD=-40;
    gyro_d=0.01;

		while (MPU_Init())
		{
			printf("STC32 MPU6050 GYRO Init test!  \r\n");
		}
		MPU6050_DMP_Init();
		printf("STC32 MPU6050 DMP test!  \r\n");
  
	
	  get_switch();//��ȡ��ʼ��ʱ��Ĳ���״̬�������ж��Ƿ�Ҫ��ȡeeprom����ֵ
    if(SWITCH3_FLAG==1)
		{
     read_all_par();
		}
		else//ʹ���趨��ֵ
		{
			car_init(&jiangxing_No_1);//��״̬��ʼ��
				
				Speed_PID_L.KP=33.0f;
				Speed_PID_L.KI=3.4f;
				Speed_PID_L.KD=0.0f;	
				Speed_PID_R.KP=33.0f;
				Speed_PID_R.KI=3.4f;
				Speed_PID_R.KD=0.0f;
				Forward_PID.KP=-3;
				Forward_PID.KI=-0.01;
				Forward_PID.KD=-40;
				gyro_d=0.01;
		}
  

	 // �˴���д�û����� ���������ʼ�������
    while(1)	
    {
			
			enablemotor();//ʹ�ܵ��	
			MPU_Acquire();			
			wireless_read(1);
			wireless_send(jiangxing_No_1.uart_mode, 5);
			printf("%d \n",uart_mode);
			// read_all_par();
			// printf("%d,%d,%d,%d \n",SWITCH1_CURRENTSTATE,SWITCH2_CURRENTSTATE,SWITCH3_CURRENTSTATE,SWITCH4_CURRENTSTATE);
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}


