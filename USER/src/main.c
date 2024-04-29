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
		// �˴���д�û����� ���������ʼ�������
		EAXFR=1;//ʹ�ܷ���XFR
		clock_init(SYSTEM_CLOCK_52M);	// ��ʼ��ϵͳƵ��,��ɾ���˾���롣
		board_init();					// ��ʼ���Ĵ���,��ɾ���˾���롣
		DisableGlobalIRQ(); // 	�ر������ж�

		ctimer_count_init(SPEEDR_PLUSE);					// ��ʼ����ʱ��4��Ϊ�ⲿ����
		ctimer_count_init(SPEEDL_PLUSE);					// ��ʼ����ʱ��3��Ϊ�ⲿ����
		pit_timer_ms(TIM_1, 5);				// ʹ�ö�ʱ��4�������жϣ�ʱ��4msһ�Ρ��������

    LED_init();//led��ʼ��
		WORK_LED_on();//�����ƴ�
		Disablespiwork();// �ر�SPI��������


		spi_slave_init();							// ��ʼ��SPI(��Ϊ�ӻ�)		
		wireless_uart_init();				  // ����ת����ģ���ʼ��
	
		motor_init();
		switch_init();                //������ʼ��
		iap_init();			            	// ��ʼ��EEPROM
		EnableSPIIRQ();								// ʹ��SPI�ж�
		EnableGlobalIRQ();						// ʹ�����ж�


	  pid_all_init();//��ʼ��
		disenablemotor();//�رյ��
		

		while (MPU_Init())
		{
			printf("STC32 MPU6050 GYRO Init test!  \r\n");
		}
		MPU6050_DMP_Init();
		printf("STC32 MPU6050 DMP test!  \r\n");
	  delay_ms(20);//�ȴ���ʱ����ȡ��ʼ����ʱ��ı�����ֵ
	  get_switch();//��ȡ��ʼ��ʱ��Ĳ���״̬�������ж��Ƿ�Ҫ��ȡeeprom����ֵ
		
		if(SWITCH3_FLAG==1)
		{
			 car_init(&jiangxing_No_1);//��״̬��ʼ��
			 delay_ms(100);
       read_all_par();
			 delay_ms(100);
		}
		else
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
   
    Zero_Offset_set();  //������Ʈ
		Enablespiwork();   //��������
//		enablemotor();//ʹ�ܵ��
	 // �˴���д�û����� ���������ʼ�������


    while(1)	
    { 

			MPU_Acquire();			
			wireless_read(1);
			wireless_send(jiangxing_No_1.uart_mode, 5);
		  printf("%d,%d,%d,%d,%d ,%d,%f\n",SWITCH1_FLAG,SWITCH2_FLAG,SWITCH3_FLAG,SWITCH4_FLAG,jiangxing_No_1.uart_mode,uart_mode,Forward_PID.KP);
			printf("%f,%f,%f\n", y_gyro_offset, z_gyro_offset, x_gyro_offset);
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}


