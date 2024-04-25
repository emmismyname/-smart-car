
/*********************************************************************************************************************
 * @file       		main (Signal_Board)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32F12K
 * @date       		2024-03-04  Ver0.0.0
									2024-03-08  Ver0.1.1
									2024-03-XX  Ver0.1.2

 ********************************************************************************************************************/
#include "headfile.h"

/*
 *�����ں�Ƶ�ʵ��趨�����Բ鿴board.h�ļ�
 *��board_init��,�Ѿ���P54��������Ϊ��λ
 *�����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */
 
	
void main()
{
	clock_init(SYSTEM_CLOCK_52M);		// ��ʼ��ϵͳƵ��
	board_init();			 							// ��ʼ���Ĵ���
	
	allPins_init();									// ��ʼ������
	allADC_init();                  // ��ʼ��ADC
	pit_timer_ms(TIM_3,5);					// ��ʼ����ʱ��
	pit_timer_ms(TIM_0,5);					// ��ʼ����ʱ��
	wireless_uart_init();						// ��ʼ�����ߴ���
//	dl1a_init();										// ��ʼ��TOF
	spi_master_init();							// ��ʼ��SPI(��Ϊ����)
	
	EnableSPIIRQ();									// ʹ��SPI�ж�
	EnableGlobalIRQ();            	// �����ж�
				
	resetelec_flag = 1;							// �ϵ�ɨ���
	resetelec_Init();

  while(1)
	 {
			resetelec_Init();
		 wireless_send_SBtoPC();			// ���ߴ��ڷ�����������λ��
		 wireless_read(1);						// ��ȡָ��
		 spi_send_float(err);					// SPI����err������
//		 delay_ms(10);
//		 printf("%f",err);
   }
}

