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
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"
#include "allcodeinr.h"

/*********************************************��������*********************************************************/

/********************************��ʼ����������**********/
void initall_PID();			// PID�ṹ�����������
void arangement_init(); // ��ʼ���趨
void parameter_init(); // ��ʼ���趨
/***************************************************************************************************/

/********************************�˶�ģʽ��������**********/

void Hall_detection(); // �������
void TOF_detection();	 // ������
void left_infrared();	 // ��������
void right_infrared(); // �Һ������

/***************************************************************************************************/

/********************************�������͵�ŵ�ѹ����������**********/
// void motor_output();//���PWM���
// void brushes_out();//��ˢ������
void ADC_battery_quire(); // ��ȡ��صĵ�ѹ��ֵ
// void counte_quire();//��ñ������������
/***************************************************************************************************/

/********************************��ʾ����������**********/
void system_display(); // ���ݰ���������Ӧ����
/***************************************************************************************************/

/********************************��к���**********/
void Element_Idef(); // Ԫ��ʶ��
/**************************************************/

/********************************PID��غ���**********/
void PID_all_set();
// void pid_servo(void);	  // pid���
/****************************************************/
void start_dection(); // �������

/*
 * ϵͳƵ�ʣ��ɲ鿴board.h�е� FOSC �궨���޸ġ�
 * board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�����ϵͳƵ��Ϊ33.1776MHZ
 * ��board_init��,�Ѿ���P54��������Ϊ��λ
 * �����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */

void main()
{
	arangement_init(); // ��ʼ���趨
	parameter_init();//������ʼ��
	// gyro_flag=0;
	// motor_flag=0;
	// outward_basin=1;
	// trace_flag=1;
	// OUTP_flag=0;
	while (1)
	{
		if (gyro_flag)
		{
			MPU_Acquire();
		}
		// wireless_read(1);						 // ����ת�������
		system_display();						 // ��ʾ����
		Button_Scan();							 // ��ťɨ��
		start_dection();						 // �������
		resetelec_Init();						 // ��������Сֵ���
		wireless_send(uart_mode, 5); // ����ת�������
	}
}

void start_dection() // �������
{
	if (start_flag == 1)
	{

		// ��ʼ������//��ˢ����
		// PID�ṹ������
		brusheless_flag = 1;
		brusheless_duty = 800;
		delay_ms(1500);
		HRTRIG_flag = 1; // ͣ��
		stop_flag = 0;	 // ͣ������
		OUTP_flag = 0;	 // ��������
		// outward_basin = 0; // ��ʼ������
		initall_PID();
		// ��ʼ����ȡ������Ư
		motor_flag = 0;
		gyro_flag = 1;
		yaw_output = 0.0f;
		pitch = 4.0f;
		// Ԫ�س�ʼ��/////////////
		roundcount = 0;
		leftRound = 0;
		rightRound = 0;
		UpSlope = 0;
		distance_L = 0;
		distance_R = 0;
		isr_counter_start = isr_count_flag; // ����������
		block_timer_count = 0;							// ���ϼ�������
		blockcount_flag = 0;								// ���ϱ�־λ
		block_count = 0;										// ��������
		/////////////////////////////////////////
		// ��ʾ�����عر�
		button_switch = 0;
		display_switch = 0;
		OUTP_flag = 0;		 // ��������
		outward_basin = 1; // ����
		// �����ǳ�ʼ��
		motor_flag = 1;
		// outward_basin = 0; // ��ʼ������
		trace_flag = 1;
		Element_flag = 0; // ��Ԫ��
		// OUTP_flag = 1;		// ���ó���
		TOF_object = 0;						 // ���ϸ���
		avoid_move = 0;						 // ���Ϲ��̱�־λ
		Target_speed = base_speed; // �ٶȿ���
		//////////////////////
		start_flag = 0; // �����־λ
										/////////////////////
	}
	// {										// ��ʼ������//��ˢ����
	// 	Target_speed = 0; // �ٶȿ���
	// 	motor_flag = 0;
	// 	trace_flag = 1;
	// 	// �����ǳ�ʼ��
	// 	gyro_flag = 0;
	// 	yaw_output = 0; // ��ʼ��
	// 	pitch_result = 0.0f;
	// 	gyro_flag = 1;
	// 	// PID�ṹ������
	// 	initall_PID();
	// 	// ��ʼ����ȡ������Ư
	// 	// OUTP_flag=0;//��������
	// 	Target_speed = 50; // �ٶȿ���
	// 	motor_flag = 1;
	// 	start_flag = 0;
	// 	running_mode = 8;
	// }
}

void arangement_init() // ��ʼ���趨
{
	board_init(); // ��ʼ���Ĵ���,��ɾ���˾���롣
	// �˴���д�û�����(���磺�����ʼ�������)
	iap_init();					// ��ʼ��EEPROM
	ips114_init();			// ��ʼ��1.1.4��ips��Ļ
	DisableGlobalIRQ(); // �ر����ж�
	sys_clk = 35000000; // ����ϵͳƵ��Ϊ35MHz
	WTST = 0;						// ���ó������ȴ���������ֵΪ0�ɽ�CPUִ�г�����ٶ�����Ϊ���
	// sys_clk��ѡֵ:30000000, 27000000. 24000000, 22118400, 20000000, 18432000, 12000000, 11059200, 6000000, 5529600��
	// ����ϵͳƵ�ʣ���Ƶ����Ҫ��STC-ISP����е� <�����û��������е�IRCƵ��>ѡ���Ƶ��һ�¡�
	// ���Ƶ�����ò��ԣ����ᵼ�´��ڵ����ݲ�����,PWM�Ĺ����������ȵȡ�
	board_init(); // ��������Ƶ���ٴγ�ʼ���ڲ��Ĵ���
	delay_init();
	ips114_init();														// ��ʼ��1.1.4��ips��Ļ
	wireless_uart_init();											// ����ת����ģ���ʼ��
	ctimer_count_init(SPEEDL_PLUSE);					// ��ʼ����ʱ��0��Ϊ�ⲿ����
	ctimer_count_init(SPEEDR_PLUSE);					// ��ʼ����ʱ��3��Ϊ�ⲿ����
	gpio_mode(KEY1_PIN, GPI_IMPEDANCE);				// S1��������ģʽ: �������
	gpio_mode(KEY2_PIN, GPI_IMPEDANCE);				// S2��������ģʽ: �������
	gpio_mode(KEY3_PIN, GPI_IMPEDANCE);				// S3��������ģʽ: �������
	gpio_mode(KEY4_PIN, GPI_IMPEDANCE);				// S4��������ģʽ: �������
	gpio_mode(HRTRIG, GPI_IMPEDANCE);					// S4��������ģʽ: �������
	gpio_mode(P6_7, GPO_PP);									// ��P6.7����������Ϊ�������
	BEEP = 0;																	// �������ر�
	pit_timer_ms(TIM_4, 5);										// ʹ�ö�ʱ��4�������жϣ�ʱ��5msһ�Ρ��������
	EnableGlobalIRQ();												// �������ж�
	adc_init(BAT_VOL_PIN, ADC_SYSclk_DIV_32); // ��ʼ����ѹ����
	initall_PID();														// PID�ṹ������
	gpio_mode(P6_4, GPO_PP);									// P64��������Ϊ�������
	gpio_mode(P6_0, GPO_PP);									// P60��������Ϊ�������
	pwm_init(PWM_1, 17000, 0);								// ��ʼ��PWM1  ʹ��P60����  ��ʼ��Ƶ��Ϊ17Khz
	pwm_init(PWM_2, 17000, 0);								// ��ʼ��PWM2  ʹ��P62����  ��ʼ��Ƶ��Ϊ17Khz
	pwm_init(PWMB_CH4_P77, 50, 0);						// ��ʼ����ˢ���Ϊ 50hz
	pwm_init(PWMB_CH3_P33, 50, 0);						// (1-2ms/20ms * 10000)��10000��PWM����ռ�ձ�ʱ���ֵ�� 10000ΪPWM���ֵ
	adc_init(ADC_P00, 0);											// P00����		����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴zf_adc.h�ļ�	  adc[7].data_0 = adc_mean_filter(ADC_P00,8);		//�ɼ�������ADC_P16��ѹ������12λR1����������(L1)
	adc_init(ADC_P01, 0);											// P01����		����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴zf_adc.h�ļ�	  adc[6].data_0 = adc_mean_filter(ADC_P01,8);		//�ɼ�������ADC_P14��ѹ������12λR2����������(L2)
	adc_init(ADC_P05, 0);											// P05����		����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴zf_adc.h�ļ�	  adc[5].data_0 = adc_mean_filter(ADC_P06,8);		//�ɼ�������ADC_P06��ѹ������12λR3����������(L3)
	adc_init(ADC_P13, 0);											// P13����		����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴zf_adc.h�ļ�		adc[1].data_0 = adc_mean_filter(ADC_P13,8);		//�ɼ�������ADC_P17��ѹ������12λL1����������(L7)
	adc_init(ADC_P14, 0);											// P14����		����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴zf_adc.h�ļ�    adc[2].data_0 = adc_mean_filter(ADC_P14,8);		//�ɼ�������ADC_P13��ѹ������12λL2����������(L6)
	adc_init(ADC_P16, 0);											// P16����		����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴zf_adc.h�ļ�    adc[3].data_0 = adc_mean_filter(ADC_P16,8);		//�ɼ�������ADC_P01��ѹ������12λL3����������(L5)
	adc_init(ADC_P17, 0);											// P17����		����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴zf_adc.h�ļ�	  adc[4].data_0 = adc_mean_filter(ADC_P17,8);		//�ɼ�������ADC_P05��ѹ������12λM-����������(L4)
	adc_init(ADC_P06, 0);											// P06����		����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴zf_adc.h�ļ�
	adc_init(ADC_P10, 0);											// P10����
	adc_init(ADC_P11, 0);											// P11����
	// �ⲿ�ж�2��3��
	exit_init(INT2_P36, FALLING_EDGE);
	exit_init(INT3_P37, FALLING_EDGE);
	// EX2=1;
//tof initial
//	while (dl1a_init())
//	{
//		delay_ms(500);
//		printf("VL53L0X init try again.\r\n");
//	}

	while (MPU_Init())
	{
		printf("STC32 MPU6050 GYRO Init test!  \r\n");
	}
	MPU6050_DMP_Init();
	printf("STC32 MPU6050 DMP test!  \r\n");
	
	read_all_par();		 // ��ȡeeprom��ֵ
	// Zero_Offset_set(); // mpu6050 DMP�ϵ����ʱ��
}

void parameter_init()//������ʼ��
{
// ���ŵ�����ֵ
	adc[1].data_max = 2359;
	adc[2].data_max = 2424;
	adc[3].data_max = 3121;
	adc[4].data_max = 2399;
	adc[5].data_max = 2322;
	// ǰ�ŵ�����ֵ
	adc[6].data_max = 3070;
	adc[7].data_max = 1802;
	adc[8].data_max = 2506;
	adc[9].data_max = 1698;
	adc[10].data_max = 3194;
	// ���ŵ����Сֵ
	adc[1].data_min = 0;
	adc[2].data_min = 0;
	adc[3].data_min = 0;
	adc[4].data_min = 0;
	adc[5].data_min = 0;
	// ǰ�ŵ����Сֵ
	adc[6].data_min = 0;
	adc[7].data_min = 0;
	adc[8].data_min = 0;
	adc[9].data_min = 0;
	adc[10].data_min = 0;
		// ZeroOffset_init(); //yaw��ֵ��Ʈ
	//��ʾ��ҳ������
	sys_display[1] = 1;
	sys_display[2] = 0;
	sys_display[3] = 0;
	button_switch = 1;//��ť����
	display_switch = 1;
	// brusheless_duty = 0;��ˢ��������
	// brusheless_flag = 1;��ˢ���ȿ�����־λ
	gyro_flag = 1;//����������
	yaw_output = 0.0f;
	// PID�ṹ������
	initall_PID();
	// ��ʼ����ȡ������Ư
	Element_flag = 0; // ��Ԫ��
	motor_flag = 0;
	roundcount = 0;
	leftRound = 0;
	rightRound = 0;
	UpSlope = 0;

	distance_L = 0;//�������
	distance_R = 0;//�������

	block_count = 0;			 // ��������
	stop_flag = 0;				 // ͣ������
	isr_counter_start = 0; // ����������
	block_timer_count = 0; // ������������
	blockcount_flag = 0;	 // ������־λ
	// outward_basin = 1;		 // ����
	// OUTP_flag = 0;				 // ��������
	HRTRIG_flag = 1; // ͣ��
	//////////////////////
	outward_basin = 0; // ��ʼ������
	OUTP_flag = 1;		 // ���ó���

	start_flag = 0;		 // �����־λ
	base_speed = 220;	 // �ٶȿ���
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		�������
//  @param
//  @param
//  @return  ���޸�trace_flagѰ������
//-------------------------------------------------------------------------------------------------------------------
// void Hall_detection() // ���
// {
// 	// int32 hall_counter=0;
// 	if (HRTRIG == 0 && isr_count_flag > 400)
// 	{
// 		if (outward_basin == 1)
// 		{
// 			yaw = 0;
// 			outward_basin = 2; // ����
// 		}
// 		trace_flag = 0;
// 		running_mode = 0; // ����
// 		motor_flag = 0;
// 		pid_init(&Speed_PID_L);
// 		pid_init(&Speed_PID_R);
// 		// PID_all_set();
// 		motor_flag = 1; // ������
// 		distance_L = 0;
// 		distance_R = 0; // ����
// 		gyro_flag = 0;
// 		yaw = 0.0f;
// 		Dir_car = 0;
// 		Target_speed = 50;
// 		while ((distance_L < 17000) && (distance_R < 18000))
// 		{
// 			gyro_flag = 1;
// 			OUTP_flag = 7; // ���
// 			motor_flag = 1;
// 			imu660ra_get_gyro(); // ��ȡ����������
// 			wireless_send(1, 5);
// 			ips114_showstr(15, 2, "distance_L");
// 			ips114_showfloat(8 * 16, 2, distance_L, 5, 2);
// 			ips114_showstr(15, 3, "distance_R");
// 			ips114_showfloat(8 * 16, 3, distance_R, 5, 2);
// 			ips114_showstr(15, 4, "Tgt_Spd_L=");
// 			ips114_showfloat(8 * 16, 4, Target_Speed_L, 3, 3);
// 			ips114_showstr(15, 5, "Tgt_Spd_R=");
// 			ips114_showfloat(8 * 16, 5, Target_Speed_R, 3, 3);
// 			ips114_showstr(15, 6, "yaw");
// 			ips114_showfloat(8 * 16, 6, yaw, 4, 2);
// 		}
// 		motor_flag = 0;
// 		brusheless_flag = 0;
// 	}
// }
// void Hall_detection() // �Һ�
// {
// 	int32 hall_counter = 0;
// 	if (HRTRIG == 0 && count_flag > 400)
// 	{
// 		if (outward_basin == 1)
// 		{
// 			yaw = 0;
// 			outward_basin = 2; // ����
// 		}
// 		trace_flag = 0;
// 		running_mode = 0; // ����
// 		motor_flag = 0;
// 		pid_init(&Speed_PID_L);
// 		pid_init(&Speed_PID_R);
// 		// PID_all_set();
// 		motor_flag = 1; // ������
// 		distance_L = 0;
// 		distance_R = 0; // ����
// 		gyro_flag = 0;
// 		yaw = 0.0f;
// 		Dir_car = 0;
// 		Target_speed = 50;
// 		while ((distance_L < 21000) && (distance_R < 20000))
// 		{
// 			gyro_flag = 1;
// 			OUTP_flag = 8; // �Һ�
// 			motor_flag = 1;
// 			imu660ra_get_gyro(); // ��ȡ����������
// 			wireless_send(1, 5);
// 			ips114_showstr(15, 2, "distance_L");
// 			ips114_showfloat(8 * 16, 2, distance_L, 5, 2);
// 			ips114_showstr(15, 3, "distance_R");
// 			ips114_showfloat(8 * 16, 3, distance_R, 5, 2);
// 			ips114_showstr(15, 4, "Tgt_Spd_L=");
// 			ips114_showfloat(8 * 16, 4, Target_Speed_L, 3, 3);
// 			ips114_showstr(15, 5, "Tgt_Spd_R=");
// 			ips114_showfloat(8 * 16, 5, Target_Speed_R, 3, 3);
// 			ips114_showstr(15, 6, "yaw");
// 			ips114_showfloat(8 * 16, 6, yaw, 4, 2);
// 		}
// 		motor_flag = 0;
// 		brusheless_flag = 0;
// 	}
// }

//-------------------------------------------------------------------------------------------------------------------
// @brief		PID�ṹ����������
//
//-------------------------------------------------------------------------------------------------------------------
void initall_PID()
{
	pid_init(&Speed_PID_L);
	pid_init(&Speed_PID_R);
	pid_init(&Swerve_PID);
	pid_init(&Forward_PID);
	pid_init(&Pout_PID);
	pid_init(&Pout_rate_PID);
	pid_init(&roundaboutL_PID);
	pid_init(&roundaboutR_PID);
	pid_init(&climb_PID);
	// pid_init(&fuzzy_PID);
	err = 0;
	err_last = 0;
	err_angle_rate = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ť��ȡ
//  @param      ����״̬
//  @param      ����״̬
//  @return     PID�������� ��ʾ����
//-------------------------------------------------------------------------------------------------------------------
void Button_Scan()
{
	// ��ȡ���뿪��״̬
	sw1_status = SW1_PIN;
	sw2_status = SW2_PIN;

	// ʹ�ô˷����ŵ����ڣ�����Ҫʹ��while(1) �ȴ������⴦������Դ�˷�
	// ���水��״̬
	key1_last_status = key1_status;
	key2_last_status = key2_status;
	key3_last_status = key3_status;
	key4_last_status = key4_status;
	// ��ȡ��ǰ����״̬
	key1_status = KEY1_PIN;
	key2_status = KEY2_PIN;
	key3_status = KEY3_PIN;
	key4_status = KEY4_PIN;

	// ��⵽��������֮��  ���ſ���λ��־λ
	if (key1_status && !key1_last_status)
		key1_flag = 1;
	if (key2_status && !key2_last_status)
		key2_flag = 1;
	if (key3_status && !key3_last_status)
		key3_flag = 1;
	if (key4_status && !key4_last_status)
		key4_flag = 1;

	// ��־λ��λ֮�󣬿���ʹ�ñ�־λִ���Լ���Ҫ�����¼�
	if (key1_flag)
	{
		if (sw1_status == 1 && sw2_status == 1)
		{
			cursor = cursor - 1;
			ips114_clear(WHITE);											// ����
			ips114_showstr(0, cursor, (uint8 *)"->"); // ���λ��
		}
		///////////////ʹ�ð���֮��Ӧ�������־λ
		key1_flag = 0;													// ʹ�ð���֮��Ӧ�������־λ
		if (sw1_status == 0 && sw2_status == 0) // ˫���뿪����Ļ
		{
			ips114_clear(WHITE); // ����
			display_switch = 0;
			button_switch = 0;
		}
	}
	if (key2_flag)
	{
		if (sw1_status == 1 && sw2_status == 1)
		{
			cursor = cursor + 1;
			ips114_clear(WHITE);											// ����
			ips114_showstr(0, cursor, (uint8 *)"->"); // ���λ��
		}
		///////////////ʹ�ð���֮��Ӧ�������־λ
		key2_flag = 0;													// ʹ�ð���֮��Ӧ�������־λ
		if (sw1_status == 0 && sw2_status == 0) // ˫���뿪����Ļ
		{
			display_switch = 1;
			button_switch = 1;
		}
	}

	if (button_switch == 1) // debug�򿪰�������
	{
		if (key3_flag)
		{
			if (sw1_status == 1 && sw2_status == 1) // ������
			{
				if (sys_display[1] == 1)
				{
					sys_display[1] = cursor + 2;
				}
				else if (sys_display[1] != 1)
				{
					sys_display[2] = cursor + 1;
				}
				ips114_clear(WHITE);
				cursor = 0;
				ips114_showstr(0, cursor, (uint8 *)"->"); // ���λ��
			}

			else if ((sw2_status == 0 || sw1_status == 0) && sys_display[1] == 2) // ����򿪵��ռ�ر����
			{
				switch (cursor)
				{
				case 0:
					start_flag = 1;
					break;
				case 1:
					resetelec_flag = 0;
					break;
				case 2:
					PID_all_set(); // ����pid
					break;
				case 3:
				{
					uart_mode = uart_mode + 1;
					if (uart_mode > 10)
						uart_mode = 10;
					else if (uart_mode < 0)
						uart_mode = 0;
				}
				break;
				case 4:
				{
					brusheless_flag = 1;
					brusheless_duty = 630;
					delay_ms(1000);
					trace_flag = 1;
					initall_PID();
				}

				break;
				case 5:
				{
					strategy_mode = strategy_mode + 1;
					if (strategy_mode > 10)
						strategy_mode = 10;
					else if (strategy_mode < 0)
						strategy_mode = 0;
					strategy_choose(strategy_mode);
				}
				break;
				case 6:
				{
					pack_mode = pack_mode + 1;
					if (pack_mode > 10)
						pack_mode = 10;
					else if (pack_mode < 0)
						pack_mode = 0;
					pack_choose(pack_mode);
				}
				break;
				default:
					printf("error\n");
				}
			}
			// pid����
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 1) // PID1����1����Ϊ1 +multiple_time
			{
				switch (cursor)
				{
				case 1:
					Speed_PID_L.KP = Speed_PID_L.KP + multiple_time;
					break;
				case 2:
					Speed_PID_L.KI = Speed_PID_L.KI + multiple_time;
					break;
				case 3:
					Speed_PID_L.KD = Speed_PID_L.KD + multiple_time;
					break;
				case 4:
					Speed_PID_R.KP = Speed_PID_R.KP + multiple_time;
					break;
				case 5:
					Speed_PID_R.KI = Speed_PID_R.KI + multiple_time;
					break;
				case 6:
					Speed_PID_R.KD = Speed_PID_R.KD + multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 1) // PID1����1 ����Ϊû�� ����multiple_time*10
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 10;
					break;
				case 2:
					multiple_time = multiple_time * 10;
					break;
				case 3:
					multiple_time = multiple_time * 10;
					break;
				case 4:
					multiple_time = multiple_time * 10;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				case 6:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 2) // PID2ֱ�����������2����Ϊ1 +multiple_time
			{
				switch (cursor)
				{
				case 1:
					Forward_PID.KP = Forward_PID.KP + multiple_time;
					break;
				case 2:
					Forward_PID.KI = Forward_PID.KI + multiple_time;
					break;
				case 3:
					Forward_PID.KD = Forward_PID.KD + multiple_time;
					break;
				case 4:
					Swerve_PID.KP = Swerve_PID.KP + multiple_time;
					break;
				case 5:
					Swerve_PID.KI = Swerve_PID.KI + multiple_time;
					break;
				case 6:
					Swerve_PID.KD = Swerve_PID.KD + multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 2) // PID2ֱ�����������1��2 ����Ϊû�� ����multiple_time*10
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 10;
					break;
				case 2:
					multiple_time = multiple_time * 10;
					break;
				case 3:
					multiple_time = multiple_time * 10;
					break;
				case 4:
					multiple_time = multiple_time * 10;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				case 6:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 3) // PID����3 ����Ϊû�� ����multiple_time*10
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 10;
					break;
				case 2:
					multiple_time = multiple_time * 10;
					break;
				case 3:
					multiple_time = multiple_time * 10;
					break;
				case 4:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 3) // PID����3	����Ϊ1 +multiple_time
			{
				switch (cursor)
				{
				case 1:
					Pout_PID.KP = Pout_PID.KP + multiple_time;
					break;
				case 2:
					Pout_PID.KI = Pout_PID.KI + multiple_time;
					break;
				case 3:
					Pout_PID.KD = Pout_PID.KD + multiple_time;
					break;
				case 4:
					gyro_d += multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 4) // PID4 ����4����Ϊ1 +multiple_time
			{
				switch (cursor)
				{
				case 1:
					roundaboutL_PID.KP = roundaboutL_PID.KP + multiple_time;
					break;
				case 2:
					roundaboutL_PID.KI = roundaboutL_PID.KI + multiple_time;
					break;
				case 3:
					roundaboutL_PID.KD = roundaboutL_PID.KD + multiple_time;
					break;
				case 4:
					roundaboutR_PID.KP = roundaboutR_PID.KP + multiple_time;
					break;
				case 5:
					roundaboutR_PID.KI = roundaboutR_PID.KI + multiple_time;
					break;
				case 6:
					roundaboutR_PID.KD = roundaboutR_PID.KD + multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status ==  1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 4) // PID2ֱ�����������1��2 ����Ϊû�� ����multiple_time*10
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 10;
					break;
				case 2:
					multiple_time = multiple_time * 10;
					break;
				case 3:
					multiple_time = multiple_time * 10;
					break;
				case 4:
					multiple_time = multiple_time * 10;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				case 6:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 5) // PID4 ����4����Ϊ1 +multiple_time
			{
				switch (cursor)
				{
				case 1:
					Pout_rate_PID.KP = Pout_rate_PID.KP + multiple_time;
					break;
				case 2:
					Pout_rate_PID.KI = Pout_rate_PID.KI + multiple_time;
					break;
				case 3:
					Pout_rate_PID.KD = Pout_rate_PID.KD + multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 5) // PID2ֱ�����������1��2 ����Ϊû�� ����multiple_time*10
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 10;
					break;
				case 2:
					multiple_time = multiple_time * 10;
					break;
				case 3:
					multiple_time = multiple_time * 10;
					break;
				case 4:
					multiple_time = multiple_time * 10;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				case 6:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 5) // ��� +multiple_time
			{
				switch (cursor)
				{
				case 0:
					distance_break = distance_break + multiple_time;
					break;
				case 1:
					intop_angle = intop_angle + multiple_time;
					break;
				case 2:
					intop_distance = intop_distance + multiple_time;
					break;
				case 3:
					intop_radiu = intop_radiu + multiple_time;
					break;
				case 4:
					outp_angle = outp_angle + multiple_time;
					break;
				case 5:
					outp_distance = outp_distance + multiple_time;
					break;
				case 6:
					outp_radiu = outp_radiu + multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 5) // ��Ⲧ��Ϊû�� ����multiple_time*10
			{
				switch (cursor)
				{
				case 0:
					multiple_time = multiple_time * 10;
					break;
				case 1:
					multiple_time = multiple_time * 10;
					break;
				case 2:
					multiple_time = multiple_time * 10;
					break;
				case 3:
					multiple_time = multiple_time * 10;
					break;
				case 4:
					multiple_time = multiple_time * 10;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				case 6:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 6) // ��ˢ���ٲ���Ϊ1 +100
			{
				switch (cursor)
				{
				case 1:
					Round_mode++;
					if (Round_mode > 9)
					{
						Round_mode = 0;
					}
					break;
				case 3:
					brusheless_flag = 1;
					brusheless_duty = 630;
					break;
				case 4:
					brusheless_duty = brusheless_duty + 100;
					break;
				case 5:
					base_speed = base_speed + 100;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 6) // ��ˢ���ٲ���Ϊ2 +10
			{
				switch (cursor)
				{
				case 1:
					Round_mode++;
					if (Round_mode > 9)
					{
						Round_mode = 0;
					}
					break;
				case 3:
					brusheless_flag = 1;
					brusheless_duty = 630;
					break;
				case 4:
					brusheless_duty = brusheless_duty + 10;
					break;
				case 5:
					base_speed = base_speed + 10;
					break;
					;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 1) // ����
			{
				switch (cursor)
				{
				case 0:
					leftround_diff += multiple_time;
					break;
				case 1:
					leftround_ready += multiple_time;
					break;
				case 2:
					leftround_yawtarget += multiple_time;
					break;
				case 3:
					pretoreadyL_round_distance += multiple_time;
					break;
				case 4:
					lmid_del += multiple_time;
					break;
				case 5:
					r2 += multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 1) // ����
			{
				switch (cursor)
				{
				case 0:
					multiple_time = multiple_time * 10;
					break;
				case 1:
					multiple_time = multiple_time * 10;
					break;
				case 2:
					multiple_time = multiple_time * 10;
					break;
				case 3:
					multiple_time = multiple_time * 10;
					break;
				case 4:
					multiple_time = multiple_time * 10;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 2) // ����
			{
				switch (cursor)
				{
				case 0:
					rigtround_diff += multiple_time;
					break;
				case 1:
					rigtround_ready += multiple_time;
					break;
				case 2:
					rigtround_yawtarget += multiple_time;
					break;
				case 3:
					pretoreadyR_round_distance += multiple_time;
					break;
				case 4:
					rmid_del += multiple_time;
					break;
				case 5:
					l2 += multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 2) // ����
			{
				switch (cursor)
				{
				case 0:
					multiple_time = multiple_time * 10;
					break;
				case 1:
					multiple_time = multiple_time * 10;
					break;
				case 2:
					multiple_time = multiple_time * 10;
					break;
				case 3:
					multiple_time = multiple_time * 10;
					break;
				case 4:
					multiple_time = multiple_time * 10;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 4) // �����趨��ֵ�ٲ���Ϊ1 +5
			{
				switch (cursor)
				{
				case 4:
					Target_Speed_L = Target_Speed_L + 50;
					break;
				case 5:
					Target_Speed_R = Target_Speed_R + 50;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 4) // �����趨��ֵ2����Ϊ2 +1
			{
				switch (cursor)
				{
				case 4:
					Target_Speed_L = Target_Speed_L + 10;
					break;
				case 5:
					Target_Speed_R = Target_Speed_R + 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 8) // ����
			{
				switch (cursor)
				{
				case 1:
					out_angle += multiple_time;
					break;
				case 2:
					out_distance += multiple_time;
					break;
				case 3:
					in_angle += multiple_time;
					break;
				case 4:
					in_distance += multiple_time;
					break;

				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 8) // ����
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 10;
					break;
				case 2:
					multiple_time = multiple_time * 10;
					break;
				case 3:
					multiple_time = multiple_time * 10;
					break;
				case 4:
					multiple_time = multiple_time * 10;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				case 6:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 9) // ����
			{
				switch (cursor)
				{
				case 1:
					pre_distance += multiple_time;
					break;
				case 2:
					ready_distance += multiple_time;
					break;

				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 9) // ����
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 10;
					break;
				case 2:
					multiple_time = multiple_time * 10;
					break;
				case 3:
					multiple_time = multiple_time * 10;
					break;
				case 4:
					multiple_time = multiple_time * 10;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				case 6:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			///////////////ʹ�ð���֮��Ӧ�������־λ
			write_all_par();
			key3_flag = 0; // ʹ�ð���֮��Ӧ�������־λ
		}
		if (key4_flag)
		{
			if (sw1_status == 1 && sw2_status == 1) // ������
			{
				if (sys_display[1] != 1 && sys_display[2] != 0) // �����˵���λ
				{
					sys_display[2] = 0;
				}
				else if (sys_display[1] != 1 && sys_display[2] == 0) // һ���˵���λ
				{
					sys_display[1] = 1;
				}

				cursor = 0;
				ips114_clear(WHITE);
				ips114_showstr(0, cursor, (uint8 *)"->"); // ���λ��
			}
			else if ((sw2_status == 0 || sw1_status == 0) && sys_display[1] == 2) // ����رյ��ռ�ر����
			{
				switch (cursor)
				{
				case 0:
					start_flag = 1;
					break;
				case 1:
					resetelec_flag = 1;
					break;
				case 2:
					PID_all_set(); // ����pid
					break;
				case 3:
				{
					uart_mode = uart_mode - 1;
					if (uart_mode > 10)
						uart_mode = 10;
					else if (uart_mode < 0)
						uart_mode = 0;
				}
				break;
				case 4:
					trace_flag = 0;
					break;
				case 5:
				{
					strategy_mode = strategy_mode - 1;
					if (strategy_mode > 10)
						strategy_mode = 10;
					else if (strategy_mode < 0)
						strategy_mode = 0;
					strategy_choose(strategy_mode);
				}
				break;
				case 6:
				{
					pack_mode = pack_mode - 1;
					if (pack_mode > 10)
						pack_mode = 10;
					else if (pack_mode < 0)
						pack_mode = 0;
					pack_choose(pack_mode);
				}
				break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 1) // PID����1 ����Ϊ1 -multiple_time
			{
				switch (cursor)
				{
				case 1:
					Speed_PID_L.KP = Speed_PID_L.KP - multiple_time;
					break;
				case 2:
					Speed_PID_L.KI = Speed_PID_L.KI - multiple_time;
					break;
				case 3:
					Speed_PID_L.KD = Speed_PID_L.KD - multiple_time;
					break;
				case 4:
					Speed_PID_R.KP = Speed_PID_R.KP - multiple_time;
					break;
				case 5:
					Speed_PID_R.KI = Speed_PID_R.KI - multiple_time;
					break;
				case 6:
					Speed_PID_R.KD = Speed_PID_R.KD - multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 1) // PID����1 ����Ϊû�� ����multiple_time*0.1
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 0.1;
					break;
				case 2:
					multiple_time = multiple_time * 0.1;
					break;
				case 3:
					multiple_time = multiple_time * 0.1;
					break;
				case 4:
					multiple_time = multiple_time * 0.1;
					break;
				case 5:
					multiple_time = multiple_time * 0.1;
					break;
				case 6:
					multiple_time = multiple_time * 0.1;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 2) // PID����2����Ϊ1 -multiple_time
			{
				switch (cursor)
				{
				case 1:
					Forward_PID.KP = Forward_PID.KP - multiple_time;
					break;
				case 2:
					Forward_PID.KI = Forward_PID.KI - multiple_time;
					break;
				case 3:
					Forward_PID.KD = Forward_PID.KD - multiple_time;
					break;
				case 4:
					Swerve_PID.KP = Swerve_PID.KP - multiple_time;
					break;
				case 5:
					Swerve_PID.KI = Swerve_PID.KI - multiple_time;
					break;
				case 6:
					Swerve_PID.KD = Swerve_PID.KD - multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 2) // PID����2 ����Ϊû�� ����multiple_time*0.1
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 0.1;
					break;
				case 2:
					multiple_time = multiple_time * 0.1;
					break;
				case 3:
					multiple_time = multiple_time * 0.1;
					break;
				case 4:
					multiple_time = multiple_time * 0.1;
					break;
				case 5:
					multiple_time = multiple_time * 0.1;
					break;
				case 6:
					multiple_time = multiple_time * 0.1;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 3) // PID����3 ����Ϊû�� ����multiple_time*10
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 0.1;
					break;
				case 2:
					multiple_time = multiple_time * 0.1;
					break;
				case 3:
					multiple_time = multiple_time * 0.1;
					break;
				case 4:
					multiple_time = multiple_time * 0.1;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 3) // PID����3����Ϊ1 +multiple_time
			{
				switch (cursor)
				{
				case 1:
					Pout_PID.KP = Pout_PID.KP - multiple_time;
					break;
				case 2:
					Pout_PID.KI = Pout_PID.KI - multiple_time;
					break;
				case 3:
					Pout_PID.KD = Pout_PID.KD - multiple_time;
					break;
				case 4:
					gyro_d -= multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 4) // PID4 ����4����Ϊ1 +multiple_time
			{
				switch (cursor)
				{
				case 1:
					roundaboutL_PID.KP = roundaboutL_PID.KP - multiple_time;
					break;
				case 2:
					roundaboutL_PID.KI = roundaboutL_PID.KI - multiple_time;
					break;
				case 3:
					roundaboutL_PID.KD = roundaboutL_PID.KD - multiple_time;
					break;
				case 4:
					roundaboutR_PID.KP = roundaboutR_PID.KP - multiple_time;
					break;
				case 5:
					roundaboutR_PID.KI = roundaboutR_PID.KI - multiple_time;
					break;
				case 6:
					roundaboutR_PID.KD = roundaboutR_PID.KD - multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 4) // PID2ֱ�����������1��2 ����Ϊû�� ����multiple_time*10
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 0.1;
					break;
				case 2:
					multiple_time = multiple_time * 0.1;
					break;
				case 3:
					multiple_time = multiple_time * 0.1;
					break;
				case 4:
					multiple_time = multiple_time * 0.1;
					break;
				case 5:
					multiple_time = multiple_time * 0.1;
					break;
				case 6:
					multiple_time = multiple_time * 0.1;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 4) // PID4 ����4����Ϊ1 +multiple_time
			{
				switch (cursor)
				{
				case 1:
					Pout_rate_PID.KP = Pout_rate_PID.KP - multiple_time;
					break;
				case 2:
					Pout_rate_PID.KI = Pout_rate_PID.KI - multiple_time;
					break;
				case 3:
					Pout_rate_PID.KD = Pout_rate_PID.KD - multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 4) // PID2ֱ�����������1��2 ����Ϊû�� ����multiple_time*10
			{
				switch (cursor)
				{
				case 0:
					multiple_time = multiple_time * 0.1;
					break;
				case 1:
					multiple_time = multiple_time * 0.1;
					break;
				case 2:
					multiple_time = multiple_time * 0.1;
					break;
				case 3:
					multiple_time = multiple_time * 0.1;
					break;
				case 4:
					multiple_time = multiple_time * 0.1;
					break;
				case 5:
					multiple_time = multiple_time * 0.1;
					break;
				case 6:
					multiple_time = multiple_time * 0.1;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 5) // ��� +multiple_time
			{
				switch (cursor)
				{
				case 0:
					distance_break = distance_break - multiple_time;
					break;
				case 1:
					intop_angle = intop_angle - multiple_time;
					break;
				case 2:
					intop_distance = intop_distance - multiple_time;
					break;
				case 3:
					intop_radiu = intop_radiu - multiple_time;
					break;
				case 4:
					outp_angle = outp_angle - multiple_time;
					break;
				case 5:
					outp_distance = outp_distance - multiple_time;
					break;
				case 6:
					outp_radiu = outp_radiu - multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 5) // PID2ֱ�����������1��2 ����Ϊû�� ����multiple_time*10
			{
				switch (cursor)
				{
				case 0:
					multiple_time = multiple_time * 0.1;
					break;
				case 1:
					multiple_time = multiple_time * 0.1;
					break;
				case 2:
					multiple_time = multiple_time * 0.1;
					break;
				case 3:
					multiple_time = multiple_time * 0.1;
					break;
				case 4:
					multiple_time = multiple_time * 0.1;
					break;
				case 5:
					multiple_time = multiple_time * 0.1;
					break;
				case 6:
					multiple_time = multiple_time * 0.1;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 6) // ��ˢ���ٲ���Ϊ1 -100
			{
				// ��ˢ���ٲ���Ϊ2 +10

				switch (cursor)
				{
				case 1:
					Round_mode--;
					if (Round_mode > 9)
					{
						Round_mode = 0;
					}
					break;
				case 3:
					brusheless_flag = 0;
					brusheless_duty = 500;
					break;
				case 4:
					brusheless_duty = brusheless_duty - 100;
					break;
				case 5:
					base_speed = base_speed - 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 6) // ��ˢ���ٲ���Ϊ2 +10
			{
				switch (cursor)
				{
				case 1:
					Round_mode--;
					if (Round_mode > 9)
					{
						Round_mode = 0;
					}
					break;
				case 3:
					brusheless_flag = 0;
					brusheless_duty = 500;
					break;
				case 4:
					brusheless_duty = brusheless_duty - 10;
					break;
				case 5:
					base_speed = base_speed - 100;
					break;
					;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 1) // ����
			{
				switch (cursor)
				{
				case 0:
					leftround_diff -= multiple_time;
					break;
				case 1:
					leftround_ready -= multiple_time;
					break;
				case 2:
					leftround_yawtarget -= multiple_time;
					break;
				case 3:
					pretoreadyL_round_distance -= multiple_time;
					break;
				case 4:
					lmid_del -= multiple_time;
					break;
				case 5:
					r2 -= multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 1) // ����
			{
				switch (cursor)
				{
				case 0:
					multiple_time = multiple_time * 0.1;
					break;
				case 1:
					multiple_time = multiple_time * 0.1;
					break;
				case 2:
					multiple_time = multiple_time * 0.1;
					break;
				case 3:
					multiple_time = multiple_time * 0.1;
					break;
				case 4:
					multiple_time = multiple_time * 0.1;
					break;
				case 5:
					multiple_time = multiple_time * 0.1;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 2) // ����
			{
				switch (cursor)
				{
				case 0:
					rigtround_diff -= multiple_time;
					break;
				case 1:
					rigtround_ready -= multiple_time;
					break;
				case 2:
					rigtround_yawtarget -= multiple_time;
					break;
				case 3:
					pretoreadyR_round_distance -= multiple_time;
					break;
				case 4:
					rmid_del -= multiple_time;
					break;
				case 5:
					l2 -= multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 2) // ����
			{
				switch (cursor)
				{
				case 0:
					multiple_time = multiple_time * 0.1;
					break;
				case 1:
					multiple_time = multiple_time * 0.1;
					break;
				case 2:
					multiple_time = multiple_time * 0.1;
					break;
				case 3:
					multiple_time = multiple_time * 0.1;
					break;
				case 4:
					multiple_time = multiple_time * 0.1;
					break;
				case 5:
					multiple_time = multiple_time * 0.1;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 4) // �����趨��ֵ�ٲ���Ϊ1 +5
			{
				switch (cursor)
				{
				case 4:
					Target_Speed_L = Target_Speed_L - 50;
					break;
				case 5:
					Target_Speed_R = Target_Speed_R - 50;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 4) // �����趨��ֵ2����Ϊ2 +1
			{
				switch (cursor)
				{
				case 4:
					Target_Speed_L = Target_Speed_L - 10;
					break;
				case 5:
					Target_Speed_R = Target_Speed_R - 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 8) // ����
			{
				switch (cursor)
				{
				case 1:
					out_angle -= multiple_time;
					break;
				case 2:
					out_distance -= multiple_time;
					break;
				case 3:
					in_angle -= multiple_time;
					break;
				case 4:
					in_distance -= multiple_time;
					break;

				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 8) // ����
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 0.1;
					break;
				case 2:
					multiple_time = multiple_time * 0.1;
					break;
				case 3:
					multiple_time = multiple_time * 0.1;
					break;
				case 4:
					multiple_time = multiple_time * 0.1;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				case 6:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 9) // ����
			{
				switch (cursor)
				{
				case 1:
					pre_distance -= multiple_time;
					break;
				case 2:
					ready_distance -= multiple_time;
					break;
				default:
					printf("error\n");
				}
			}
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 9) // ����
			{
				switch (cursor)
				{
				case 1:
					multiple_time = multiple_time * 0.1;
					break;
				case 2:
					multiple_time = multiple_time * 0.1;
					break;
				case 3:
					multiple_time = multiple_time * 0.1;
					break;
				case 4:
					multiple_time = multiple_time * 0.1;
					break;
				case 5:
					multiple_time = multiple_time * 10;
					break;
				case 6:
					multiple_time = multiple_time * 10;
					break;
				default:
					printf("error\n");
				}
			}

			write_all_par();
			key4_flag = 0; // ʹ�ð���֮��Ӧ�������־λ
		}
	}


}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʾ����ϵͳ����ʾ����
//  @param      display_switch 1�򿪽��� 0�رս���
//  @param      ��ʾ����0���˵� ����1PID1���� ����2PID2����  ����3�ٶ���ʾģ��  ����4ģʽѡ��ģ�� ����5��ˢ���ģ��� ����6���ADCģ����� ����7���ģ�� ����8��PID3
//  @return     ��ʾ����
//-------------------------------------------------------------------------------------------------------------------
void system_display() // ��ʾ����ϵͳ����ʾ����
{

	if (display_switch) // ������Ļ
	{
		switch (sys_display[1])
		{
		case 1: // ���˵���ʾ
		{
			ips114_showstr(15, 0, (uint8 *)"mode-choose");								// PID��������
			ips114_showstr(15, 1, (uint8 *)"arguments-pid");							// PID��������
			ips114_showstr(15, 2, (uint8 *)"round");											// ����ģ���������
			ips114_showstr(15, 3, (uint8 *)"pack");												// �������ٶȻ�����ѡ��ģ�����
			ips114_showstr(15, 4, (uint8 *)"imu_display");       					// ��̬��������ʾ����
			ips114_showstr(15, 5, (uint8 *)"electric ADC ");							// ��ʾADC��Ž���
			ips114_showstr(15, 6, (uint8 *)"obstacle_avoidance");					// ���ϲ���
			ips114_showstr(15, 7, (uint8 *)"dill");												// ��ʾADC��Ž���
			ips114_showstr(0, cursor, (uint8 *)"->");											// ���λ��
			ips114_showuint16(8 * 20, 7, battery_voltage);								// 1�Ǵ� 0�ǹر�
			ips114_showfloat(20 * 8, 1, dl1a_distance_mm, 5, 1);					// P������ʾ
		}
		break;
		case 2: // ģʽ����
		{
			ips114_showfloat(26 * 8, 1, dl1a_distance_mm, 5, 1); // P������ʾ
			ips114_showstr(15, 0, (uint8 *)"start_switch");			 // ������־λ
			ips114_showuint8(8 * 16, 0, start_flag);						 // 1�Ǵ� 0�ǹر�
			ips114_showstr(15, 1, (uint8 *)"resetelec_flag");		 // ����Ƿ��
			ips114_showuint8(8 * 16, 1, resetelec_flag);				 // 1�Ǵ� 0�ǹر�
			ips114_showstr(15, 2, (uint8 *)"PID_all_set");			 // �Ƿ����PID
			ips114_showstr(15, 3, (uint8 *)"uart=");
			ips114_showint16(9 * 16, 3, uart_mode);						 // ����ģʽ��ʾ
			ips114_showstr(15, 4, (uint8 *)"trace_flag");			 // Ѱ������
			ips114_showint8(8 * 16, 4, trace_flag);						 // 1�Ǵ� 0�ǹر�
			ips114_showstr(15, 5, (uint8 *)"stragy-choose");	 // ģʽѡ��ģ�����
			ips114_showint16(8 * 16, 5, strategy_mode);				 // ģʽѡ��
			ips114_showstr(15, 6, (uint8 *)"pack-choose");		 // ģʽѡ��ģ�����
			ips114_showint16(8 * 16, 6, pack_mode);						 // ͣ������ģʽѡ��
			ips114_showstr(15, 7, (uint8 *)"strategy-choose"); // ģʽѡ��ģ�����
			ips114_showint16(8 * 20, 7, strategy_mode);				 // ͣ������ģʽѡ��
		}
		break;
		case 3: // ���ν�����ʾ 1ת�� ֱ�л� �Ƕ� �� �һ�
		{
			switch (sys_display[2])
			{
			case 0:
			{
				ips114_showstr(15, 0, (uint8 *)"arguments-pid1_speed"); // PID��������
				ips114_showstr(15, 1, (uint8 *)"arguments-pid2_F&S");		// PID��������
				ips114_showstr(15, 2, (uint8 *)"arguments-pid3_P_out"); // PID��������
				ips114_showstr(15, 3, (uint8 *)"arguments-pid4_round"); // PID��������
				ips114_showstr(15, 4, (uint8 *)"arguments-pid4_rate");	// PID��������
			}
			break;
			case 1: // �����ٶ�
			{
				ips114_showstr(15, 0, (uint8 *)"arguments-pid1");		// PID��������
				ips114_showstr(15, 1, (uint8 *)"pidL-P");						// P
				ips114_showstr(15, 2, (uint8 *)"pidL-I");						// I
				ips114_showstr(15, 3, (uint8 *)"pidL-D");						// D
				ips114_showfloat(10 * 8, 1, Speed_PID_L.KP, 4, 5);	// P������ʾ
				ips114_showfloat(10 * 8, 2, Speed_PID_L.KI, 4, 5);	// P������ʾ
				ips114_showfloat(10 * 8, 3, Speed_PID_L.KD, 4, 5);	// P������ʾ
				ips114_showstr(15, 4, (uint8 *)"pidR-P");						// P
				ips114_showstr(15, 5, (uint8 *)"pidR-I");						// I
				ips114_showstr(15, 6, (uint8 *)"pidR-D");						// D
				ips114_showfloat(10 * 8, 4, Speed_PID_R.KP, 4, 5);	// P������ʾ
				ips114_showfloat(10 * 8, 5, Speed_PID_R.KI, 4, 5);	// P������ʾ
				ips114_showfloat(10 * 8, 6, Speed_PID_R.KD, 4, 5);	// P������ʾ
				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // ����λ��
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// ����λ����ʾ
			}
			break;
			case 2: // ֱ��ת��
			{
				ips114_showstr(15, 0, (uint8 *)"arguments-pid1");		// PID��������
				ips114_showstr(15, 1, (uint8 *)"Forward-P");				// P
				ips114_showstr(15, 2, (uint8 *)"Forward-I");				// I
				ips114_showstr(15, 3, (uint8 *)"Forward-D");				// D
				ips114_showfloat(14 * 8, 1, Forward_PID.KP, 4, 5);	// P������ʾ
				ips114_showfloat(14 * 8, 2, Forward_PID.KI, 4, 5);	// P������ʾ
				ips114_showfloat(14 * 8, 3, Forward_PID.KD, 4, 5);	// P������ʾ
				ips114_showstr(15, 4, (uint8 *)"Swerve-P");					// P
				ips114_showstr(15, 5, (uint8 *)"Swerve-I");					// I
				ips114_showstr(15, 6, (uint8 *)"Swerve-D");					// D
				ips114_showfloat(14 * 8, 4, Swerve_PID.KP, 4, 5);		// P������ʾ
				ips114_showfloat(14 * 8, 5, Swerve_PID.KI, 4, 5);		// P������ʾ
				ips114_showfloat(14 * 8, 6, Swerve_PID.KD, 4, 5);		// P������ʾ
				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // ����λ��
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// ����λ����ʾ
			}
			break;
			case 3: // �ǶȻ�
			{
				ips114_showstr(15, 1, "kp");
				ips114_showfloat(8 * 16, 1, Pout_PID.KP, 4, 4);
				ips114_showstr(15, 2, "ki");
				ips114_showfloat(8 * 16, 2, Pout_PID.KI, 4, 4);
				ips114_showstr(15, 3, "kd");
				ips114_showfloat(8 * 16, 3, Pout_PID.KD, 4, 4);
				ips114_showstr(15, 4, "gyro_d");
				ips114_showfloat(10 * 8, 4, gyro_d, 2, 4);
				ips114_showfloat(15, 5, Target_Speed_L, 3, 3);
				ips114_showfloat(8 * 16, 5, Target_Speed_R, 3, 3);
				ips114_showstr(15, 6, (uint8 *)"Targetspeed");			// ģʽѡ��ģ�����
				ips114_showint16(8 * 16, 6, Target_speed);					// ģʽѡ��
				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // ����λ��
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 3);		// ����λ����ʾ
			}
			break;
			case 4: // ����
			{
				ips114_showstr(15, 0, (uint8 *)"arguments-pid4");			 // PID��������
				ips114_showstr(15, 1, (uint8 *)"roundaboutL-P");			 // P
				ips114_showstr(15, 2, (uint8 *)"roundaboutL-I");			 // I
				ips114_showstr(15, 3, (uint8 *)"roundaboutL-D");			 // D
				ips114_showfloat(20 * 8, 1, roundaboutL_PID.KP, 4, 5); // P������ʾ
				ips114_showfloat(20 * 8, 2, roundaboutL_PID.KI, 4, 5); // P������ʾ
				ips114_showfloat(20 * 8, 3, roundaboutL_PID.KD, 4, 5); // P������ʾ
				ips114_showstr(15, 4, (uint8 *)"roundaboutR-P");			 // P
				ips114_showstr(15, 5, (uint8 *)"roundaboutR-I");			 // I
				ips114_showstr(15, 6, (uint8 *)"roundaboutR-D");			 // D
				ips114_showfloat(20 * 8, 4, roundaboutR_PID.KP, 4, 5); // P������ʾ
				ips114_showfloat(20 * 8, 5, roundaboutR_PID.KI, 4, 5); // P������ʾ
				ips114_showfloat(20 * 8, 6, roundaboutR_PID.KD, 4, 5); // P����s��ʾ
				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time");		 // ����λ��
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);			 // ����λ����ʾ
			}
			break;
			case 5: // �Ƕ��ٶȻ�
			{
				ips114_showstr(15, 1, "ratekp");
				ips114_showfloat(8 * 16, 1, Pout_rate_PID.KP, 4, 4);
				ips114_showstr(15, 2, "rateki");
				ips114_showfloat(8 * 16, 2, Pout_rate_PID.KI, 4, 4);
				ips114_showstr(15, 3, "ratekd");
				ips114_showfloat(8 * 16, 3, Pout_rate_PID.KD, 4, 4);
				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // ����λ��
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 3);		// ����λ����ʾ}
				break;
			default:
				break;
			}
				// ips114_showstr(15, 0, (uint8 *)"arguments-pid2");		// PID��������
				// ips114_showstr(15, 1, (uint8 *)"Forward-P");				// P
				// ips114_showstr(15, 2, (uint8 *)"Forward-I");				// I
				// ips114_showstr(15, 3, (uint8 *)"Forward-D");				// D
				// ips114_showfloat(14 * 8, 1, Forward_PID.KP, 4, 5);	// P������ʾ
				// ips114_showfloat(14 * 8, 2, Forward_PID.KI, 4, 5);	// P������ʾ
				// ips114_showfloat(14 * 8, 3, Forward_PID.KD, 4, 5);	// P������ʾ
				// ips114_showstr(15, 4, (uint8 *)"Swerve-P");					// P
				// ips114_showstr(15, 5, (uint8 *)"Swerve-I");					// I
				// ips114_showstr(15, 6, (uint8 *)"Swerve-D");					// D
				// ips114_showfloat(14 * 8, 4, Swerve_PID.KP, 4, 5);		// P������ʾ
				// ips114_showfloat(14 * 8, 5, Swerve_PID.KI, 4, 5);		// P������ʾ
				// ips114_showfloat(14 * 8, 6, Swerve_PID.KD, 4, 5);		// P������ʾ
				// ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // ����λ��
				// ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// ����λ����ʾ
			}
			break;
		}
		case 4: // ��������
		{
			switch (sys_display[2])
			{
			case 0:
			{
				ips114_showstr(15, 0, (uint8 *)"left-judge-round");
				ips114_showstr(15, 1, (uint8 *)"right-judge-round");
				ips114_showstr(15, 2, (uint8 *)"left-move-round");
				ips114_showstr(15, 3, (uint8 *)"right-move-round");
				ips114_showstr(15, 4, (uint8 *)"motor-speed");
			}
			break;
			case 1: // �󻷵�
			{
				ips114_showstr(15, 0, "lrd_diff =");
				ips114_showfloat(9 * 12, 0, leftround_diff, 4, 3);
				ips114_showstr(15, 1, "lrd_redy =");
				ips114_showfloat(9 * 12, 1, leftround_ready, 4, 3);
				ips114_showstr(15, 2, "left_yaw =");
				ips114_showfloat(9 * 12, 2, leftround_yawtarget, 4, 3);
				ips114_showstr(15, 3, "prea_l_dis =");
				ips114_showfloat(9 * 12, 3, pretoreadyL_round_distance, 4, 3);
				ips114_showstr(15, 4, "lrd_mid_del =");
				ips114_showfloat(9 * 12, 4, lmid_del, 4, 3);
				ips114_showstr(15, 5, "r2 =");
				ips114_showfloat(9 * 12, 5, r2, 4, 3);				
			}
			break;
			case 2: // �һ���
			{
				ips114_showstr(15, 0, "rrd_diff =");
				ips114_showfloat(9 * 12, 0, rigtround_diff, 4, 3);
				ips114_showstr(15, 1, "rrd_redy =");
				ips114_showfloat(9 * 12, 1, rigtround_ready, 4, 3);
				ips114_showstr(15, 2, "rigt_yaw =");
				ips114_showfloat(9 * 12, 2, rigtround_yawtarget, 4, 3);
				ips114_showstr(15, 3, "prea_r_dis =");
				ips114_showfloat(9 * 12, 3, pretoreadyR_round_distance, 4, 3);
				ips114_showstr(15, 4, "rrd_mid_del =");
				ips114_showfloat(9 * 12, 4, rmid_del, 4, 3);
				ips114_showstr(15, 5, "l2 =");
				ips114_showfloat(9 * 12, 5, l2, 4, 3);		

				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // ����λ��
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// ����λ����ʾ
			}
			break;
			case 4: // �ٶȼ���
			{
				ips114_showstr(15, 0, (uint8 *)"motor-speed");
				ips114_showfloat(8 * 18, 0, Forward_PID.p_out, 4, 4);
				ips114_showstr(15, 1, "Curnt_l =");
				ips114_showfloat(8 * 12, 1, Current_Speed_L, 4, 3);
				ips114_showfloat(8 * 20, 1, Current_Speed_L - Target_Speed_L, 2, 3);
				ips114_showstr(15, 2, "Curnt_r =");
				ips114_showfloat(8 * 12, 2, Current_Speed_R, 4, 2);
				ips114_showfloat(8 * 20, 2, Current_Speed_R - Target_Speed_R, 3, 3);
				ips114_showstr(15, 3, "isr_count_flag =");
				//				ips114_showint16(8 * 14, 3, isr_count_flag / 200);
				ips114_showstr(15, 4, "Tgt_Spd_L=");
				ips114_showfloat(8 * 16, 4, Target_Speed_L, 3, 3);
				ips114_showstr(15, 5, "Tgt_Spd_R=");
				ips114_showfloat(8 * 16, 5, Target_Speed_R, 3, 3);
				ips114_showstr(15, 6, "duty*L =");
				ips114_showint16(8 * 10, 6, set_duty_L);
				ips114_showstr(8 * 17, 6, "*R =");
				ips114_showint16(8 * 20, 6, set_duty_R);
				ips114_showfloat(15, 7, Err_Hori, 3, 3);
				ips114_showfloat(8 * 11, 7, Err_Vert, 3, 3);
				ips114_showint16(8 * 20, 7, Target_speed);
			}
			break;
			}
		}
		break;
		case 5: // ����������
		{
			ips114_showstr(15, 0, (uint8 *)"Current_Speed_L");		// PID��������
			ips114_showstr(15, 1, (uint8 *)"Current_Speed_R");				// P
			ips114_showstr(15, 2, (uint8 *)"intop_dis");				// I
			ips114_showstr(15, 3, (uint8 *)"intop_rad");				// D
			ips114_showfloat(18 * 8, 0, Current_Speed_L, 4, 5);	// P������ʾ
			ips114_showfloat(18 * 8, 1, Current_Speed_R, 4, 5);			// P������ʾ
			ips114_showfloat(15 * 8, 2, intop_distance, 4, 5);	// P������ʾ
			ips114_showfloat(15 * 8, 3, intop_radiu, 4, 5);			// P������ʾ
			ips114_showstr(15, 4, (uint8 *)"outp_ang");					// P
			ips114_showstr(15, 5, (uint8 *)"outp_dis");					// I
			ips114_showstr(15, 6, (uint8 *)"outp_rad");					// D
			ips114_showfloat(15 * 8, 4, outp_angle, 4, 5);			// P������ʾ
			ips114_showfloat(15 * 8, 5, outp_distance, 4, 5);		// P������ʾ
			ips114_showfloat(15 * 8, 6, outp_radiu, 4, 5);			// P������ʾ
			ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // ����λ��
			ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// ����λ����ʾ
		}
		break;
		case 6: // ��ˢ�����ʾ����
		{
			ips114_showstr(15, 1, (uint8 *)"brusheless_duty");
			ips114_showint16(8 * 18, 4, brusheless_duty);
			ips114_showstr(15, 3, (uint8 *)"brusheless_flag");
			ips114_showint16(8 * 18, 3, brusheless_flag);


			// ips114_showstr(15, 1, (uint8 *)"Roundabout-mode"); // ����ģʽ
			// ips114_showuint16(15, 2, Round_mode);							 // 1�Ǵ� 0�ǹر�
			// ips114_showstr(15, 5, (uint8 *)"base_speed");			 // ����ģʽ
			// ips114_showuint16(15 * 8, 5, base_speed);					 // 1�Ǵ� 0�ǹر�
			// 																									 /* 1 СС 2 С�� 3 С�� 4 ��С 5 ���� 6 �д� 7 ��С 8 ���� 9 ��� */


		}
		break;
		case 7: // �����ʾ����
		{
			switch (sys_display[2])
			{
			case 0:
				ips114_showstr(0, 0, "origin:");
				ips114_showstr(0, 1, "processed:");
				break;
			case 1:
				ips114_showfloat(0, 0, Err_Hori, 3, 3);
				ips114_showfloat(10 * 8, 0, Err_Vert, 3, 3);
				ips114_showfloat(21 * 8, 0, Err_2_Hori, 3, 3);
				ips114_showstr(0, 1, "data1:");
				ips114_showint16(6 * 8, 1, adc[1].data_0);
				ips114_showstr(0, 2, "data2:");
				ips114_showint16(6 * 8, 2, adc[2].data_0);
				ips114_showstr(0, 3, "data3:");
				ips114_showint16(6 * 8, 3, adc[3].data_0);
				ips114_showstr(0, 4, "data4:");
				ips114_showint16(6 * 8, 4, adc[4].data_0);
				ips114_showstr(0, 5, "data5:");
				ips114_showint16(6 * 8, 5, adc[5].data_0);

				ips114_showstr(13 * 8, 1, "data6:");
				ips114_showint16(21 * 8, 1, adc[6].data_0);
				ips114_showstr(13 * 8, 2, "data7:");
				ips114_showint16(21 * 8, 2, adc[7].data_0);
				ips114_showstr(13 * 8, 3, "data8:");
				ips114_showint16(21 * 8, 3, adc[8].data_0);
				ips114_showstr(13 * 8, 4, "data9:");
				ips114_showint16(21 * 8, 4, adc[9].data_0);
				ips114_showstr(13 * 8, 5, "data10:");
				ips114_showint16(21 * 8, 5, adc[10].data_0);
				break;
				case 2:
				ips114_showfloat(0, 0, Err_Hori, 3, 3);
				ips114_showfloat(10 * 8, 0, Err_Vert, 3, 3);
				ips114_showfloat(21 * 8, 0, Err_2_Hori, 3, 3);
				ips114_showstr(0, 1, "adc_L1");
				ips114_showfloat(6 * 8, 1, adc_L1,3,2);
				ips114_showstr(0, 2, "adc_L2:");
				ips114_showfloat(6 * 8, 2, adc_L2,3,2);
				ips114_showstr(0, 3, "adc_M:");
				ips114_showfloat(6 * 8, 3, adc_M,3,2);
				ips114_showstr(0, 4, "adc_R2");
				ips114_showfloat(6 * 8, 4, adc_R2,3,2);
				ips114_showstr(0, 5, "adc_R1:");
				ips114_showfloat(6 * 8, 5, adc_R1,3,2);

				ips114_showstr(13 * 8, 1, "adc2_L1");
				ips114_showfloat(21 * 8, 1, adc_2_L1,3,2);
				ips114_showstr(13 * 8, 2, "adc2_L2");
				ips114_showfloat(21 * 8, 2, adc_2_L2,3,2);
				ips114_showstr(13 * 8, 3, "adc2_M");
				ips114_showfloat(21 * 8, 3, adc_2_M,3,2);
				ips114_showstr(13 * 8, 4, "adc2_R2");
				ips114_showfloat(21 * 8, 4,	adc_2_R2,3,2);
				ips114_showstr(13 * 8, 5, "adc2_R1");
				ips114_showfloat(21 * 8, 5, adc_2_R1,3,2);
				break;
			default:
				break;
			}
		}
		break;
		case 8: // ���ϲ�������
		{
			ips114_showstr(15, 0, (uint8 *)"obstacle_avoidance"); // PID��������
			ips114_showstr(15, 1, (uint8 *)"out_angle");					// P
			ips114_showstr(15, 2, (uint8 *)"out_distance");				// I
			ips114_showstr(15, 3, (uint8 *)"in_angle");						// D
			ips114_showfloat(20 * 8, 1, out_angle, 4, 5);					// P������ʾ
			ips114_showfloat(20 * 8, 2, out_distance, 4, 5);			// P������ʾ
			ips114_showfloat(20 * 8, 3, in_angle, 4, 5);					// P������ʾ
			ips114_showstr(15, 4, (uint8 *)"in_distance");				// P
			ips114_showfloat(20 * 8, 4, in_distance, 4, 5);				// P������ʾ
			ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time");		// ����λ��
			ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);			// ����λ����ʾ
		}
		break;
		case 9: // ����
		{
			ips114_showstr(15, 0, "dill");
			ips114_showfloat(8 * 16, 0, dl1a_distance_mm, 4, 4);
			ips114_showstr(15, 1, "prep_av");
			ips114_showfloat(8 * 16, 2, pre_distance, 4, 4);
			ips114_showstr(15, 2, "read_av");
			ips114_showfloat(8 * 16, 3, ready_distance, 4, 4);
			ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // ����λ��
			ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// ����λ����ʾ
		}
		break;
		case 10: // ���糡�����ֵ��Сֵ��ʱ��ʾ
		{
			ips114_showstr(0, 1, "dt1_max:");
			ips114_showstr(16 * 8, 1, "dt1_min:");
			ips114_showint16(8 * 8, 1, adc[1].data_max);
			ips114_showint16(24 * 8, 1, adc[1].data_min);
			ips114_showstr(0, 2, "dt2_max:");
			ips114_showstr(16 * 8, 2, "dt2_min:");
			ips114_showint16(8 * 8, 2, adc[2].data_max);
			ips114_showint16(24 * 8, 2, adc[2].data_min);
			ips114_showstr(0, 3, "dt3_max:");
			ips114_showstr(16 * 8, 3, "dt3_min:");
			ips114_showint16(8 * 8, 3, adc[3].data_max);
			ips114_showint16(24 * 8, 3, adc[3].data_min);
			ips114_showstr(0, 4, "dt4_max:");
			ips114_showstr(16 * 8, 4, "dt4_min:");
			ips114_showint16(8 * 8, 4, adc[4].data_max);
			ips114_showint16(24 * 8, 4, adc[4].data_min);
			ips114_showstr(0, 5, "dt5_max:");
			ips114_showstr(16 * 8, 5, "dt5_min:");
			ips114_showint16(8 * 8, 5, adc[5].data_max);
			ips114_showint16(24 * 8, 5, adc[5].data_min);
			ips114_showstr(0, 6, "dt6_max:");
			ips114_showstr(16 * 8, 6, "dt6_min:");
			ips114_showint16(8 * 8, 6, adc[6].data_max);
			ips114_showint16(24 * 8, 6, adc[6].data_min);
			ips114_showstr(0, 7, "dt7_max:");
			ips114_showstr(16 * 8, 7, "dt7_min:");
		}
		break;

		default:
			printf("error\n");
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��õ�ص�ѹ
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void ADC_battery_quire() // ��õ�ص�ѹ
{
	ad_result = adc_once(BAT_VOL_PIN, ADC_12BIT);
	temp = (((uint32)ad_result * 5000) / 4096); // �������ǰadc���ŵĵ�ѹ ���㹫ʽΪ ad_result*VCC/ADC�ֱ���    VCC��λΪmv
	battery_voltage = temp * 11;								// �������ŵ�ѹ  �ͷ�ѹ�������ֵ�����ص�ѹ ���㹫˾Ϊ   ���ŵ�ѹ*(R2+R3)/R3   R3Ϊ�ӵض˵���

	// if(battery_voltage<11500&&battery_voltage>10000)
	// {
	// 	if(isr_count_flag%100<50)
	// 	{BEEP=1;trigflag_beep==0;}
	// 	else if(count_flag%100>50)
	// 	{BEEP=0;trigflag_beep==1;}
	// }
	// else if(trigflag_beep==0&&(battery_voltage>11500||battery_voltage<10000))
	// 	{BEEP=0;}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����PID����
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void PID_all_set() // ����PID����
{
	Speed_PID_L.KP = 30.1f;
	Speed_PID_L.KI = 2.31f;
	Speed_PID_L.KD = 0.0f;
	Speed_PID_R.KP = 33.0f;
	Speed_PID_R.KI = 2.32f;
	Speed_PID_R.KD = 0.0f;
	Forward_PID.KP = 8.2f;
	Forward_PID.KI = 1.5f;
	Forward_PID.KD = 80.0f;
	Swerve_PID.KP = 11.2f;
	Swerve_PID.KI = 1.2f;
	Swerve_PID.KD = 72.0f;
	Pout_PID.KP = 75.0f;
	Pout_PID.KI = 0.025f;
	Pout_PID.KD = 3200.0f;
	Pout_rate_PID.KP = 0.12f;
	Pout_rate_PID.KI = 0.0f;
	Pout_rate_PID.KD = 0.1f;
	climb_PID.KP = 0.0f;
	climb_PID.KI = 0.0;
	climb_PID.KD = 0.0f;
	roundaboutL_PID.KP = 13.2f;
	roundaboutL_PID.KI = 1.2f;
	roundaboutL_PID.KD = 90.0f;
	roundaboutR_PID.KP = 13.2f;
	roundaboutR_PID.KI = 1.2f;
	roundaboutR_PID.KD = 90.0f;
	gyro_d = -0.1f;
	pid_init(&Speed_PID_L);
	pid_init(&Speed_PID_R);
	pid_init(&Swerve_PID);
	pid_init(&Forward_PID);
	pid_init(&Pout_PID);
	pid_init(&roundaboutL_PID);
	pid_init(&roundaboutR_PID);
	// iap_erase(0x20);//����0-0X200
	// iap_erase(0x40);//����0X400
}