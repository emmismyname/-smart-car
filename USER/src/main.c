/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"
#include "allcodeinr.h"

/*********************************************函数声明*********************************************************/

/********************************初始化操作函数**********/
void initall_PID();			// PID结构体内误差清理
void arangement_init(); // 初始化设定
void parameter_init(); // 初始化设定
/***************************************************************************************************/

/********************************运动模式操作函数**********/

void Hall_detection(); // 霍尔检测
void TOF_detection();	 // 红外检测
void left_infrared();	 // 左红外避障
void right_infrared(); // 右红外避障

/***************************************************************************************************/

/********************************电机输出和电磁电压检测操作函数**********/
// void motor_output();//电机PWM输出
// void brushes_out();//无刷电机输出
void ADC_battery_quire(); // 获取电池的电压数值
// void counte_quire();//获得编码器脉冲个数
/***************************************************************************************************/

/********************************显示屏操作函数**********/
void system_display(); // 根据按键反馈对应界面
/***************************************************************************************************/

/********************************电感函数**********/
void Element_Idef(); // 元素识别
/**************************************************/

/********************************PID相关函数**********/
void PID_all_set();
// void pid_servo(void);	  // pid输出
/****************************************************/
void start_dection(); // 发车检测

/*
 * 系统频率，可查看board.h中的 FOSC 宏定义修改。
 * board.h文件中FOSC的值设置为0,则程序自动设置系统频率为33.1776MHZ
 * 在board_init中,已经将P54引脚设置为复位
 * 如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */

void main()
{
	arangement_init(); // 初始化设定
	parameter_init();//参数初始化
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
		// wireless_read(1);						 // 无线转串口入出
		system_display();						 // 显示界面
		Button_Scan();							 // 按钮扫描
		start_dection();						 // 发车检测
		resetelec_Init();						 // 电感最大最小值检测
		wireless_send(uart_mode, 5); // 无线转串口输出
	}
}

void start_dection() // 发车检测
{
	if (start_flag == 1)
	{

		// 初始化发车//无刷启动
		// PID结构体清零
		brusheless_flag = 1;
		brusheless_duty = 800;
		delay_ms(1500);
		HRTRIG_flag = 1; // 停车
		stop_flag = 0;	 // 停车归零
		OUTP_flag = 0;	 // 出库设置
		// outward_basin = 0; // 初始化出库
		initall_PID();
		// 初始化读取积分零漂
		motor_flag = 0;
		gyro_flag = 1;
		yaw_output = 0.0f;
		pitch = 4.0f;
		// 元素初始化/////////////
		roundcount = 0;
		leftRound = 0;
		rightRound = 0;
		UpSlope = 0;
		distance_L = 0;
		distance_R = 0;
		isr_counter_start = isr_count_flag; // 霍尔防误判
		block_timer_count = 0;							// 避障计数归零
		blockcount_flag = 0;								// 避障标志位
		block_count = 0;										// 计数清零
		/////////////////////////////////////////
		// 显示屏开关关闭
		button_switch = 0;
		display_switch = 0;
		OUTP_flag = 0;		 // 出库设置
		outward_basin = 1; // 出库
		// 陀螺仪初始化
		motor_flag = 1;
		// outward_basin = 0; // 初始化出库
		trace_flag = 1;
		Element_flag = 0; // 无元素
		// OUTP_flag = 1;		// 启用出库
		TOF_object = 0;						 // 避障个数
		avoid_move = 0;						 // 避障过程标志位
		Target_speed = base_speed; // 速度控制
		//////////////////////
		start_flag = 0; // 清除标志位
										/////////////////////
	}
	// {										// 初始化发车//无刷启动
	// 	Target_speed = 0; // 速度控制
	// 	motor_flag = 0;
	// 	trace_flag = 1;
	// 	// 陀螺仪初始化
	// 	gyro_flag = 0;
	// 	yaw_output = 0; // 初始化
	// 	pitch_result = 0.0f;
	// 	gyro_flag = 1;
	// 	// PID结构体清零
	// 	initall_PID();
	// 	// 初始化读取积分零漂
	// 	// OUTP_flag=0;//出库设置
	// 	Target_speed = 50; // 速度控制
	// 	motor_flag = 1;
	// 	start_flag = 0;
	// 	running_mode = 8;
	// }
}

void arangement_init() // 初始化设定
{
	board_init(); // 初始化寄存器,勿删除此句代码。
	// 此处编写用户代码(例如：外设初始化代码等)
	iap_init();					// 初始化EEPROM
	ips114_init();			// 初始化1.1.4寸ips屏幕
	DisableGlobalIRQ(); // 关闭总中断
	sys_clk = 35000000; // 设置系统频率为35MHz
	WTST = 0;						// 设置程序代码等待参数，赋值为0可将CPU执行程序的速度设置为最快
	// sys_clk可选值:30000000, 27000000. 24000000, 22118400, 20000000, 18432000, 12000000, 11059200, 6000000, 5529600。
	// 设置系统频率，此频率需要跟STC-ISP软件中的 <输入用户程序运行的IRC频率>选项的频率一致。
	// 如果频率设置不对，将会导致串口的数据不正常,PWM的工作不正常等等。
	board_init(); // 设置玩主频后再次初始化内部寄存器
	delay_init();
	ips114_init();														// 初始化1.1.4寸ips屏幕
	wireless_uart_init();											// 无线转串口模块初始化
	ctimer_count_init(SPEEDL_PLUSE);					// 初始化定时器0作为外部计数
	ctimer_count_init(SPEEDR_PLUSE);					// 初始化定时器3作为外部计数
	gpio_mode(KEY1_PIN, GPI_IMPEDANCE);				// S1按键引脚模式: 高阻输出
	gpio_mode(KEY2_PIN, GPI_IMPEDANCE);				// S2按键引脚模式: 高阻输出
	gpio_mode(KEY3_PIN, GPI_IMPEDANCE);				// S3按键引脚模式: 高阻输出
	gpio_mode(KEY4_PIN, GPI_IMPEDANCE);				// S4按键引脚模式: 高阻输出
	gpio_mode(HRTRIG, GPI_IMPEDANCE);					// S4按键引脚模式: 高阻输出
	gpio_mode(P6_7, GPO_PP);									// 将P6.7蜂鸣器设置为推挽输出
	BEEP = 0;																	// 蜂鸣器关闭
	pit_timer_ms(TIM_4, 5);										// 使用定时器4做周期中断，时间5ms一次。电机控制
	EnableGlobalIRQ();												// 开启总中断
	adc_init(BAT_VOL_PIN, ADC_SYSclk_DIV_32); // 初始化电压引脚
	initall_PID();														// PID结构体清零
	gpio_mode(P6_4, GPO_PP);									// P64引脚设置为推挽输出
	gpio_mode(P6_0, GPO_PP);									// P60引脚设置为推挽输出
	pwm_init(PWM_1, 17000, 0);								// 初始化PWM1  使用P60引脚  初始化频率为17Khz
	pwm_init(PWM_2, 17000, 0);								// 初始化PWM2  使用P62引脚  初始化频率为17Khz
	pwm_init(PWMB_CH4_P77, 50, 0);						// 初始化无刷电机为 50hz
	pwm_init(PWMB_CH3_P33, 50, 0);						// (1-2ms/20ms * 10000)（10000是PWM的满占空比时候的值） 10000为PWM最大值
	adc_init(ADC_P00, 0);											// P00引脚		具体通道与引脚对应关系可以查看zf_adc.h文件	  adc[7].data_0 = adc_mean_filter(ADC_P00,8);		//采集并处理ADC_P16电压，精度12位R1—————(L1)
	adc_init(ADC_P01, 0);											// P01引脚		具体通道与引脚对应关系可以查看zf_adc.h文件	  adc[6].data_0 = adc_mean_filter(ADC_P01,8);		//采集并处理ADC_P14电压，精度12位R2—————(L2)
	adc_init(ADC_P05, 0);											// P05引脚		具体通道与引脚对应关系可以查看zf_adc.h文件	  adc[5].data_0 = adc_mean_filter(ADC_P06,8);		//采集并处理ADC_P06电压，精度12位R3—————(L3)
	adc_init(ADC_P13, 0);											// P13引脚		具体通道与引脚对应关系可以查看zf_adc.h文件		adc[1].data_0 = adc_mean_filter(ADC_P13,8);		//采集并处理ADC_P17电压，精度12位L1—————(L7)
	adc_init(ADC_P14, 0);											// P14引脚		具体通道与引脚对应关系可以查看zf_adc.h文件    adc[2].data_0 = adc_mean_filter(ADC_P14,8);		//采集并处理ADC_P13电压，精度12位L2—————(L6)
	adc_init(ADC_P16, 0);											// P16引脚		具体通道与引脚对应关系可以查看zf_adc.h文件    adc[3].data_0 = adc_mean_filter(ADC_P16,8);		//采集并处理ADC_P01电压，精度12位L3—————(L5)
	adc_init(ADC_P17, 0);											// P17引脚		具体通道与引脚对应关系可以查看zf_adc.h文件	  adc[4].data_0 = adc_mean_filter(ADC_P17,8);		//采集并处理ADC_P05电压，精度12位M-—————(L4)
	adc_init(ADC_P06, 0);											// P06引脚		具体通道与引脚对应关系可以查看zf_adc.h文件
	adc_init(ADC_P10, 0);											// P10引脚
	adc_init(ADC_P11, 0);											// P11引脚
	// 外部中断2，3打开
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
	
	read_all_par();		 // 读取eeprom数值
	// Zero_Offset_set(); // mpu6050 DMP上电矫正时间
}

void parameter_init()//参数初始化
{
// 后排电感最大值
	adc[1].data_max = 2359;
	adc[2].data_max = 2424;
	adc[3].data_max = 3121;
	adc[4].data_max = 2399;
	adc[5].data_max = 2322;
	// 前排电感最大值
	adc[6].data_max = 3070;
	adc[7].data_max = 1802;
	adc[8].data_max = 2506;
	adc[9].data_max = 1698;
	adc[10].data_max = 3194;
	// 后排电感最小值
	adc[1].data_min = 0;
	adc[2].data_min = 0;
	adc[3].data_min = 0;
	adc[4].data_min = 0;
	adc[5].data_min = 0;
	// 前排电感最小值
	adc[6].data_min = 0;
	adc[7].data_min = 0;
	adc[8].data_min = 0;
	adc[9].data_min = 0;
	adc[10].data_min = 0;
		// ZeroOffset_init(); //yaw数值零飘
	//显示屏页面设置
	sys_display[1] = 1;
	sys_display[2] = 0;
	sys_display[3] = 0;
	button_switch = 1;//按钮开启
	display_switch = 1;
	// brusheless_duty = 0;无刷风扇设置
	// brusheless_flag = 1;无刷风扇开启标志位
	gyro_flag = 1;//开启陀螺仪
	yaw_output = 0.0f;
	// PID结构体清零
	initall_PID();
	// 初始化读取积分零漂
	Element_flag = 0; // 无元素
	motor_flag = 0;
	roundcount = 0;
	leftRound = 0;
	rightRound = 0;
	UpSlope = 0;

	distance_L = 0;//左轮里程
	distance_R = 0;//右轮里程

	block_count = 0;			 // 计数清零
	stop_flag = 0;				 // 停车归零
	isr_counter_start = 0; // 霍尔防误判
	block_timer_count = 0; // 霍尔计数归零
	blockcount_flag = 0;	 // 霍尔标志位
	// outward_basin = 1;		 // 出库
	// OUTP_flag = 0;				 // 出库设置
	HRTRIG_flag = 1; // 停车
	//////////////////////
	outward_basin = 0; // 初始化出库
	OUTP_flag = 1;		 // 启用出库

	start_flag = 0;		 // 清除标志位
	base_speed = 220;	 // 速度控制
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		霍尔检测
//  @param
//  @param
//  @return  将修改trace_flag寻迹开关
//-------------------------------------------------------------------------------------------------------------------
// void Hall_detection() // 左后
// {
// 	// int32 hall_counter=0;
// 	if (HRTRIG == 0 && isr_count_flag > 400)
// 	{
// 		if (outward_basin == 1)
// 		{
// 			yaw = 0;
// 			outward_basin = 2; // 结束
// 		}
// 		trace_flag = 0;
// 		running_mode = 0; // 待机
// 		motor_flag = 0;
// 		pid_init(&Speed_PID_L);
// 		pid_init(&Speed_PID_R);
// 		// PID_all_set();
// 		motor_flag = 1; // 电机输出
// 		distance_L = 0;
// 		distance_R = 0; // 出库
// 		gyro_flag = 0;
// 		yaw = 0.0f;
// 		Dir_car = 0;
// 		Target_speed = 50;
// 		while ((distance_L < 17000) && (distance_R < 18000))
// 		{
// 			gyro_flag = 1;
// 			OUTP_flag = 7; // 左后
// 			motor_flag = 1;
// 			imu660ra_get_gyro(); // 获取陀螺仪数据
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
// void Hall_detection() // 右后
// {
// 	int32 hall_counter = 0;
// 	if (HRTRIG == 0 && count_flag > 400)
// 	{
// 		if (outward_basin == 1)
// 		{
// 			yaw = 0;
// 			outward_basin = 2; // 结束
// 		}
// 		trace_flag = 0;
// 		running_mode = 0; // 待机
// 		motor_flag = 0;
// 		pid_init(&Speed_PID_L);
// 		pid_init(&Speed_PID_R);
// 		// PID_all_set();
// 		motor_flag = 1; // 电机输出
// 		distance_L = 0;
// 		distance_R = 0; // 出库
// 		gyro_flag = 0;
// 		yaw = 0.0f;
// 		Dir_car = 0;
// 		Target_speed = 50;
// 		while ((distance_L < 21000) && (distance_R < 20000))
// 		{
// 			gyro_flag = 1;
// 			OUTP_flag = 8; // 右后
// 			motor_flag = 1;
// 			imu660ra_get_gyro(); // 获取陀螺仪数据
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
// @brief		PID结构体内容清理
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
//  @brief      按钮获取
//  @param      按键状态
//  @param      拨码状态
//  @return     PID参数调整 显示界面
//-------------------------------------------------------------------------------------------------------------------
void Button_Scan()
{
	// 获取拨码开关状态
	sw1_status = SW1_PIN;
	sw2_status = SW2_PIN;

	// 使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费
	// 保存按键状态
	key1_last_status = key1_status;
	key2_last_status = key2_status;
	key3_last_status = key3_status;
	key4_last_status = key4_status;
	// 读取当前按键状态
	key1_status = KEY1_PIN;
	key2_status = KEY2_PIN;
	key3_status = KEY3_PIN;
	key4_status = KEY4_PIN;

	// 检测到按键按下之后  并放开置位标志位
	if (key1_status && !key1_last_status)
		key1_flag = 1;
	if (key2_status && !key2_last_status)
		key2_flag = 1;
	if (key3_status && !key3_last_status)
		key3_flag = 1;
	if (key4_status && !key4_last_status)
		key4_flag = 1;

	// 标志位置位之后，可以使用标志位执行自己想要做的事件
	if (key1_flag)
	{
		if (sw1_status == 1 && sw2_status == 1)
		{
			cursor = cursor - 1;
			ips114_clear(WHITE);											// 清屏
			ips114_showstr(0, cursor, (uint8 *)"->"); // 光标位置
		}
		///////////////使用按键之后，应该清除标志位
		key1_flag = 0;													// 使用按键之后，应该清除标志位
		if (sw1_status == 0 && sw2_status == 0) // 双拨码开关屏幕
		{
			ips114_clear(WHITE); // 清屏
			display_switch = 0;
			button_switch = 0;
		}
	}
	if (key2_flag)
	{
		if (sw1_status == 1 && sw2_status == 1)
		{
			cursor = cursor + 1;
			ips114_clear(WHITE);											// 清屏
			ips114_showstr(0, cursor, (uint8 *)"->"); // 光标位置
		}
		///////////////使用按键之后，应该清除标志位
		key2_flag = 0;													// 使用按键之后，应该清除标志位
		if (sw1_status == 0 && sw2_status == 0) // 双拨码开关屏幕
		{
			display_switch = 1;
			button_switch = 1;
		}
	}

	if (button_switch == 1) // debug打开按键调参
	{
		if (key3_flag)
		{
			if (sw1_status == 1 && sw2_status == 1) // 光标操作
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
				ips114_showstr(0, cursor, (uint8 *)"->"); // 光标位置
			}

			else if ((sw2_status == 0 || sw1_status == 0) && sys_display[1] == 2) // 软件打开电机占控比输出
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
					PID_all_set(); // 设置pid
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
			// pid代码
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 1) // PID1界面1拨杆为1 +multiple_time
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 1) // PID1界面1 拨杆为没动 调整multiple_time*10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 2) // PID2直道和弯道界面2拨杆为1 +multiple_time
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 2) // PID2直道和弯道界面1和2 拨杆为没动 调整multiple_time*10
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 3) // PID界面3 拨杆为没动 调整multiple_time*10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 3) // PID界面3	拨杆为1 +multiple_time
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 4) // PID4 界面4拨杆为1 +multiple_time
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
			else if (sw1_status ==  1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 4) // PID2直道和弯道界面1和2 拨杆为没动 调整multiple_time*10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 5) // PID4 界面4拨杆为1 +multiple_time
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 5) // PID2直道和弯道界面1和2 拨杆为没动 调整multiple_time*10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 5) // 入库 +multiple_time
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 5) // 入库拨杆为没动 调整multiple_time*10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 6) // 无刷定速拨杆为1 +100
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 6) // 无刷定速拨杆为2 +10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 1) // 环岛
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 1) // 环岛
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 2) // 环岛
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 2) // 环岛
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 4) // 左轮设定数值速拨杆为1 +5
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 4) // 左轮设定数值2拨杆为2 +1
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 8) // 避障
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 8) // 避障
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 9) // 避障
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 9) // 避障
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
			///////////////使用按键之后，应该清除标志位
			write_all_par();
			key3_flag = 0; // 使用按键之后，应该清除标志位
		}
		if (key4_flag)
		{
			if (sw1_status == 1 && sw2_status == 1) // 光标操作
			{
				if (sys_display[1] != 1 && sys_display[2] != 0) // 二级菜单回位
				{
					sys_display[2] = 0;
				}
				else if (sys_display[1] != 1 && sys_display[2] == 0) // 一级菜单回位
				{
					sys_display[1] = 1;
				}

				cursor = 0;
				ips114_clear(WHITE);
				ips114_showstr(0, cursor, (uint8 *)"->"); // 光标位置
			}
			else if ((sw2_status == 0 || sw1_status == 0) && sys_display[1] == 2) // 软件关闭电机占控比输出
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
					PID_all_set(); // 设置pid
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 1) // PID界面1 拨杆为1 -multiple_time
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 1) // PID界面1 拨杆为没动 调整multiple_time*0.1
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 2) // PID界面2拨杆为1 -multiple_time
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 2) // PID界面2 拨杆为没动 调整multiple_time*0.1
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 3) // PID界面3 拨杆为没动 调整multiple_time*10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 3) // PID界面3拨杆为1 +multiple_time
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 4) // PID4 界面4拨杆为1 +multiple_time
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 4) // PID2直道和弯道界面1和2 拨杆为没动 调整multiple_time*10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 3 && sys_display[2] == 4) // PID4 界面4拨杆为1 +multiple_time
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 3 && sys_display[2] == 4) // PID2直道和弯道界面1和2 拨杆为没动 调整multiple_time*10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 5) // 入库 +multiple_time
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 5) // PID2直道和弯道界面1和2 拨杆为没动 调整multiple_time*10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 6) // 无刷定速拨杆为1 -100
			{
				// 无刷定速拨杆为2 +10

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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 6) // 无刷定速拨杆为2 +10
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 1) // 环岛
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 1) // 环岛
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 2) // 环岛
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 2) // 环岛
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 4 && sys_display[2] == 4) // 左轮设定数值速拨杆为1 +5
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 4 && sys_display[2] == 4) // 左轮设定数值2拨杆为2 +1
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 8) // 避障
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 8) // 避障
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
			else if (sw1_status == 0 && sw2_status == 1 && sys_display[1] == 9) // 避障
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
			else if (sw1_status == 1 && sw2_status == 0 && sys_display[1] == 9) // 避障
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
			key4_flag = 0; // 使用按键之后，应该清除标志位
		}
	}


}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      显示界面系统的显示控制
//  @param      display_switch 1打开界面 0关闭界面
//  @param      显示界面0主菜单 界面1PID1参数 界面2PID2参数  界面3速度显示模块  界面4模式选择模块 界面5无刷电机模块进 界面6电磁ADC模块进入 界面7电池模块 界面8的PID3
//  @return     显示界面
//-------------------------------------------------------------------------------------------------------------------
void system_display() // 显示界面系统的显示控制
{

	if (display_switch) // 开关屏幕
	{
		switch (sys_display[1])
		{
		case 1: // 主菜单显示
		{
			ips114_showstr(15, 0, (uint8 *)"mode-choose");								// PID参数设置
			ips114_showstr(15, 1, (uint8 *)"arguments-pid");							// PID参数设置
			ips114_showstr(15, 2, (uint8 *)"round");											// 环岛模块参数进入
			ips114_showstr(15, 3, (uint8 *)"pack");												// 左右轮速度环调参选择模块进入
			ips114_showstr(15, 4, (uint8 *)"imu_display");       					// 姿态传感器显示载入
			ips114_showstr(15, 5, (uint8 *)"electric ADC ");							// 显示ADC电磁界面
			ips114_showstr(15, 6, (uint8 *)"obstacle_avoidance");					// 避障参数
			ips114_showstr(15, 7, (uint8 *)"dill");												// 显示ADC电磁界面
			ips114_showstr(0, cursor, (uint8 *)"->");											// 光标位置
			ips114_showuint16(8 * 20, 7, battery_voltage);								// 1是打开 0是关闭
			ips114_showfloat(20 * 8, 1, dl1a_distance_mm, 5, 1);					// P参数显示
		}
		break;
		case 2: // 模式界面
		{
			ips114_showfloat(26 * 8, 1, dl1a_distance_mm, 5, 1); // P参数显示
			ips114_showstr(15, 0, (uint8 *)"start_switch");			 // 发车标志位
			ips114_showuint8(8 * 16, 0, start_flag);						 // 1是打开 0是关闭
			ips114_showstr(15, 1, (uint8 *)"resetelec_flag");		 // 电机是否打开
			ips114_showuint8(8 * 16, 1, resetelec_flag);				 // 1是打开 0是关闭
			ips114_showstr(15, 2, (uint8 *)"PID_all_set");			 // 是否更新PID
			ips114_showstr(15, 3, (uint8 *)"uart=");
			ips114_showint16(9 * 16, 3, uart_mode);						 // 串口模式显示
			ips114_showstr(15, 4, (uint8 *)"trace_flag");			 // 寻迹开关
			ips114_showint8(8 * 16, 4, trace_flag);						 // 1是打开 0是关闭
			ips114_showstr(15, 5, (uint8 *)"stragy-choose");	 // 模式选择模块进入
			ips114_showint16(8 * 16, 5, strategy_mode);				 // 模式选择
			ips114_showstr(15, 6, (uint8 *)"pack-choose");		 // 模式选择模块进入
			ips114_showint16(8 * 16, 6, pack_mode);						 // 停车参数模式选择
			ips114_showstr(15, 7, (uint8 *)"strategy-choose"); // 模式选择模块进入
			ips114_showint16(8 * 20, 7, strategy_mode);				 // 停车参数模式选择
		}
		break;
		case 3: // 调参界面显示 1转弯 直行环 角度 左环 右环
		{
			switch (sys_display[2])
			{
			case 0:
			{
				ips114_showstr(15, 0, (uint8 *)"arguments-pid1_speed"); // PID参数设置
				ips114_showstr(15, 1, (uint8 *)"arguments-pid2_F&S");		// PID参数设置
				ips114_showstr(15, 2, (uint8 *)"arguments-pid3_P_out"); // PID参数设置
				ips114_showstr(15, 3, (uint8 *)"arguments-pid4_round"); // PID参数设置
				ips114_showstr(15, 4, (uint8 *)"arguments-pid4_rate");	// PID参数设置
			}
			break;
			case 1: // 基础速度
			{
				ips114_showstr(15, 0, (uint8 *)"arguments-pid1");		// PID参数设置
				ips114_showstr(15, 1, (uint8 *)"pidL-P");						// P
				ips114_showstr(15, 2, (uint8 *)"pidL-I");						// I
				ips114_showstr(15, 3, (uint8 *)"pidL-D");						// D
				ips114_showfloat(10 * 8, 1, Speed_PID_L.KP, 4, 5);	// P参数显示
				ips114_showfloat(10 * 8, 2, Speed_PID_L.KI, 4, 5);	// P参数显示
				ips114_showfloat(10 * 8, 3, Speed_PID_L.KD, 4, 5);	// P参数显示
				ips114_showstr(15, 4, (uint8 *)"pidR-P");						// P
				ips114_showstr(15, 5, (uint8 *)"pidR-I");						// I
				ips114_showstr(15, 6, (uint8 *)"pidR-D");						// D
				ips114_showfloat(10 * 8, 4, Speed_PID_R.KP, 4, 5);	// P参数显示
				ips114_showfloat(10 * 8, 5, Speed_PID_R.KI, 4, 5);	// P参数显示
				ips114_showfloat(10 * 8, 6, Speed_PID_R.KD, 4, 5);	// P参数显示
				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // 调整位数
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// 调整位数显示
			}
			break;
			case 2: // 直行转弯
			{
				ips114_showstr(15, 0, (uint8 *)"arguments-pid1");		// PID参数设置
				ips114_showstr(15, 1, (uint8 *)"Forward-P");				// P
				ips114_showstr(15, 2, (uint8 *)"Forward-I");				// I
				ips114_showstr(15, 3, (uint8 *)"Forward-D");				// D
				ips114_showfloat(14 * 8, 1, Forward_PID.KP, 4, 5);	// P参数显示
				ips114_showfloat(14 * 8, 2, Forward_PID.KI, 4, 5);	// P参数显示
				ips114_showfloat(14 * 8, 3, Forward_PID.KD, 4, 5);	// P参数显示
				ips114_showstr(15, 4, (uint8 *)"Swerve-P");					// P
				ips114_showstr(15, 5, (uint8 *)"Swerve-I");					// I
				ips114_showstr(15, 6, (uint8 *)"Swerve-D");					// D
				ips114_showfloat(14 * 8, 4, Swerve_PID.KP, 4, 5);		// P参数显示
				ips114_showfloat(14 * 8, 5, Swerve_PID.KI, 4, 5);		// P参数显示
				ips114_showfloat(14 * 8, 6, Swerve_PID.KD, 4, 5);		// P参数显示
				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // 调整位数
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// 调整位数显示
			}
			break;
			case 3: // 角度环
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
				ips114_showstr(15, 6, (uint8 *)"Targetspeed");			// 模式选择模块进入
				ips114_showint16(8 * 16, 6, Target_speed);					// 模式选择
				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // 调整位数
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 3);		// 调整位数显示
			}
			break;
			case 4: // 环岛
			{
				ips114_showstr(15, 0, (uint8 *)"arguments-pid4");			 // PID参数设置
				ips114_showstr(15, 1, (uint8 *)"roundaboutL-P");			 // P
				ips114_showstr(15, 2, (uint8 *)"roundaboutL-I");			 // I
				ips114_showstr(15, 3, (uint8 *)"roundaboutL-D");			 // D
				ips114_showfloat(20 * 8, 1, roundaboutL_PID.KP, 4, 5); // P参数显示
				ips114_showfloat(20 * 8, 2, roundaboutL_PID.KI, 4, 5); // P参数显示
				ips114_showfloat(20 * 8, 3, roundaboutL_PID.KD, 4, 5); // P参数显示
				ips114_showstr(15, 4, (uint8 *)"roundaboutR-P");			 // P
				ips114_showstr(15, 5, (uint8 *)"roundaboutR-I");			 // I
				ips114_showstr(15, 6, (uint8 *)"roundaboutR-D");			 // D
				ips114_showfloat(20 * 8, 4, roundaboutR_PID.KP, 4, 5); // P参数显示
				ips114_showfloat(20 * 8, 5, roundaboutR_PID.KI, 4, 5); // P参数显示
				ips114_showfloat(20 * 8, 6, roundaboutR_PID.KD, 4, 5); // P参数s显示
				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time");		 // 调整位数
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);			 // 调整位数显示
			}
			break;
			case 5: // 角度速度环
			{
				ips114_showstr(15, 1, "ratekp");
				ips114_showfloat(8 * 16, 1, Pout_rate_PID.KP, 4, 4);
				ips114_showstr(15, 2, "rateki");
				ips114_showfloat(8 * 16, 2, Pout_rate_PID.KI, 4, 4);
				ips114_showstr(15, 3, "ratekd");
				ips114_showfloat(8 * 16, 3, Pout_rate_PID.KD, 4, 4);
				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // 调整位数
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 3);		// 调整位数显示}
				break;
			default:
				break;
			}
				// ips114_showstr(15, 0, (uint8 *)"arguments-pid2");		// PID参数设置
				// ips114_showstr(15, 1, (uint8 *)"Forward-P");				// P
				// ips114_showstr(15, 2, (uint8 *)"Forward-I");				// I
				// ips114_showstr(15, 3, (uint8 *)"Forward-D");				// D
				// ips114_showfloat(14 * 8, 1, Forward_PID.KP, 4, 5);	// P参数显示
				// ips114_showfloat(14 * 8, 2, Forward_PID.KI, 4, 5);	// P参数显示
				// ips114_showfloat(14 * 8, 3, Forward_PID.KD, 4, 5);	// P参数显示
				// ips114_showstr(15, 4, (uint8 *)"Swerve-P");					// P
				// ips114_showstr(15, 5, (uint8 *)"Swerve-I");					// I
				// ips114_showstr(15, 6, (uint8 *)"Swerve-D");					// D
				// ips114_showfloat(14 * 8, 4, Swerve_PID.KP, 4, 5);		// P参数显示
				// ips114_showfloat(14 * 8, 5, Swerve_PID.KI, 4, 5);		// P参数显示
				// ips114_showfloat(14 * 8, 6, Swerve_PID.KD, 4, 5);		// P参数显示
				// ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // 调整位数
				// ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// 调整位数显示
			}
			break;
		}
		case 4: // 环岛界面
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
			case 1: // 左环岛
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
			case 2: // 右环岛
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

				ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // 调整位数
				ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// 调整位数显示
			}
			break;
			case 4: // 速度见面
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
		case 5: // 入库参数界面
		{
			ips114_showstr(15, 0, (uint8 *)"Current_Speed_L");		// PID参数设置
			ips114_showstr(15, 1, (uint8 *)"Current_Speed_R");				// P
			ips114_showstr(15, 2, (uint8 *)"intop_dis");				// I
			ips114_showstr(15, 3, (uint8 *)"intop_rad");				// D
			ips114_showfloat(18 * 8, 0, Current_Speed_L, 4, 5);	// P参数显示
			ips114_showfloat(18 * 8, 1, Current_Speed_R, 4, 5);			// P参数显示
			ips114_showfloat(15 * 8, 2, intop_distance, 4, 5);	// P参数显示
			ips114_showfloat(15 * 8, 3, intop_radiu, 4, 5);			// P参数显示
			ips114_showstr(15, 4, (uint8 *)"outp_ang");					// P
			ips114_showstr(15, 5, (uint8 *)"outp_dis");					// I
			ips114_showstr(15, 6, (uint8 *)"outp_rad");					// D
			ips114_showfloat(15 * 8, 4, outp_angle, 4, 5);			// P参数显示
			ips114_showfloat(15 * 8, 5, outp_distance, 4, 5);		// P参数显示
			ips114_showfloat(15 * 8, 6, outp_radiu, 4, 5);			// P参数显示
			ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // 调整位数
			ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// 调整位数显示
		}
		break;
		case 6: // 无刷电机显示界面
		{
			ips114_showstr(15, 1, (uint8 *)"brusheless_duty");
			ips114_showint16(8 * 18, 4, brusheless_duty);
			ips114_showstr(15, 3, (uint8 *)"brusheless_flag");
			ips114_showint16(8 * 18, 3, brusheless_flag);


			// ips114_showstr(15, 1, (uint8 *)"Roundabout-mode"); // 环岛模式
			// ips114_showuint16(15, 2, Round_mode);							 // 1是打开 0是关闭
			// ips114_showstr(15, 5, (uint8 *)"base_speed");			 // 环岛模式
			// ips114_showuint16(15 * 8, 5, base_speed);					 // 1是打开 0是关闭
			// 																									 /* 1 小小 2 小中 3 小大 4 中小 5 中中 6 中大 7 大小 8 大中 9 大大 */


		}
		break;
		case 7: // 电磁显示界面
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
		case 8: // 避障参数设置
		{
			ips114_showstr(15, 0, (uint8 *)"obstacle_avoidance"); // PID参数设置
			ips114_showstr(15, 1, (uint8 *)"out_angle");					// P
			ips114_showstr(15, 2, (uint8 *)"out_distance");				// I
			ips114_showstr(15, 3, (uint8 *)"in_angle");						// D
			ips114_showfloat(20 * 8, 1, out_angle, 4, 5);					// P参数显示
			ips114_showfloat(20 * 8, 2, out_distance, 4, 5);			// P参数显示
			ips114_showfloat(20 * 8, 3, in_angle, 4, 5);					// P参数显示
			ips114_showstr(15, 4, (uint8 *)"in_distance");				// P
			ips114_showfloat(20 * 8, 4, in_distance, 4, 5);				// P参数显示
			ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time");		// 调整位数
			ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);			// 调整位数显示
		}
		break;
		case 9: // 避障
		{
			ips114_showstr(15, 0, "dill");
			ips114_showfloat(8 * 16, 0, dl1a_distance_mm, 4, 4);
			ips114_showstr(15, 1, "prep_av");
			ips114_showfloat(8 * 16, 2, pre_distance, 4, 4);
			ips114_showstr(15, 2, "read_av");
			ips114_showfloat(8 * 16, 3, ready_distance, 4, 4);
			ips114_showstr(8 * 4, 7, (uint8 *)"multiple_time"); // 调整位数
			ips114_showfloat(20 * 8, 7, multiple_time, 4, 5);		// 调整位数显示
		}
		break;
		case 10: // 检测电场的最大值最小值临时显示
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
//  @brief      获得电池电压
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void ADC_battery_quire() // 获得电池电压
{
	ad_result = adc_once(BAT_VOL_PIN, ADC_12BIT);
	temp = (((uint32)ad_result * 5000) / 4096); // 计算出当前adc引脚的电压 计算公式为 ad_result*VCC/ADC分辨率    VCC单位为mv
	battery_voltage = temp * 11;								// 根据引脚电压  和分压电阻的阻值计算电池电压 计算公司为   引脚电压*(R2+R3)/R3   R3为接地端电阻

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
//  @brief      设置PID参数
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void PID_all_set() // 设置PID参数
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
	// iap_erase(0x20);//擦除0-0X200
	// iap_erase(0x40);//擦除0X400
}