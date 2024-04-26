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
 * @Target core		STC32
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2023-07-27

 ********************************************************************************************************************/
#include "headfile.h"
#include "mycode.h"


// 关于内核频率的设定，可以查看board.h文件
// 在board_init中,已经将P54引脚设置为复位
// 如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可


void main()
{ 
		//板载初始化·引脚
		EAXFR=1;//使能访问XFR
		clock_init(SYSTEM_CLOCK_52M);	// 初始化系统频率,勿删除此句代码。
		board_init();					// 初始化寄存器,勿删除此句代码。
		DisableGlobalIRQ(); // 	
		ctimer_count_init(SPEEDR_PLUSE);					// 初始化定时器4作为外部计数
		ctimer_count_init(SPEEDL_PLUSE);					// 初始化定时器3作为外部计数
		spi_slave_init();							// 初始化SPI(作为从机)		
		wireless_uart_init();						// 无线转串口模块初始化
	
		pit_timer_ms(TIM_1, 5);										// 使用定时器4做周期中断，时间5ms一次。电机控制
			// 此处编写用户代码 例如外设初始化代码等
	//  gpio_mode(P0_5, GPO_PP);									// P64引脚设置为推挽输出
		motor_init();
		switch_init();                //按键初始化
		iap_init();				// 初始化EEPROM
		EnableSPIIRQ();								// 使能SPI中断
		EnableGlobalIRQ();						// 使能总中断
		//20  0.6  40
	  pid_all_init();//初始化
		disenablemotor();//关闭电机
		car_init(&jiangxing_No_1);//车状态初始化
		
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
  
	
	  get_switch();//获取初始化时候的拨码状态，用于判断是否要读取eeprom的数值
    if(SWITCH3_FLAG==1)
		{
     read_all_par();
		}
		else//使用设定数值
		{
			car_init(&jiangxing_No_1);//车状态初始化
				
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
  

	 // 此处编写用户代码 例如外设初始化代码等
    while(1)	
    {
			
			enablemotor();//使能电机	
			MPU_Acquire();			
			wireless_read(1);
			wireless_send(jiangxing_No_1.uart_mode, 5);
			printf("%d \n",uart_mode);
			// read_all_par();
			// printf("%d,%d,%d,%d \n",SWITCH1_CURRENTSTATE,SWITCH2_CURRENTSTATE,SWITCH3_CURRENTSTATE,SWITCH4_CURRENTSTATE);
        // 此处编写需要循环执行的代码
    }
}


