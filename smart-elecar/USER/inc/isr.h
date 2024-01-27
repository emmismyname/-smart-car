/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/

#ifndef __ISR_H_
#define __ISR_H_

extern void Button_Scan();
extern void ADC_battery_quire(); // 获得电池电压
extern void motor_output();      // 电机输出PWM
extern void ADC_battery_quire(); // 电压检测
extern void Element_Idef();      // 模式
extern void pid_servo();         // 方向环计算输出
extern void Hall_detection();    // 霍尔检测
extern void TOF_detection();     // 检测红外距离
extern void outP_moving();       // 运动代码

#endif