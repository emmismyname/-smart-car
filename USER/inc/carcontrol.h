#ifndef _CARCONTROL_H_
#define _CARCONTROL_H_

#include "headfile.h"
#include "motor.h"
#include "adc_aquire.h"
#include "imu.h"
#include "car_state.h"
// #include "eeprom.h"
// #include "element.h"


typedef struct // 定义pid常量参数及输出变量
{
	 uint32 start_time; //发车时间标志位
	 uint32 end_time;   //停车时间标志位
   uint32 running_time;   //停车时间标志位
   uint8 work_flag;//工作标志位 0在停止工作1开始工作
} time_run;



// /****************************运动状态检测********************************/




void Element_Idef(); // 元素识别
void mode_choose(int move_mode); // 运动模式切换
void car_start();//发车
void car_strategy(uint8 s_mode);//发车模式


#endif