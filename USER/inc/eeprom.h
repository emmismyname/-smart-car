#ifndef _EEPROM_H_
#define _EEPROM__H_
#include "common.h"
#include "zf_pwm.h"
#include "motor.h"
#include "zf_eeprom.h"
#include "zf_pwm.h"
#include "imu.h"

/********************************PID相关变量参数输入标志位**********/
extern uint8 para_change_flag;


/********************************PID相关变量**********/
extern int rule_d[7];  // 模糊规则表 D
extern float kp_m;     // 模糊kp最大值
extern float kd_m;     // 模糊kp最大值
extern float err;      // 寻迹误差
extern float err_last; // 上一次的寻迹误查
extern float angle_err;
extern float Kp, Kd, output;
extern float EC, E;
/***************************************************************************************************/

/********************************静态储存操作函数**********/
extern void write_all_par(); // 利用eeporm写入数据
extern void read_all_par();  // 开机读取数据
/***************************************************************************************************/

#endif
