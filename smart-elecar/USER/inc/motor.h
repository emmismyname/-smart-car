#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "common.h"
#include "zf_tim.h"
#include "zf_pwm.h"

// 定义脉冲引脚
#define SPEEDR_PLUSE CTIM0_P34
#define SPEEDL_PLUSE CTIM3_P04
// 定义方向引脚
#define SPEEDR_DIR P35
#define SPEEDL_DIR P53
// 设置电机
#define DIR_2 P64
#define DIR_1 P60
#define PWM_2 PWMA_CH4P_P66
#define PWM_1 PWMA_CH2P_P62

void counte_quire(); // 获得编码器脉冲个数
void brushes_out();	 // 无刷控制输出
void motor_output(); // 电机输出

typedef struct // 定义pid常量参数及输出变量
{
	float KP;	   // P 比例系数
	float KI;	   // I 积分系数
	float KD;	   // D 微分系数
	float i_limit; // 积分限幅

	float p_out; // KP输出
	float i_out; // KI输出
	float d_out; // KD输出

	float last_err;		  // 上次偏差值
	float lastBefore_err; // 上上次偏差值
	float err;			  // 偏差值

	float pid_out; // PID控制器输出
} PID_P;

extern PID_P Speed_PID_L; // 左轮速度pid
extern PID_P Speed_PID_R; // 右轮速度pid

extern PID_P Swerve_PID;	  // 转弯差速pid
extern PID_P Forward_PID;	  // 直行PID控制
extern PID_P roundaboutL_PID; // 左环岛PID控制
extern PID_P roundaboutR_PID; // 右环岛PID控制
extern PID_P Pout_PID;		  /// 陀螺仪PID控制
extern PID_P Pout_rate_PID; //角速度PID控制
extern PID_P climb_PID;		  /// 坡道PID控制

extern uint8 brusheless_flag;  // 无刷开关
extern uint16 brusheless_duty; // 无刷电机数值数值范围在500-1000对应占空比为0%-100%
extern uint8 motor_flag;

extern uint8 Dir_L;	  // 1左边方向正转  0反转
extern uint8 Dir_R;	  // 0右边方向正转  1反转
extern uint8 Dir_car; // 正走dir_car=1 L=1 R=0  ;dir_car=0 反走 L=0 R=1

extern float Target_Speed_L; // 左轮目标速度
extern float Target_Speed_R; // 右轮目标速度

extern float Current_Speed_L; // 左轮当前距离
extern float Current_Speed_R; // 右轮当前距离

extern int32 distance_L; // 左轮目标速度
extern int32 distance_R; // 右轮目标速度

extern float duty_L;	 // 左电机pid占空比
extern float duty_R;	 // 右电机pid占空比
extern float set_duty_L; // 左电机输出占空比
extern float set_duty_R; // 右电机输出占空比

extern int templ_pluse;	 // 左轮脉冲个数
extern int tempr_pluse;	 // 右轮脉冲个数
extern int Target_speed; // 车体目标速度

// extern float PIDOUT_speed_L;//左轮速度环的偏差计算结果
// extern float PIDOUT_speed_R;//右轮速度环的偏差计算结果

// extern int32 Target_pluse_L=0;//左轮目标脉冲数目
// extern int32 Target_pluse_R=0;//右轮目标脉冲数目

// extern int32 Motor_pluse_L;//左轮速度环的脉冲个数
// extern int32 Motor_pluse_R;//左轮速度环的脉冲个数

// extern float Motor_duty_L=470;	//左电机目标初始占空比用于克服死区电压，电源为满电状态
// extern float Motor_duty_R=360;	//右电机目标初始占空比用于克服死区电压，电源为满电状态
void pid_init(PID_P *pid);
//-------------------------------------------------------------------------------------------------------------------
//   @brief      pid的误差输出初始化
//   @param      pid				pid参数
//   @param      error			pid输入误差
//   @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_Control(PID_P *pid, float error);
//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid位置式控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_direction_Control(PID_P *pid, float error);
//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid增量式控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Inc_Control(PID_P *pid, float error);
//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid位置式控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_motor_Control(PID_P *pid, float error);
//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid角度控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_angle_Control(PID_P *pid, float error);

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid角速度控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_angle_rate_Control(PID_P *pid, float error);
#endif