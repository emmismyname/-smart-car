#ifndef _CARCONTROL_H_
#define _CARCONTROL__H_

#include "adc_acquire.h"

extern int display_switch; // 显示屏的开关状态 1开 0关
extern int button_switch;  // 显示屏的开关状态 1开 0关
extern uint8 start_flag;   // 发车标志位
extern uint8 stop_flag;    // 停车标志位
/****************************运动状态检测********************************/
extern int running_mode;   // 1是待机 2是发车 3普通寻迹（计时器）
extern uint8 trace_flag;   // 寻迹开关
extern int current_number; // 扫描开始标志位
extern int trigflag_beep;  // 蜂鸣器防误触
extern int outward_basin;  // 出库标志位
extern uint8 avoid_move;   // 避障开关
extern uint8 TOF_object;   // 未检测到 1检测到障碍物 2检测到坡道
extern int TOF_barrier[9];
extern int OUTP_flag; // 定时器启动计算 0不启用运动控制 1右出库 2左出库 3上坡直行
extern int err_speed;
extern uint8 small_radiu;//小圆引入环半径

//避障
extern float err_angle_rate;
extern int outtoin_flag;
extern float yaw_target;   //避障角度
extern float out_distance;//避障距离
extern float in_distance;//回正距离
extern float out_angle;//避障目标角度
extern float in_angle;//回正目标角度



//库状态
extern int pack_mode;
extern int strategy_mode; 
//入库
extern float intop_angle;//入库目标角度
extern float intop_distance;//入库距离
extern float intop_radiu;//入库圆环
/***************************************************************************************/

//出库
extern float outp_angle;//出库目标角度
extern float outp_distance;//入库距离
extern float outp_radiu;//入库圆环
/***************************************************************************************/
extern float move_z_gyro;


// extern void outP_moving();              // 运动代码

// extern void left_outP();                // 向右出库控制代码
// extern void right_outP();               // 向右出库控制代码
extern void strategy_choose(int mode);
extern void pack_choose(int mode);
extern void mode_choose(int move_mode); // 运动模式切换
extern int obstacle_avoidance(float out_angle1, float in_angle1,float target_out_distance1,float target_in_distance1); //避障处理
extern void outp(float target_angle,float target_distance,float target_radiu);
extern int8 intop(float target_angle,float target_distance,float target_radiu);
extern void work(int mode);


#endif