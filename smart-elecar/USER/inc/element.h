#ifndef ELEMENT_H
#define ELEMENT_H

#include "carcontrol.h"
#include "imu.h"
#define BEEP P67
// 停车触发
#define HRTRIG P26

void TOF_detection(); // TOF检测
void Element_Idef();  // 元素识别
void Hall_detection(); //霍尔检测

/********************************元素识别变量******/
extern int8 Element_flag; // 0是无元素 1是有元素
extern int8 Swerve_flag;  // 0是直道 1是弯道
extern int16 UpSlope;
extern int16 DownSlope_begin;
extern int16 TurnLeft, TurnRight;
extern int16 Cross;
extern uint32 isr_counter_start;
/***************************************************************************************/

/*******************环岛变量*******************/
// extern int32 judge_delay_flag;
// extern int16 RoundAbout_small_left;
// extern int16 RoundAbout_small_right;
// extern int16 RoundAbout_big_left;
// extern int16 RoundAbout_big_right;
// extern int16 Roundflag;
// extern int16 roundcount;
// /**********************************************/

/*******************环岛变量*******************/
extern float pretoreadyL_round_distance;
extern float pretoreadyR_round_distance;
extern float leftround_diff;
extern float rigtround_diff;
extern float leftround_ready;
extern float rigtround_ready;
extern float leftround_yawtarget;
extern float rigtround_yawtarget;
extern float lmid_del;
extern float rmid_del;
extern float r2;
extern float l2;
extern int16 Roundflag;
extern int16 roundcount;
extern int16 rightRound;
extern int16 leftRound;
extern int16 Round_mode;
// /**********************************************/

extern int8 block_flag;   // 障碍标志
extern int8 block_count;  // 判读次数
extern int16 blockcount_flag; //判读标志位
extern int32 block_timer_end;  // 避障计时器
extern int8 blockcount_target;  // 目标判读次数
extern int32 block_timer_count; //计时器计数
extern uint32 isr_count_flag;
extern int OUTP_flag;        // 定时器出库启动计算
extern int HRTRIG_flag;
extern float distance_break;//刹车距离
/////////避障
extern float pre_distance;
extern float ready_distance;

#endif