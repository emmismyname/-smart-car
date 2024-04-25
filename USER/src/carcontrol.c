#include "carcontrol.h"

int err_speed = 0;
int running_mode = 1;		// 1是待机motor等于0 2是停车 3普通寻迹（计时器）
uint8 trace_flag = 0;		// 寻迹开关
int current_number = 0; // 扫描开始标志位
int trigflag_beep = 1;	// 蜂鸣器防误触
int outward_basin = 0;	// 出库标志位
uint8 avoid_move = 0;		// 避障开关
uint8 TOF_object = 0;		// 未检测到 1检测到障碍物 2检测障碍物后开始处理 3避障结束
int TOF_barrier[9] = {0};
int OUTP_flag = 0;		// 定时器启动计算 0不启用运动控制 1右出库 2左出库 3上坡直行
uint8 start_flag = 0; // 发车标志位
uint8 stop_flag = 0;	// 停车标志位
uint8 small_radiu = 0;

// 避障
float err_angle_rate = 0;
extern int outtoin_flag = 0;
float yaw_target = 0;		 // 避障角度
float out_distance = 18; // 避障距离
float in_distance = 14;	 // 回正距离
float out_angle = 65;		 // 避障目标角度
float in_angle = -35;		 // 回正目标角度
// 硅胶
//  float out_distance = 19; // 避障距离
//  float in_distance = 22;	 // 回正距离
//  float out_angle = 65;		 // 避障目标角度
//  float in_angle = -30;		 // 回正目标角度
//  库状态
int pack_mode = 0;
int strategy_mode = 0;

// 入库
float intop_angle = 80;		// 入库目标角度
float intop_distance = 15; // 入库距离
float intop_radiu = 0;		// 入库圆环
/***************************************************************************************/

// 出库
float outp_angle = 70;		 // 出库目标角度
float outp_distance = 13; // 入库距离
float outp_radiu = -22;		 // 入库圆环

