#ifndef __GYRO_H_
#define __GYRO_H_
#define q30 1073741824.0f
#include "common.h"
#include "MPU6050.h"
//#include <math.h>
#include "DMP.h"
#define Zero_Offset_time 200 // 200次循环等待陀螺仪归零


extern float pitch, roll, yaw;
extern float pitch_result, roll_result, yaw_result; // 计算结果
extern int16 gyro_flag; // 0为零漂状态，1为正在计算标志，2为置0标志
// extern uint8 mpugyro_flag;

extern int Zero_Offset_Count;                             // 初始化累加值
// extern float y_gyro, z_gyro, x_gyro;                      // 初始化陀螺仪数值
extern float y_gyro_offset, z_gyro_offset, x_gyro_offset; // 初始化陀螺仪零漂数值
extern float y_acc, z_acc, x_acc;                         // 初始化加速度计数值
extern float y_acc_offset, z_acc_offset, x_acc_offset;    // 初始化陀螺仪零漂数值
extern float dt;                                          // 0.01系数
extern int Zero_Offset_End;                               // 100次
extern float yaw_err;
extern float det_yaw, yaw_last, yaw, yaw_last, yaw_circle; // 角度的变化梯度
extern float yaw_current;                                  // 初始化数值
extern float yaw_output;                                   // 输出值的上一状态数值
extern float yaw_result_last, org_yaw, gyro_yaw;           // 输出值的上一状态数值
extern float gyro_d;
extern int16 y_gyro , z_gyro , x_gyro ;			  // 初始化陀螺仪数值
extern uint8 mpu_flag; //mpu的工作标志位

void MPU_Acquire();
void MPU_Process();
void Zero_Offset_set(); // 初始化纠正陀螺仪
float gyro_aquire(float org_gyro);
void ZeroOffset_init();//零飘处理函数
void enableMPU();// MPU获取数据使能
void disableMPU();//标志位清零

#endif