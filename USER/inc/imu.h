#ifndef __GYRO_H_
#define __GYRO_H_
#define q30 1073741824.0f
#include "common.h"
#include "MPU6050.h"
//#include <math.h>
#include "DMP.h"
#define Zero_Offset_time 200 // 200��ѭ���ȴ������ǹ���


extern float pitch, roll, yaw;
extern float pitch_result, roll_result, yaw_result; // ������
extern int16 gyro_flag; // 0Ϊ��Ư״̬��1Ϊ���ڼ����־��2Ϊ��0��־
// extern uint8 mpugyro_flag;

extern int Zero_Offset_Count;                             // ��ʼ���ۼ�ֵ
// extern float y_gyro, z_gyro, x_gyro;                      // ��ʼ����������ֵ
extern float y_gyro_offset, z_gyro_offset, x_gyro_offset; // ��ʼ����������Ư��ֵ
extern float y_acc, z_acc, x_acc;                         // ��ʼ�����ٶȼ���ֵ
extern float y_acc_offset, z_acc_offset, x_acc_offset;    // ��ʼ����������Ư��ֵ
extern float dt;                                          // 0.01ϵ��
extern int Zero_Offset_End;                               // 100��
extern float yaw_err;
extern float det_yaw, yaw_last, yaw, yaw_last, yaw_circle; // �Ƕȵı仯�ݶ�
extern float yaw_current;                                  // ��ʼ����ֵ
extern float yaw_output;                                   // ���ֵ����һ״̬��ֵ
extern float yaw_result_last, org_yaw, gyro_yaw;           // ���ֵ����һ״̬��ֵ
extern float gyro_d;
extern int16 y_gyro , z_gyro , x_gyro ;			  // ��ʼ����������ֵ
extern uint8 mpu_flag; //mpu�Ĺ�����־λ

void MPU_Acquire();
void MPU_Process();
void Zero_Offset_set(); // ��ʼ������������
float gyro_aquire(float org_gyro);
void ZeroOffset_init();//��Ʈ������
void enableMPU();// MPU��ȡ����ʹ��
void disableMPU();//��־λ����

#endif