#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "common.h"
#include "zf_tim.h"
#include "zf_pwm.h"

// ������������
#define SPEEDR_PLUSE CTIM0_P34
#define SPEEDL_PLUSE CTIM3_P04
// ���巽������
#define SPEEDR_DIR P35
#define SPEEDL_DIR P53
// ���õ��
#define DIR_2 P64
#define DIR_1 P60
#define PWM_2 PWMA_CH4P_P66
#define PWM_1 PWMA_CH2P_P62

void counte_quire(); // ��ñ������������
void brushes_out();	 // ��ˢ�������
void motor_output(); // ������

typedef struct // ����pid�����������������
{
	float KP;	   // P ����ϵ��
	float KI;	   // I ����ϵ��
	float KD;	   // D ΢��ϵ��
	float i_limit; // �����޷�

	float p_out; // KP���
	float i_out; // KI���
	float d_out; // KD���

	float last_err;		  // �ϴ�ƫ��ֵ
	float lastBefore_err; // ���ϴ�ƫ��ֵ
	float err;			  // ƫ��ֵ

	float pid_out; // PID���������
} PID_P;

extern PID_P Speed_PID_L; // �����ٶ�pid
extern PID_P Speed_PID_R; // �����ٶ�pid

extern PID_P Swerve_PID;	  // ת�����pid
extern PID_P Forward_PID;	  // ֱ��PID����
extern PID_P roundaboutL_PID; // �󻷵�PID����
extern PID_P roundaboutR_PID; // �һ���PID����
extern PID_P Pout_PID;		  /// ������PID����
extern PID_P Pout_rate_PID; //���ٶ�PID����
extern PID_P climb_PID;		  /// �µ�PID����

extern uint8 brusheless_flag;  // ��ˢ����
extern uint16 brusheless_duty; // ��ˢ�����ֵ��ֵ��Χ��500-1000��Ӧռ�ձ�Ϊ0%-100%
extern uint8 motor_flag;

extern uint8 Dir_L;	  // 1��߷�����ת  0��ת
extern uint8 Dir_R;	  // 0�ұ߷�����ת  1��ת
extern uint8 Dir_car; // ����dir_car=1 L=1 R=0  ;dir_car=0 ���� L=0 R=1

extern float Target_Speed_L; // ����Ŀ���ٶ�
extern float Target_Speed_R; // ����Ŀ���ٶ�

extern float Current_Speed_L; // ���ֵ�ǰ����
extern float Current_Speed_R; // ���ֵ�ǰ����

extern int32 distance_L; // ����Ŀ���ٶ�
extern int32 distance_R; // ����Ŀ���ٶ�

extern float duty_L;	 // ����pidռ�ձ�
extern float duty_R;	 // �ҵ��pidռ�ձ�
extern float set_duty_L; // �������ռ�ձ�
extern float set_duty_R; // �ҵ�����ռ�ձ�

extern int templ_pluse;	 // �����������
extern int tempr_pluse;	 // �����������
extern int Target_speed; // ����Ŀ���ٶ�

// extern float PIDOUT_speed_L;//�����ٶȻ���ƫ�������
// extern float PIDOUT_speed_R;//�����ٶȻ���ƫ�������

// extern int32 Target_pluse_L=0;//����Ŀ��������Ŀ
// extern int32 Target_pluse_R=0;//����Ŀ��������Ŀ

// extern int32 Motor_pluse_L;//�����ٶȻ����������
// extern int32 Motor_pluse_R;//�����ٶȻ����������

// extern float Motor_duty_L=470;	//����Ŀ���ʼռ�ձ����ڿ˷�������ѹ����ԴΪ����״̬
// extern float Motor_duty_R=360;	//�ҵ��Ŀ���ʼռ�ձ����ڿ˷�������ѹ����ԴΪ����״̬
void pid_init(PID_P *pid);
//-------------------------------------------------------------------------------------------------------------------
//   @brief      pid����������ʼ��
//   @param      pid				pid����
//   @param      error			pid�������
//   @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_Control(PID_P *pid, float error);
//-------------------------------------------------------------------------------------------------------------------
//  @brief      pidλ��ʽ���������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_direction_Control(PID_P *pid, float error);
//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid����ʽ���������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Inc_Control(PID_P *pid, float error);
//-------------------------------------------------------------------------------------------------------------------
//  @brief      pidλ��ʽ���������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_motor_Control(PID_P *pid, float error);
//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid�Ƕȿ��������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_angle_Control(PID_P *pid, float error);

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid���ٶȿ��������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_angle_rate_Control(PID_P *pid, float error);
#endif