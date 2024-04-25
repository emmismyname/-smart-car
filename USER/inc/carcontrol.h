#ifndef _CARCONTROL_H_
#define _CARCONTROL__H_

#include "adc_acquire.h"

extern int display_switch; // ��ʾ���Ŀ���״̬ 1�� 0��
extern int button_switch;  // ��ʾ���Ŀ���״̬ 1�� 0��
extern uint8 start_flag;   // ������־λ
extern uint8 stop_flag;    // ͣ����־λ
/****************************�˶�״̬���********************************/
extern int running_mode;   // 1�Ǵ��� 2�Ƿ��� 3��ͨѰ������ʱ����
extern uint8 trace_flag;   // Ѱ������
extern int current_number; // ɨ�迪ʼ��־λ
extern int trigflag_beep;  // ����������
extern int outward_basin;  // �����־λ
extern uint8 avoid_move;   // ���Ͽ���
extern uint8 TOF_object;   // δ��⵽ 1��⵽�ϰ��� 2��⵽�µ�
extern int TOF_barrier[9];
extern int OUTP_flag; // ��ʱ���������� 0�������˶����� 1�ҳ��� 2����� 3����ֱ��
extern int err_speed;
extern uint8 small_radiu;//СԲ���뻷�뾶

//����
extern float err_angle_rate;
extern int outtoin_flag;
extern float yaw_target;   //���ϽǶ�
extern float out_distance;//���Ͼ���
extern float in_distance;//��������
extern float out_angle;//����Ŀ��Ƕ�
extern float in_angle;//����Ŀ��Ƕ�



//��״̬
extern int pack_mode;
extern int strategy_mode; 
//���
extern float intop_angle;//���Ŀ��Ƕ�
extern float intop_distance;//������
extern float intop_radiu;//���Բ��
/***************************************************************************************/

//����
extern float outp_angle;//����Ŀ��Ƕ�
extern float outp_distance;//������
extern float outp_radiu;//���Բ��
/***************************************************************************************/
extern float move_z_gyro;


// extern void outP_moving();              // �˶�����

// extern void left_outP();                // ���ҳ�����ƴ���
// extern void right_outP();               // ���ҳ�����ƴ���
extern void strategy_choose(int mode);
extern void pack_choose(int mode);
extern void mode_choose(int move_mode); // �˶�ģʽ�л�
extern int obstacle_avoidance(float out_angle1, float in_angle1,float target_out_distance1,float target_in_distance1); //���ϴ���
extern void outp(float target_angle,float target_distance,float target_radiu);
extern int8 intop(float target_angle,float target_distance,float target_radiu);
extern void work(int mode);


#endif