#include "carcontrol.h"

int err_speed = 0;
int running_mode = 1;		// 1�Ǵ���motor����0 2��ͣ�� 3��ͨѰ������ʱ����
uint8 trace_flag = 0;		// Ѱ������
int current_number = 0; // ɨ�迪ʼ��־λ
int trigflag_beep = 1;	// ����������
int outward_basin = 0;	// �����־λ
uint8 avoid_move = 0;		// ���Ͽ���
uint8 TOF_object = 0;		// δ��⵽ 1��⵽�ϰ��� 2����ϰ����ʼ���� 3���Ͻ���
int TOF_barrier[9] = {0};
int OUTP_flag = 0;		// ��ʱ���������� 0�������˶����� 1�ҳ��� 2����� 3����ֱ��
uint8 start_flag = 0; // ������־λ
uint8 stop_flag = 0;	// ͣ����־λ
uint8 small_radiu = 0;

// ����
float err_angle_rate = 0;
extern int outtoin_flag = 0;
float yaw_target = 0;		 // ���ϽǶ�
float out_distance = 18; // ���Ͼ���
float in_distance = 14;	 // ��������
float out_angle = 65;		 // ����Ŀ��Ƕ�
float in_angle = -35;		 // ����Ŀ��Ƕ�
// �轺
//  float out_distance = 19; // ���Ͼ���
//  float in_distance = 22;	 // ��������
//  float out_angle = 65;		 // ����Ŀ��Ƕ�
//  float in_angle = -30;		 // ����Ŀ��Ƕ�
//  ��״̬
int pack_mode = 0;
int strategy_mode = 0;

// ���
float intop_angle = 80;		// ���Ŀ��Ƕ�
float intop_distance = 15; // ������
float intop_radiu = 0;		// ���Բ��
/***************************************************************************************/

// ����
float outp_angle = 70;		 // ����Ŀ��Ƕ�
float outp_distance = 13; // ������
float outp_radiu = -22;		 // ���Բ��

