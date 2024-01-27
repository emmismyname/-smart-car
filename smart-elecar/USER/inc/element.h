#ifndef ELEMENT_H
#define ELEMENT_H

#include "carcontrol.h"
#include "imu.h"
#define BEEP P67
// ͣ������
#define HRTRIG P26

void TOF_detection(); // TOF���
void Element_Idef();  // Ԫ��ʶ��
void Hall_detection(); //�������

/********************************Ԫ��ʶ�����******/
extern int8 Element_flag; // 0����Ԫ�� 1����Ԫ��
extern int8 Swerve_flag;  // 0��ֱ�� 1�����
extern int16 UpSlope;
extern int16 DownSlope_begin;
extern int16 TurnLeft, TurnRight;
extern int16 Cross;
extern uint32 isr_counter_start;
/***************************************************************************************/

/*******************��������*******************/
// extern int32 judge_delay_flag;
// extern int16 RoundAbout_small_left;
// extern int16 RoundAbout_small_right;
// extern int16 RoundAbout_big_left;
// extern int16 RoundAbout_big_right;
// extern int16 Roundflag;
// extern int16 roundcount;
// /**********************************************/

/*******************��������*******************/
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

extern int8 block_flag;   // �ϰ���־
extern int8 block_count;  // �ж�����
extern int16 blockcount_flag; //�ж���־λ
extern int32 block_timer_end;  // ���ϼ�ʱ��
extern int8 blockcount_target;  // Ŀ���ж�����
extern int32 block_timer_count; //��ʱ������
extern uint32 isr_count_flag;
extern int OUTP_flag;        // ��ʱ��������������
extern int HRTRIG_flag;
extern float distance_break;//ɲ������
/////////����
extern float pre_distance;
extern float ready_distance;

#endif