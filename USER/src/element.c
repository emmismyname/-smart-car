#include "element.h"

/********************************Ԫ��ʶ�����******/
int8 Element_flag = 0; // 0����Ԫ�� 1����Ԫ��
int8 Swerve_flag = 0;  // 0��ֱ�� 1�����
int16 UpSlope = 0;
int16 DownSlope_begin = 0;
int16 TurnLeft = 0, TurnRight = 0;
int16 Cross = 0;
/***************************************************************************************/

// /*******************��������*******************/
int32 judge_delay_flag = 0;
int16 RoundAbout_small_left = 0;
int16 RoundAbout_small_right = 0;
int16 RoundAbout_big_left = 0;
int16 RoundAbout_big_right = 0;
// int16 Roundflag = 0;
// int16 roundcount = 0;
// /**********************************************/
/*******************��������*******************/
float pretoreadyL_round_distance = 11;
float pretoreadyR_round_distance = 20;
float leftround_diff = 15;
float rigtround_diff = 20;
float leftround_ready = 130;
float rigtround_ready = 140;
float leftround_yawtarget = 65;
float rigtround_yawtarget = -65;
float lmid_del = -20;
float rmid_del = -15;
float r2 = 130;
float l2 = 120;
int16 Round_mode = 3;
int16 Roundflag = 0;
int16 roundcount = 0;
int16 rightRound = 0;
int16 leftRound = 0;
int16 Round_mode;
/**********************************************/

// /********************************Ԫ��ʶ�����******/
int HRTRIG_flag = 1;
float distance_break = 3;    // ɲ������
int8 block_flag = 0;         // �ϰ���־
int8 block_count = 0;        // �ж�����
int16 blockcount_flag = 0;   // �ж���־λ
int32 block_timer_end = 100; // ���ϼ�ʱ��
int8 blockcount_target = 0;  // Ŀ���ж�����
int32 block_timer_count = 0; // ��ʱ������
// /***************************************************************************************/
float pre_distance = 1000;
float ready_distance = 750;



// //-------------------------------------------------------------------------------------------------------------------
// // @brief		������
// //  @param  �����Ժ���Ϊ��λ�ķ�Χ����
// //  @param
// //  @return  dl1a_get_distance
// //-------------------------------------------------------------------------------------------------------------------
void TOF_detection()
{
    dl1a_get_distance();
    if (Element_flag == 0 && UpSlope == 0 && Swerve_flag == 0 && outward_basin == 1)
    {
        //  int32 hadercounter=0;
        // �ֱ�
        if (dl1a_distance_mm > 1600 && avoid_move == 0 && TOF_object != 2) // ����ֵ����8190ʱ������û�м�⵽����
        {
            TOF_object = 0; // δ��⵽����
        }
        if (pre_distance < dl1a_distance_mm && dl1a_distance_mm < pre_distance + 200 && avoid_move == 0 && TOF_object == 0 && avoid_move == 0) // 1200-1000֮��Ԥ���
        {
            TOF_object = 1; // Ԥ����⵽
//            Target_speed = 120;
        }
        else if (TOF_object == 1 && dl1a_distance_mm > 1600) // δ��⵽�ϰ�
        {
            TOF_object = 0; // δ��⵽����
            avoid_move = 0;
        }
        // else if (600 > dl1a_distance_mm && TOF_object == 1 && block_count == blockcount_target) // 600ʱ��ʼ����
        else if (ready_distance > dl1a_distance_mm && TOF_object == 1) // 700ʱ��ʼ����
        {
            err_angle_rate = 0;
//            pid_init(&Pout_rate_PID); // ��ʼ���ǶȻ��ڷ�ֹ�ۼ�
//            pid_init(&Pout_PID);      // ��ʼ���ǶȻ��ڷ�ֹ�ۼ�
//            Pout_rate_PID.pid_out = Forward_PID.pid_out;
            avoid_move = 1; // ���Ͽ���
            TOF_object = 2; // ��ʼ���Ϸ�ֹ����
//            distance_L = 0;
//            distance_R = 0;
//            yaw_output = 0;   // ��ʼ���Ƕ�
            outtoin_flag = 0; // ��ʼ��
        }
        // ��������
        if (800 > dl1a_distance_mm && blockcount_flag == 0)
        {
            blockcount_flag = 1;
            block_timer_count = 0;
        }
        if (blockcount_flag == 1 && block_timer_count < block_timer_end)
        {
            block_timer_count++; // 20ms+1
        }
        else if (blockcount_flag == 1 && block_timer_count >= block_timer_end)
        {
            block_count = block_count + 1;
            blockcount_flag = 0;
            block_timer_count = 0;
        } // ����+1������������б�־λ
    }
//    if (avoid_move == 1)
//    {
//        avoid_move = obstacle_avoidance(out_angle, in_angle, out_distance, in_distance);
//    }
}

////-------------------------------------------------------------------------------------------------------------------
//// @brief		�������
////  @param
////  @param
////  @return  ���޸�trace_flagѰ������
////-------------------------------------------------------------------------------------------------------------------
//void Hall_detection() // �������
//{
//    if (HRTRIG_flag == 0 && isr_count_flag > isr_counter_start + 800)
//    {
//        if (outward_basin == 1)
//        {
//            yaw = 0;
//            outward_basin = 2; // ����
//            distance_L = 0;
//            distance_R = 0;
//        }
//    }
//    if (outward_basin == 2 && Current_Speed_L > 10 && Current_Speed_R > 10)
//    {
//        running_mode = 2;
//        mode_choose(running_mode);
//    }
//    else if (outward_basin == 2 && Current_Speed_L < 10 && Current_Speed_R < 10)
//    {
//        running_mode = 2;
//        mode_choose(running_mode);
//        pid_init(&Pout_rate_PID);
//        pid_init(&Pout_PID);
//        err_angle_rate = 0;
//        yaw_output = 0.0f;
//        distance_L = 0;
//        distance_R = 0;
//        outward_basin = 3;
//    }

//    if (Current_Speed_L<10 && Current_Speed_R < 10 && outward_basin == 3 && distance_L < -distance_break * 2000 && distance_L < -distance_break * 2000)
//    {
//        running_mode = 2;
//        mode_choose(running_mode);
//        outward_basin =4;
//    }
//    else if (outward_basin == 3 && distance_L > -distance_break * 2000 && distance_L > -distance_break * 2000)
//    {
//        Target_speed = -50;
//        yaw_target = 0;
//        running_mode = 8;
//        mode_choose(running_mode);
//    }
//    else if (Current_Speed_L <= 10 && Current_Speed_R <= 10 && outward_basin == 4)
//    {
//        outward_basin = 5;
//        pid_init(&Pout_rate_PID);
//        pid_init(&Pout_PID);
//        err_angle_rate = 0;
//        yaw_output = 0;
//    } // ��ʼ����
//    else if (outward_basin == 5)
//    {
//        running_mode = 0;
//        outward_basin = intop(intop_angle, intop_distance, intop_radiu);
//    }
//}