#include "element.h"

extern float base_speed;
extern void initall_PID();

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

//////

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Ԫ��ʶ��
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void Element_Idef() // Ԫ��ʶ��
{
    if (outward_basin == 1) // �����ѭ��
    {
        // �µ�
        if (Element_flag == 0 && UpSlope == 0 &&
            adc_2_M > 130 && dl1a_distance_mm < 500)
        {
            Target_speed = base_speed;
            Element_flag = 1;
            UpSlope = 1;
            BEEP = 1;
        }
        else if (Element_flag == 1 && UpSlope == 1 && adc_2_M < 100 && pitch > 2)
        {
            Target_speed = base_speed;
            Element_flag = 0;
            UpSlope = 0;
            BEEP = 0;
        }
        else if (UpSlope == 1 && pitch < -6)
        {
            Target_speed = base_speed;
            UpSlope = 2;
            Element_flag = 1;
        }
        else if (UpSlope == 2 && pitch > 0)
        {
            Target_speed = base_speed;
            UpSlope = 3;
            BEEP = 0;
        }
        else if (UpSlope == 3 && pitch > 12)
        {
            Target_speed = base_speed;
            UpSlope = 4;
            BEEP = 1;
        }
        else if (UpSlope == 4 && pitch < 10)
        {
            Target_speed = base_speed;
            UpSlope = 0;
            Element_flag = 0;
            BEEP = 0;
        }
        if (UpSlope != 0)
        {
            Target_speed = base_speed;
            running_mode = 9;
        }

        // // Ԥ�뻷�����һ���
        // if (Element_flag == 0)
        // {
        //     if (adc_2_M > 120 &&
        //         adc_2_L2 - adc_2_R2 > leftround_diff && dl1a_distance_mm > 1600 &&
        //         pitch > 0)
        //     {
        //         Target_speed = base_speed - 50;
        //         Element_flag = 1;
        //         leftRound = 1;
        //         yaw_output = 0;
        //         distance_L = 0;
        //         distance_R = 0;
        //     }
        //     else if (adc_2_M > 120 &&
        //              adc_2_R2 - adc_2_L2 > rigtround_diff  && dl1a_distance_mm > 1600 &&
        //              pitch > 0)
        //     {
        //         Target_speed = base_speed - 50;
        //         Element_flag = 1;
        //         rightRound = 1;
        //         yaw_output = 0;
        //         distance_L = 0;
        //         distance_R = 0;
        //     }
        // }
        // Ԥ�뻷�����һ���
        if (Element_flag == 0)
        {
            if (adc_M - adc_2_M < lmid_del && adc_2_M > 105 &&
                adc_2_L2 - adc_2_R2 > leftround_diff && dl1a_distance_mm > 1600 &&
                pitch > 0)
            {
                Target_speed = 100;
                Element_flag = 1;
                leftRound = 1;
                yaw_output = 0;
                distance_L = 0;
                distance_R = 0;
            }
            else if (adc_M - adc_2_M < rmid_del && adc_2_M > 105 &&
                     adc_2_R2 - adc_2_L2 > rigtround_diff && dl1a_distance_mm > 1600 &&
                     pitch > 0)
            {
                Target_speed = 100;
                Element_flag = 1;
                rightRound = 1;
                yaw_output = 0;
                distance_L = 0;
                distance_R = 0;
            }
        }
        // �µ������й���
        if (Element_flag == 1 && (leftRound || rightRound) && pitch < -6)
        {
            UpSlope = 1;
            leftRound = 0;
            rightRound = 0;
        }
        // ��״̬���
        if (Element_flag == 1 && leftRound == 1 &&
            adc_2_M < leftround_ready && adc_2_R2 < r2 &&
            distance_L > pretoreadyL_round_distance * 1000 && distance_R > pretoreadyL_round_distance * 1000)
        {
            leftRound = 2;
            err_angle_rate = 0;
            pid_init(&Pout_rate_PID);                    // ��ʼ���ǶȻ��ڷ�ֹ�ۼ�
            pid_init(&Pout_PID);                         // ��ʼ���ǶȻ��ڷ�ֹ�ۼ�
            Pout_rate_PID.pid_out = Forward_PID.pid_out; // �н�ֱ�����
        }
        else if (Element_flag == 1 && leftRound == 2 && yaw_output > leftround_yawtarget)
        {
            leftRound = 3;
            base_speed=300;
            roundaboutL_PID.pid_out = Pout_rate_PID.pid_out;
        }
        else if (Element_flag == 1 && leftRound == 3 && yaw_output > 340)
        {
            leftRound = 4;
            pid_init(&Forward_PID); // ��ʼ���ٶȻ��ڷ�ֹ�ۼ�
            Forward_PID.pid_out = roundaboutL_PID.pid_out;
        }
        else if (Element_flag == 1 && leftRound == 4 &&
                 adc_M < 100 && adc_2_M < 100)
        {
            Element_flag = 0;
            leftRound = 0;
            roundcount++;
            pid_init(&roundaboutL_PID); // ��ʼ���һ����ڷ�ֹ�ۼ�
        }

        // �һ�״̬���
        if (Element_flag == 1 && rightRound == 1 &&
            adc_2_M < rigtround_ready && adc_2_L2 < l2 &&
            distance_L > pretoreadyR_round_distance * 1000 && distance_R > pretoreadyR_round_distance * 1000)
        {
            rightRound = 2;
            err_angle_rate = 0;
            pid_init(&Pout_rate_PID);                    // ��ʼ���ǶȻ��ڷ�ֹ�ۼ�
            pid_init(&Pout_PID);                         // ��ʼ���ǶȻ��ڷ�ֹ�ۼ�
            Pout_rate_PID.pid_out = Forward_PID.pid_out; // �н�ֱ�����
        }
        else if (Element_flag == 1 && rightRound == 2 && yaw_output < rigtround_yawtarget)
        {
            rightRound = 3;
            base_speed=300;
            roundaboutR_PID.pid_out = Pout_rate_PID.pid_out;
        }
        else if (Element_flag == 1 && rightRound == 3 && yaw_output < -340)
        {
            rightRound = 4;
            pid_init(&Forward_PID); // ��ʼ���ٶȻ��ڷ�ֹ�ۼ�
            Forward_PID.pid_out = roundaboutR_PID.pid_out;
        }
        else if (Element_flag == 1 && rightRound == 4 &&
                 adc_M < 100 && adc_2_M < 100)
        {
            Element_flag = 0;
            rightRound = 0;
            roundcount++;
            pid_init(&roundaboutR_PID); // ��ʼ���һ����ڷ�ֹ�ۼ�
        }

        // //////////////////////////////////
        // // ����ģʽת��
        // if (leftRound == 1)
        // {
        //     running_mode = 7;
        // } // ����
        // else if (leftRound == 2)
        // {
        //     running_mode = 16;
        // } // ���뻷
        // else if (leftRound == 3)
        // {
        //     running_mode = 6;
        //     // running_mode = 2;
        // } // �󻷻���
        // else if (leftRound == 4)
        // {
        //     Target_speed = base_speed;
        //     running_mode = 3;
        // } // ����

        if (rightRound == 1)
        {
            running_mode = 7;
        } // ����
        else if (rightRound == 2)
        {
            yaw_target = rigtround_yawtarget;
            running_mode = 15;
        } // �һ��뻷
        else if (rightRound == 3)
        {
            Target_speed = base_speed;
            running_mode = 5;
            //  running_mode = 2;
        } // �һ�����
        else if (rightRound == 4)
        {
            running_mode = 3;
        } // ����

        if (leftRound == 1)
        {
            running_mode = 7;
        } // ����
        else if (leftRound == 2)
        {
            yaw_target = leftround_yawtarget;
            running_mode = 16;
        } // �һ��뻷
        else if (leftRound == 3)
        {
            Target_speed = base_speed;
            running_mode = 6;
            //  running_mode = 2;
        } // �󻷻���
        else if (leftRound == 4)
        {
            running_mode = 3;
        } // ����

        // // ״̬1�����˶�״̬ С��+С��
        // if (Round_mode == 1 && roundcount == 0)
        // {
        //     //���Ϊ��
        //     if (Element_flag == 1 && leftRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        //     // ���Ϊ�һ�
        //     if (Element_flag == 1 && rightRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        // }
        // else if (Round_mode == 1 && roundcount == 1)
        // {
        //     //���Ϊ��
        //     if (Element_flag == 1 && leftRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 3)

        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        //     // ���Ϊ�һ�
        //     if (Element_flag == 1 && rightRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        // }
        // // ״̬2�����˶�״̬ С��+��
        // if (Round_mode == 2 && roundcount == 0)
        // {
        //     //���Ϊ��
        //     if (Element_flag == 1 && leftRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        //     // ���Ϊ�һ�
        //     if (Element_flag == 1 && rightRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        // }
        // else if (Round_mode == 2 && roundcount == 1)
        // {
        //     //���Ϊ��
        //     if (Element_flag == 1 && leftRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        //     // ���Ϊ�һ�
        //     if (Element_flag == 1 && rightRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        // }

        // // ״̬3�����˶�״̬ ��+С��
        // if (Round_mode == 3 && roundcount == 0)
        // {
        //     //���Ϊ��
        //     if (Element_flag == 1 && leftRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        //     // ���Ϊ�һ�
        //     if (Element_flag == 1 && rightRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        // }
        // else if (Round_mode == 3 && roundcount == 1)
        // {
        //     //���Ϊ��
        //     if (Element_flag == 1 && leftRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        //     // ���Ϊ�һ�
        //     if (Element_flag == 1 && rightRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        // }

        // // ״̬4�����˶�״̬ ��+��
        // if (Round_mode == 4 && roundcount == 0)
        // {
        //     //���Ϊ��
        //     if (Element_flag == 1 && leftRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        //     // ���Ϊ�һ�
        //     if (Element_flag == 1 && rightRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        // }
        // else if (Round_mode == 4 && roundcount == 1)
        // {
        //     //���Ϊ��
        //     if (Element_flag == 1 && leftRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && leftRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        //     // ���Ϊ�һ�
        //     if (Element_flag == 1 && rightRound == 1)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 2)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 3)
        //     {
        //         running_mode = 2;
        //     }
        //     else if (Element_flag == 1 && rightRound == 4)
        //     {
        //         running_mode = 2;
        //     }
        // }
        if (trace_flag == 1 && Element_flag == 0 && UpSlope == 0 && avoid_move == 0) //	trace_flag==1;//Ѱ����־λ
        {
            if (Swerve_flag == 1) // ���
            {
                Target_speed = base_speed - 30;
                running_mode = 4;
                // running_mode = 2;
                BEEP = 1;
                Forward_PID.pid_out = Swerve_PID.pid_out;
            }
            else // ֱ��
            {
                Target_speed = base_speed + 30;
                running_mode = 3;
                // running_mode = 2;
                BEEP = 0;
                Swerve_PID.pid_out = Forward_PID.pid_out;
            }
        }

        if (((float)(adc[1].data_0 + adc[2].data_0 + adc[4].data_0 + adc[5].data_0 + adc[6].data_0 + adc[7].data_0 + adc[9].data_0 + adc[10].data_0 <= 500)) && avoid_move == 0) // ����ͣ��
        {
            trace_flag = 0;
            motor_flag = 0;
            button_switch = 1;
            display_switch = 1;
            initall_PID();
            BEEP = 0;
        }
        if ((stop_flag == 1)) // ��λ��ͣ��
        {
            trace_flag = 0;
            motor_flag = 0;
            button_switch = 1;
            display_switch = 1;
            initall_PID();
            BEEP = 0;
        }
        mode_choose(running_mode);
    }
}

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
            Target_speed = 120;
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
            pid_init(&Pout_rate_PID); // ��ʼ���ǶȻ��ڷ�ֹ�ۼ�
            pid_init(&Pout_PID);      // ��ʼ���ǶȻ��ڷ�ֹ�ۼ�
            Pout_rate_PID.pid_out = Forward_PID.pid_out;
            avoid_move = 1; // ���Ͽ���
            TOF_object = 2; // ��ʼ���Ϸ�ֹ����
            distance_L = 0;
            distance_R = 0;
            yaw_output = 0;   // ��ʼ���Ƕ�
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
    if (avoid_move == 1)
    {
        avoid_move = obstacle_avoidance(out_angle, in_angle, out_distance, in_distance);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		�������
//  @param
//  @param
//  @return  ���޸�trace_flagѰ������
//-------------------------------------------------------------------------------------------------------------------
void Hall_detection() // �������
{
    if (HRTRIG_flag == 0 && isr_count_flag > isr_counter_start + 800)
    {
        if (outward_basin == 1)
        {
            yaw = 0;
            outward_basin = 2; // ����
            distance_L = 0;
            distance_R = 0;
        }
    }
    if (outward_basin == 2 && Current_Speed_L > 10 && Current_Speed_R > 10)
    {
        running_mode = 2;
        mode_choose(running_mode);
    }
    else if (outward_basin == 2 && Current_Speed_L < 10 && Current_Speed_R < 10)
    {
        running_mode = 2;
        mode_choose(running_mode);
        pid_init(&Pout_rate_PID);
        pid_init(&Pout_PID);
        err_angle_rate = 0;
        yaw_output = 0.0f;
        distance_L = 0;
        distance_R = 0;
        outward_basin = 3;
    }

    if (Current_Speed_L<10 && Current_Speed_R < 10 && outward_basin == 3 && distance_L < -distance_break * 2000 && distance_L < -distance_break * 2000)
    {
        running_mode = 2;
        mode_choose(running_mode);
        outward_basin =4;
    }
    else if (outward_basin == 3 && distance_L > -distance_break * 2000 && distance_L > -distance_break * 2000)
    {
        Target_speed = -50;
        yaw_target = 0;
        running_mode = 8;
        mode_choose(running_mode);
    }
    else if (Current_Speed_L <= 10 && Current_Speed_R <= 10 && outward_basin == 4)
    {
        outward_basin = 5;
        pid_init(&Pout_rate_PID);
        pid_init(&Pout_PID);
        err_angle_rate = 0;
        yaw_output = 0;
    } // ��ʼ����
    else if (outward_basin == 5)
    {
        running_mode = 0;
        outward_basin = intop(intop_angle, intop_distance, intop_radiu);
    }
}
// //-------------------------------------------------------------------------------------------------------------------
// // @brief		�������
// //  @param
// //  @param
// //  @return  ���޸�trace_flagѰ������
// //-------------------------------------------------------------------------------------------------------------------
// void Hall_detection() // �������
// {
//     HRTRIG_flag = HRTRIG;
//     if (HRTRIG == 0 && isr_count_flag > isr_counter_start + 800)
//     {
//         if (outward_basin == 1)
//         {
//             yaw = 0;
//             outward_basin = 2; // ����
//             distance_L = 0;
//             distance_R = 0;
//         }
//     }
//     if (outward_basin == 2 && distance_L > distance_break * 1000 && distance_R > distance_break * 1000)
//     {
//         running_mode = 2;
//         mode_choose(running_mode);
//         yaw_output = 0.0f;
//         distance_L = 0;
//         distance_R = 0;
//         outward_basin = 3;
//     }
//     if (Current_Speed_L < 10 && Current_Speed_R > 10 && outward_basin == 3)
//     {
//         running_mode = 2;
//         mode_choose(running_mode);
//     }
//     else if (Current_Speed_L <= 10 && Current_Speed_R <= 10 && outward_basin == 3)
//     {
//         outward_basin = 4;
//     } // ��ʼ����
//     else if (outward_basin == 4)
//     {
//         running_mode = 0;
//         Target_speed = 100;
//         outward_basin = intop(intop_angle, intop_distance, intop_radiu);
//     }
// }