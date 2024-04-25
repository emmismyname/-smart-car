#include "element.h"

/********************************元素识别变量******/
int8 Element_flag = 0; // 0是无元素 1是有元素
int8 Swerve_flag = 0;  // 0是直道 1是弯道
int16 UpSlope = 0;
int16 DownSlope_begin = 0;
int16 TurnLeft = 0, TurnRight = 0;
int16 Cross = 0;
/***************************************************************************************/

// /*******************环岛变量*******************/
int32 judge_delay_flag = 0;
int16 RoundAbout_small_left = 0;
int16 RoundAbout_small_right = 0;
int16 RoundAbout_big_left = 0;
int16 RoundAbout_big_right = 0;
// int16 Roundflag = 0;
// int16 roundcount = 0;
// /**********************************************/
/*******************环岛变量*******************/
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

// /********************************元素识别变量******/
int HRTRIG_flag = 1;
float distance_break = 3;    // 刹车距离
int8 block_flag = 0;         // 障碍标志
int8 block_count = 0;        // 判读次数
int16 blockcount_flag = 0;   // 判读标志位
int32 block_timer_end = 100; // 避障计时器
int8 blockcount_target = 0;  // 目标判读次数
int32 block_timer_count = 0; // 计时器计数
// /***************************************************************************************/
float pre_distance = 1000;
float ready_distance = 750;



// //-------------------------------------------------------------------------------------------------------------------
// // @brief		红外测距
// //  @param  返回以毫米为单位的范围读数
// //  @param
// //  @return  dl1a_get_distance
// //-------------------------------------------------------------------------------------------------------------------
void TOF_detection()
{
    dl1a_get_distance();
    if (Element_flag == 0 && UpSlope == 0 && Swerve_flag == 0 && outward_basin == 1)
    {
        //  int32 hadercounter=0;
        // 分辨
        if (dl1a_distance_mm > 1600 && avoid_move == 0 && TOF_object != 2) // 当数值属于8190时候属于没有检测到物体
        {
            TOF_object = 0; // 未检测到物体
        }
        if (pre_distance < dl1a_distance_mm && dl1a_distance_mm < pre_distance + 200 && avoid_move == 0 && TOF_object == 0 && avoid_move == 0) // 1200-1000之间预检测
        {
            TOF_object = 1; // 预检测检测到
//            Target_speed = 120;
        }
        else if (TOF_object == 1 && dl1a_distance_mm > 1600) // 未检测到障碍
        {
            TOF_object = 0; // 未检测到物体
            avoid_move = 0;
        }
        // else if (600 > dl1a_distance_mm && TOF_object == 1 && block_count == blockcount_target) // 600时候开始避障
        else if (ready_distance > dl1a_distance_mm && TOF_object == 1) // 700时候开始避障
        {
            err_angle_rate = 0;
//            pid_init(&Pout_rate_PID); // 初始化角度环内防止累计
//            pid_init(&Pout_PID);      // 初始化角度环内防止累计
//            Pout_rate_PID.pid_out = Forward_PID.pid_out;
            avoid_move = 1; // 避障开启
            TOF_object = 2; // 开始避障防止误判
//            distance_L = 0;
//            distance_R = 0;
//            yaw_output = 0;   // 初始化角度
            outtoin_flag = 0; // 初始化
        }
        // 计数策略
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
        } // 计数+1并且清除防误判标志位
    }
//    if (avoid_move == 1)
//    {
//        avoid_move = obstacle_avoidance(out_angle, in_angle, out_distance, in_distance);
//    }
}

////-------------------------------------------------------------------------------------------------------------------
//// @brief		霍尔检测
////  @param
////  @param
////  @return  将修改trace_flag寻迹开关
////-------------------------------------------------------------------------------------------------------------------
//void Hall_detection() // 霍尔检测
//{
//    if (HRTRIG_flag == 0 && isr_count_flag > isr_counter_start + 800)
//    {
//        if (outward_basin == 1)
//        {
//            yaw = 0;
//            outward_basin = 2; // 结束
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
//    } // 开始倒车
//    else if (outward_basin == 5)
//    {
//        running_mode = 0;
//        outward_basin = intop(intop_angle, intop_distance, intop_radiu);
//    }
//}