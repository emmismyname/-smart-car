#include "element.h"

extern float base_speed;
extern void initall_PID();

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

//////

//-------------------------------------------------------------------------------------------------------------------
//  @brief      元素识别
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void Element_Idef() // 元素识别
{
  
        if (trace_flag == 1) //	trace_flag==1;//寻迹标志位
        {
            if (adc_2_M > 80 && adc_2_M < 110 && adc_M < 80&&Swerve_flag==0)
            {
                Swerve_flag = 1;
                Target_speed = base_speed*0.8;
                running_mode = Forward_carmode;
            }
            else if (adc_2_M > 80 && adc_2_M < 110 && adc_M > 80 && adc_M < 110&&Swerve_flag==1)
            {
                Swerve_flag = 0;
                Target_speed = base_speed ;
                running_mode = Forward_carmode;
                pid_init(&Swerve_PID);
                //  BEEP = 0;
            }
        }

        if ((float)(adc[1].data_0 + adc[2].data_0 + adc[4].data_0 + adc[5].data_0 + adc[6].data_0 + adc[7].data_0 + adc[9].data_0 + adc[10].data_0 <= 500)) // 出线停车
        // if (((float)(adc[1].data_0 + adc[2].data_0 + adc[4].data_0 + adc[5].data_0 + adc[6].data_0 + adc[7].data_0 + adc[9].data_0 + adc[10].data_0 <= 500)) && avoid_move == 0) // 出线停车
        {
            trace_flag = 0;
            motor_flag = 0;
            button_switch = 1;
            display_switch = 1;
            initall_PID();
            BEEP = 0;
        }
        if ((jiangxing_No_1.stop_flag == 1)) // 上位机停车
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
            Target_speed = 120;
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
            pid_init(&Pout_rate_PID); // 初始化角度环内防止累计
            pid_init(&Pout_PID);      // 初始化角度环内防止累计
            Pout_rate_PID.pid_out = Forward_PID.pid_out;
            avoid_move = 1; // 避障开启
            TOF_object = 2; // 开始避障防止误判
            distance_L = 0;
            distance_R = 0;
            yaw_output = 0;   // 初始化角度
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
    if (avoid_move == 1)
    {
        avoid_move = obstacle_avoidance(out_angle, in_angle, out_distance, in_distance);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		霍尔检测
//  @param
//  @param
//  @return  将修改trace_flag寻迹开关
//-------------------------------------------------------------------------------------------------------------------
void Hall_detection() // 霍尔检测
{
}
