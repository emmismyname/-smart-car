#include "carcontrol.h"

// int running_mode = 1;		// 1是待机motor等于0 2是停车 3普通寻迹（计时器）
// uint8 trace_flag = 0;		// 寻迹开关
// int current_number = 0; // 扫描开始标志位
// int trigflag_beep = 1;	// 蜂鸣器防误触
// int outward_basin = 0;	// 出库标志位
// uint8 avoid_move = 0;		// 避障开关
// uint8 TOF_object = 0;		// 未检测到 1检测到障碍物 2检测障碍物后开始处理 3避障结束
// int TOF_barrier[9] = {0};
// int OUTP_flag = 0;		// 定时器启动计算 0不启用运动控制 1右出库 2左出库 3上坡直行
// uint8 start_flag = 0; // 发车标志位
// uint8 stop_flag = 0;	// 停车标志位
// uint8 small_radiu = 0;

// 避障
float err_angle_rate = 0;
extern int outtoin_flag = 0;
float yaw_target = 0;		 // 避障角度
float out_distance = 18; // 避障距离
float in_distance = 14;	 // 回正距离
float out_angle = 65;		 // 避障目标角度
float in_angle = -35;		 // 回正目标角度

//  库状态
int pack_mode = 0;
int strategy_mode = 0;

// // 入库
// float intop_angle = 80;		// 入库目标角度
// float intop_distance = 15; // 入库距离
// float intop_radiu = 0;		// 入库圆环
// /***************************************************************************************/

// // 出库
// float outp_angle = 70;		 // 出库目标角度
// float outp_distance = 13; // 入库距离
// float outp_radiu = -22;		 // 入库圆环
// /****************************************************************************************/


/**************************************江心一号************************************/
//发车
int16 route_distance=0;//定航距离
bit start_flag = 0; // 发车标志位
bit stop_flag = 0;	// 停车标志位



/****************************************************************************************/


//-------------------------------------------------------------------------------------------------------------------
//  @brief      运动模式切换
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void mode_choose(int move_mode) // 运动模式切换
{
	// int temp_counter=1;
	if (jiangxing_No_1.trace_flag == 0) // 完全停车
	{
		// 当霍尔检测到时候操作trace_flag==0停止寻迹
		jiangxing_No_1.car_speed = 0;
		jiangxing_No_1.left_target_speed = 0;
		jiangxing_No_1.right_target_speed = 0;
		motor_flag = 0;
	}
	else if (jiangxing_No_1.trace_flag == 1)
	{
		switch (move_mode)
		{
		case 1: // 待机
		{
		}
		break;
		case 2: // 停止pid
		{
			jiangxing_No_1.left_target_speed = 0;
			jiangxing_No_1.right_target_speed = 0;
		}
		break;
		case 3: // 直行pid
		{
			motor_flag = 1; // 电机输出
			jiangxing_No_1.left_target_speed = jiangxing_No_1.car_speed - PID_Loc_Control(&Forward_PID, get_error1) + z_gyro * gyro_d;
			jiangxing_No_1.right_target_speed = jiangxing_No_1.car_speed + PID_Loc_Control(&Forward_PID, get_error1) - z_gyro * gyro_d;
		}
		break;
		// case 4: // 弯道输出
		// {
		// 	motor_flag = 1; // 电机输出
		// 	Target_Speed_L = Target_speed - PID_Loc_Control(&Forward_PID, get_error1) - z_gyro * gyro_d;
		// 	Target_Speed_R = Target_speed + PID_Loc_Control(&Forward_PID, get_error1) + z_gyro * gyro_d;
		// }
		// break;
		// case 8: // 角度环调参
		// {
		// 	motor_flag = 1; // 电机输出
		// 	Target_Speed_L = Target_speed - PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
		// 	Target_Speed_R = Target_speed + PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
		// }
		// break;
		default:
			break;
		}
	}
}





//-------------------------------------------------------------------------------------------------------------------
//  @brief      元素识别
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void Element_Idef() // 元素识别
{
  
        if (jiangxing_No_1.trace_flag== 1) //	trace_flag==1;//寻迹标志位
        {   
					  // if(jiangxing_No_1.start_flag==1)
						// {
							enablemotor();
            	jiangxing_No_1.running_mode=3;
						// }

						if ((float)(SPI_float>= 500)) // 出线停车
						{
//							jiangxing_No_1.trace_flag = 0;
					    jiangxing_No_1.start_flag=0;
							jiangxing_No_1.stop_flag = 1;
							jiangxing_No_1.running_mode=2;
							jiangxing_No_1.motor_flag = 1;
							pid_all_init();
						}
        }
				if ((jiangxing_No_1.stop_flag == 1)) // 上位机停车
				{
					//jiangxing_No_1.trace_flag = 0;
					enablemotor();
					jiangxing_No_1.start_flag=0;
					jiangxing_No_1.stop_flag = 1;
					jiangxing_No_1.running_mode=2;
					pid_all_init();
				}
 
    mode_choose(jiangxing_No_1.running_mode);
}


void car_start()
{
 if(jiangxing_No_1.start_mode==0)//正常发车
 {
	pid_all_init();
	jiangxing_No_1.motor_flag=1;
	jiangxing_No_1.trace_flag=1;
	jiangxing_No_1.start_flag=1; 
	jiangxing_No_1.stop_flag=0;
	jiangxing_No_1.car_speed=100;
 }
}


// void car_start()
// {
//  pid_all_init();
//  jiangxing_No_1.motor_flag=1;
//  jiangxing_No_1.trace_flag=1;
//  jiangxing_No_1.start_flag=1; 
//  jiangxing_No_1.stop_flag=0;
//  jiangxing_No_1.car_speed=100;
// }



void car_strategy(uint8 s_mode)
{
  switch (s_mode)
	{
	case 0://正常发车
		;
	break;
	case 1:
		;
	break;
	case 2:
		;
	break;
	case 3:
		;
	break;
	default:
		break;
	}

}