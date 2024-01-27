#include "carcontrol.h"
int err_speed = 0;
int running_mode = 1;		// 1是待机motor等于0 2是停车 3普通寻迹（计时器）
uint8 trace_flag = 0;		// 寻迹开关
int current_number = 0; // 扫描开始标志位
int trigflag_beep = 1;	// 蜂鸣器防误触
int outward_basin = 0;	// 出库标志位
uint8 avoid_move = 0;		// 避障开关
uint8 TOF_object = 0;		// 未检测到 1检测到障碍物 2检测障碍物后开始处理 3避障结束
int TOF_barrier[9] = {0};
int OUTP_flag = 0;		// 定时器启动计算 0不启用运动控制 1右出库 2左出库 3上坡直行
uint8 start_flag = 0; // 发车标志位
uint8 stop_flag = 0;	// 停车标志位
// void right_outP()//向右出库出库控制代码
// {
// 	//int number=0;
// 	if(outward_basin==0)//没出库
// 	{
// 		brusheless_flag=1;
// 		brusheless_duty=700;
// 		delay_ms(1500);
// 		//控制函数
// 		pid_init(&Speed_PID_L);
// 		pid_init(&Speed_PID_R);
// 		//PID_all_set();
// 		motor_flag=1;//电机输出
// 		distance_L=0;
// 		distance_R=0;             //出库
// 		while((distance_L<14000)&&(distance_R<10000))
// 		{
// 			gyro_flag=1;
// 			OUTP_flag=1;
// 			motor_flag=1;
// 			imu660ra_get_gyro();//获取陀螺仪数据
// 			wireless_send(1,5);
// 			// ips114_showstr(15, 2, "distance_L");		ips114_showfloat(8*16, 2, distance_L,5,2);
// 		  // ips114_showstr(15, 3, "distance_R");		ips114_showfloat(8*16, 3, distance_R,4,2);
// 		  // ips114_showstr(15, 4, "Tgt_Spd_L=");		ips114_showfloat(8*16, 4, Target_Speed_L,3,3);
// 		  // ips114_showstr(15, 5, "Tgt_Spd_R=");		ips114_showfloat(8*16, 5, Target_Speed_R,3,3);
// 			// ips114_showstr(15, 6, "yaw");		ips114_showfloat(8*16, 6, yaw,4,2);
// 		}
// 		button_switch=0;
// 		display_switch=0;
// 		gyro_flag=0;
// 		yaw=0.0f;
// 		pitch=0.0f;
// 		motor_flag=0;
// 		outward_basin=1;
// 		trace_flag=1;
// 		OUTP_flag=0;
// 		Target_speed=80;
// 	}
// }
// void left_outP()//向左出库控制代码
// {
// // 	int number=0;
// 	if(outward_basin==0)//没出库
// 	{
// 		brusheless_flag=1;
// 		brusheless_duty=700;
// 		delay_ms(1500);
// 		//控制函数
// 		pid_init(&Speed_PID_L);
// 		pid_init(&Speed_PID_R);
// 		//PID_all_set();
// 		motor_flag=1;//电机输出
// 		distance_L=0;
// 		distance_R=0;             //出库
// 		while((distance_L<10000)&&(distance_R<14000))
// 		{
// 			gyro_flag=1;
// 			OUTP_flag=2;
// 			motor_flag=1;
// 			imu660ra_get_gyro();//获取陀螺仪数据
// 			wireless_send(1,5);
// 			ips114_showstr(15, 2, "distance_L");		ips114_showfloat(8*16, 2, distance_L,5,2);
// 		  ips114_showstr(15, 3, "distance_R");		ips114_showfloat(8*16, 3, distance_R,4,2);
// 		  ips114_showstr(15, 4, "Tgt_Spd_L=");		ips114_showfloat(8*16, 4, Target_Speed_L,3,3);
// 		  ips114_showstr(15, 5, "Tgt_Spd_R=");		ips114_showfloat(8*16, 5, Target_Speed_R,3,3);
// 			ips114_showstr(15, 6, "yaw");		ips114_showfloat(8*16, 6, yaw,4,2);
// 		}
// 		button_switch=0;
// 		display_switch=0;
// 		gyro_flag=0;
// 		yaw=0.0f;
// 		pitch=0.0f;
// 		motor_flag=0;
// 		outward_basin=1;
// 		trace_flag=1;
// 		OUTP_flag=0;
// 		Target_speed=80;
// 	}
// }
// void outP_moving()//运动代码
// {
// 	switch (OUTP_flag)
// 	{
// 	  case 1:
// 		  {
//         angle_err=1100-yaw;//左是负，右是正
// 				Target_speed=100;
// 				Target_Speed_L=Target_speed+PID_Loc_Control(&Pout_PID,angle_err);
// 	   	 	Target_Speed_R=Target_speed-PID_Loc_Control(&Pout_PID,angle_err);
// 		  }
// 		 	break;
//      case 2:
// 		  {
// 				angle_err=-1100-yaw;//左是负，右是正
// 				Target_speed=100;
// 				Target_Speed_L=Target_speed+PID_Loc_Control(&Pout_PID,angle_err);
// 				Target_Speed_R=Target_speed-PID_Loc_Control(&Pout_PID,angle_err);
// 			}
// 			 break;
// 			case 3://上坡保持直行
// 		  {
// 				angle_err=0-yaw;//左是负，右是正
// 				Target_speed=100;
// 				Target_Speed_L=Target_speed+PID_Loc_Control(&Pout_PID,angle_err);
// 				Target_Speed_R=Target_speed-PID_Loc_Control(&Pout_PID,angle_err);
// 			}
// 			 break;
// 			case 4://上坡保持直行
// 		  {
// 				angle_err=-90-yaw;//左是负，右是正
// 				Target_speed=100;
// 				Target_Speed_L=Target_speed-PID_Loc_Control(&Pout_PID,angle_err);
// 				Target_Speed_R=Target_speed+PID_Loc_Control(&Pout_PID,angle_err);
// 			}
// 			 break;
// 			case 5://左环岛航向角引入
// 			 {
// 				angle_err=-660;//左是负，右是正
// 				Target_speed=60;
// 				Target_Speed_L=Target_speed+PID_Loc_Control(&Pout_PID,angle_err);
// 				Target_Speed_R=Target_speed-PID_Loc_Control(&Pout_PID,angle_err);
// 			 }
// 			break;
// 			 	case 6://右环岛航向角引入
// 			 {
// 				angle_err=-90;//左是负，右是正
// 				Target_speed=100;
// 				Target_Speed_L=Target_speed+PID_Loc_Control(&Pout_PID,angle_err);
// 				Target_Speed_R=Target_speed-PID_Loc_Control(&Pout_PID,angle_err);
// 			 }
// 			 	break;
// 	   case 7:
// 		  {
//         angle_err=-400-yaw;//左是负，右是正
// 				Target_Speed_L=Target_speed+PID_Loc_Control(&Pout_PID,angle_err);
// 	   	 	Target_Speed_R=Target_speed-PID_Loc_Control(&Pout_PID,angle_err);
// 		  }
// 			break;
// 			case 8:
// 		  {
//         angle_err=400;//左是负，右是正
// 				Target_Speed_L=Target_speed+PID_Loc_Control(&Pout_PID,angle_err);
// 	   	 	Target_Speed_R=Target_speed-PID_Loc_Control(&Pout_PID,angle_err);
// 		  }
// 		 	break;
// 	  default: break;
// 	}
// }
uint8 small_radiu = 0;

// 避障
float err_angle_rate = 0;
extern int outtoin_flag = 0;
float yaw_target = 0;		 // 避障角度
float out_distance = 18; // 避障距离
float in_distance = 14;	 // 回正距离
float out_angle = 65;		 // 避障目标角度
float in_angle = -35;		 // 回正目标角度
// 硅胶
//  float out_distance = 19; // 避障距离
//  float in_distance = 22;	 // 回正距离
//  float out_angle = 65;		 // 避障目标角度
//  float in_angle = -30;		 // 回正目标角度
//  库状态
int pack_mode = 0;
int strategy_mode = 0;

// 入库
float intop_angle = 80;		// 入库目标角度
float intop_distance = 15; // 入库距离
float intop_radiu = 0;		// 入库圆环
/***************************************************************************************/

// 出库
float outp_angle = 70;		 // 出库目标角度
float outp_distance = 13; // 入库距离
float outp_radiu = -22;		 // 入库圆环
/*************************s**************************************************************/

//-------------------------------------------------------------------------------------------------------------------
//  @brief      运动模式切换
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void mode_choose(int move_mode) // 运动模式切换
{
	// int temp_counter=1;
	if (trace_flag == 0) // 完全停车
	{
		// 当霍尔检测到时候操作trace_flag==0停止寻迹
		Target_speed = 0;
		Target_Speed_L = 0;
		Target_Speed_R = 0;
		motor_flag = 0;
	}
	else if (trace_flag == 1)
	{
		switch (move_mode)
		{
		case 1: // 待机
		{
		}
		break;
		case 2: // 停止pid
		{
			Target_Speed_L = 0;
			Target_Speed_R = 0;
		}
		break;
		case 3: // 直行pid
		{
			motor_flag = 1; // 电机输出
			Target_Speed_L = Target_speed - PID_Loc_Control(&Forward_PID, err) - z_gyro * gyro_d;
			Target_Speed_R = Target_speed + PID_Loc_Control(&Forward_PID, err) + z_gyro * gyro_d;
		}
		break;
		case 4: // 弯道输出
		{
			motor_flag = 1; // 电机输出
			Target_Speed_L = Target_speed - PID_Loc_Control(&Swerve_PID, err) - z_gyro * gyro_d * 0.7;
			Target_Speed_R = Target_speed + PID_Loc_Control(&Swerve_PID, err) + z_gyro * gyro_d * 0.7;
		}
		break;
		case 5: // 环岛环参右环道
		{
			motor_flag = 1; // 电机输出
			Target_Speed_L = Target_speed - PID_Loc_Control(&roundaboutR_PID, err) - z_gyro * gyro_d * 0.7;
			Target_Speed_R = Target_speed + PID_Loc_Control(&roundaboutR_PID, err) + z_gyro * gyro_d * 0.7;
		}
		case 6: // 环岛环参左环道
		{
			motor_flag = 1; // 电机输出
			Target_Speed_L = Target_speed - PID_Loc_Control(&roundaboutL_PID, err) - z_gyro * gyro_d * 0.7;
			Target_Speed_R = Target_speed + PID_Loc_Control(&roundaboutL_PID, err) + z_gyro * gyro_d * 0.7;
		}
		break;
		case 7: // 入环出环参数
		{
			motor_flag = 1; // 电机输出
			Target_Speed_L = Target_speed - PID_Loc_Control(&Forward_PID, err) - z_gyro * gyro_d * 1.2;
			Target_Speed_R = Target_speed + PID_Loc_Control(&Forward_PID, err) + z_gyro * gyro_d * 1.2;
		}
		case 8: // 角度环调参
		{
			motor_flag = 1; // 电机输出
			Target_Speed_L = Target_speed - PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
			Target_Speed_R = Target_speed + PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
		}
		break;
		case 9: // 上坡调参
		{
			motor_flag = 1; // 电机输出
			Target_Speed_L = Target_speed - PID_Loc_Control(&Forward_PID, err) - z_gyro * gyro_d * 1.4;
			Target_Speed_R = Target_speed + PID_Loc_Control(&Forward_PID, err) + z_gyro * gyro_d * 1.4;
		}
		break;
		case 15: // 环岛右入环引环
		{
			motor_flag = 1; // 电机输出
			Target_Speed_L = Target_speed - PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
			Target_Speed_R = Target_speed + PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
			// Target_Speed_L = Target_speed - PID_Loc_Control(&roundaboutR_PID, Err_Inclined_right) - z_gyro * gyro_d * 0.7;
			// Target_Speed_R = Target_speed + PID_Loc_Control(&roundaboutR_PID, Err_Inclined_right) + z_gyro * gyro_d * 0.7;
		}
		break;
		case 16: // 环岛左入环引环
		{
			motor_flag = 1; // 电机输出
			Target_Speed_L = Target_speed - PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
			Target_Speed_R = Target_speed + PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
			// err_speed = small_radiu * Target_speed * 0.175;
			// Target_Speed_L = Target_speed - PID_Loc_Control(&roundaboutL_PID, Err_Inclined_left) - z_gyro * gyro_d * 0.7;
			// Target_Speed_R = Target_speed + PID_Loc_Control(&roundaboutL_PID, Err_Inclined_left) + z_gyro * gyro_d * 0.7;
		}
		break;
		default:
			break;
		}
	}
}
void pack_choose(int mode)
{
	switch (mode)
	{
	case 0:
	{
		intop_angle = 0;
		intop_distance = 0;
		intop_radiu = 0;
		outp_angle = 0;
		outp_distance = 0;
		outp_radiu = 0;
	}
	break;
	case 1: // 左出左后
	{
		distance_break = 3;
		intop_angle = -80;
		intop_distance = 16;
		intop_radiu = 50;
		// 出库
		outp_angle = 70;
		outp_distance = 13;
		outp_radiu = -22;
	}
	break;
	case 2: // 右出右后
	{
// 		float intop_angle = 80;		// 入库目标角度
// float intop_distance = 15; // 入库距离
		distance_break = 0.5;
		intop_angle = 80;
		intop_distance = 15;
		distance_break = 3;
		// intop_radiu = -15;
		// 出库
		outp_angle = -70;
		outp_distance = 13;
		outp_radiu = 26;
		// distance_break = 14;
		// intop_angle = 70;
		// intop_distance = 70;
		// intop_radiu = -15;
		// // 出库
		// outp_angle = -70;
		// outp_distance = 13;
		// outp_radiu = 26;
	}
	break;

	default:
		break;
	}
}
void strategy_choose(int mode)
{
	switch (mode)
	{
	case 0:
		in_angle = 0;
		in_distance = 0;
		out_angle = 0;
		out_distance = 0;
		break;
	case 1: // 避障
		in_angle = -60;
		in_distance = 50;
		out_angle = 30;
		out_distance = 70;
		break;
	case 2: // 右出右后
		in_angle = -60;
		in_distance = 50;
		out_angle = 30;
		out_distance = 70;
		break;

	default:
		break;
	}
} // void obstacle_avoidance(float out_distance,float in_distance,float out_angle,float in_angle) //避障处理
void work(int mode)
{
	switch (mode)
	{
	case 0:
	  base_speed=220;
		pretoreadyL_round_distance = 11;
		pretoreadyR_round_distance = 20;
		leftround_diff = 15;
		rigtround_diff = 20;
		leftround_ready = 130;
		rigtround_ready = 140;
		leftround_yawtarget = 65;
		rigtround_yawtarget = -65;
		lmid_del = -20;
		rmid_del = -15;
		break;

	default:
		break;
	}
}

//{

//}
int obstacle_avoidance(float out_angle1, float in_angle1, float target_out_distance1, float target_in_distance1) // 避障处理
{
	int res = 1;
	if (distance_L + distance_R < 2000 * target_out_distance1)
	{
		running_mode = 8;
		Target_speed = base_speed - 20;
		motor_flag = 1; // 电机输出
		yaw_target = out_angle1;
		mode_choose(running_mode);
	}
	else if (distance_L + distance_R > 2000 * target_out_distance1)
	{
		outtoin_flag = 1;
	}
	if (outtoin_flag == 1 && distance_L + distance_R < 2000 * (target_out_distance1 + target_in_distance1))
	{
		running_mode = 8;
		Target_speed = base_speed - 20;
		motor_flag = 1; // 电机输出
		yaw_target = in_angle1;
		mode_choose(running_mode);
	}
	else if (distance_L + distance_R > 2000 * (target_out_distance1 + target_in_distance1))
	{
		res = 0;
		TOF_object = 0; // 关闭
		outtoin_flag = 0;
	}

	return res;
}

void outp(float target_angle, float target_distance, float target_radiu) // 出库
{
	if (target_angle > 0)
	{
		if (yaw_output < target_angle && (distance_L + distance_R) < (target_distance * 2000) && outward_basin == 0 && OUTP_flag == 1)
		{
			running_mode = 0;
			Target_speed = 100;
			motor_flag = 1; // 电机输出
			err_speed = target_radiu * Target_speed * 0.0175;
			Target_Speed_L = Target_speed + err_speed;
			Target_Speed_R = Target_speed - err_speed;
		}
		else if (yaw_output >= target_angle || (distance_L + distance_R) >= (target_distance * 2000) && outward_basin == 0)
		{
			running_mode = 0;
			outward_basin = 1;
			OUTP_flag = 0;
			HRTRIG_flag = 1; // 停车
		}
	}
	else if (target_angle < 0)
	{
		if (yaw_output > target_angle && (distance_L + distance_R) < (target_distance * 1000) && outward_basin == 0 && OUTP_flag == 1)
		{
			running_mode = 0;
			Target_speed = 100;
			motor_flag = 1; // 电机输出
			err_speed = target_radiu * Target_speed * 0.0175;
			Target_Speed_L = Target_speed + err_speed;
			Target_Speed_R = Target_speed - err_speed;
		}
		else if (yaw_output <= target_angle || (distance_L + distance_R) >= (target_distance * 1000) && outward_basin == 0)
		{
			running_mode = 0;
			outward_basin = 1;
			OUTP_flag = 0;
			HRTRIG_flag = 1; // 停车
		}
	}
}

int8 intop(float target_angle, float target_distance, float target_radiu) // 入库
{
	int8 res = 5; // 当等于4入库成狗
	if (target_angle > 0)
	{
		// if (yaw_output < target_angle && distance_L + distance_R > -target_distance * 2000)
		// {

		// 	// motor_flag = 1; // 电机输出
		// 	// err_speed = target_radiu * Target_speed * 0.0175;
		// 	// Target_Speed_L = -Target_speed + err_speed;
		// 	// Target_Speed_R = -Target_speed - err_speed;
		// }
		// else if (yaw_output >= target_angle || distance_L + distance_R < -target_distance * 2000)
		// {
		// 	// res = 5;
		// 	// running_mode = 2;
		// 	// mode_choose(running_mode);
		// 	// stop_flag = 1;
		// }

		if (yaw_output < target_angle)
		{
			Target_speed = 0;
			yaw_target = target_angle;
			running_mode = 8;
			mode_choose(running_mode);
		}
		else if (yaw_output >= target_angle && distance_L + distance_R > -target_distance * 2000)
		{
			Target_speed = -100;
			yaw_target = target_angle;
			running_mode = 8;
			mode_choose(running_mode);
		}
		else if (yaw_output >= target_angle || distance_L + distance_R < -target_distance * 2000)
		{
			res = 5;
			running_mode = 2;
			mode_choose(running_mode);
			stop_flag = 1;
		}
	}
	if (target_angle < 0)
	{
		// if (yaw_output > target_angle && distance_L + distance_R > -target_distance * 2000)
		// {
		// 	// motor_flag = 1; // 电机输出
		// 	// err_speed = target_radiu * Target_speed * 0.0175;
		// 	// Target_Speed_L = -Target_speed + err_speed;
		// 	// Target_Speed_R = -Target_speed - err_speed;
		// }
		// else if (yaw_output <= target_angle || distance_L + distance_R < -target_distance * 2000)
		// {
		// 	// res = 5;
		// 	// running_mode = 2;
		// 	// mode_choose(running_mode);
		// 	// stop_flag = 1;
		// }
		if (yaw_output > target_angle)
		{
			Target_speed = 0;
			yaw_target = target_angle;
			running_mode = 8;
			mode_choose(running_mode);
		}
		else if (yaw_output <= target_angle && distance_L + distance_R > -target_distance * 2000)
		{
			Target_speed = -100;
			yaw_target = target_angle;
			running_mode = 8;
			mode_choose(running_mode);
		}
		else if (yaw_output <= target_angle && distance_L + distance_R < -target_distance * 2000)
		{
			res = 6;
			running_mode = 2;
			mode_choose(running_mode);
			stop_flag = 1;
		}
	}
	return res;
}
