#include "motor.h"

uint8 brusheless_flag;	// 无刷开关
uint16 brusheless_duty; // 无刷电机数值数值范围在500-1000对应占空比为0%-100%
uint8 motor_flag;

uint8 Dir_L = 1;   // 1左边方向正转  0反转
uint8 Dir_R = 0;   // 0右边方向正转  1反转
uint8 Dir_car = 1; // 正走dir_car=1 L=1 R=0  ;dir_car=0 反走 L=0 R=1

int16 templ_pluse = 0;
int16 tempr_pluse = 0;

float Target_Speed_L = 0; // 左轮目标速度
float Target_Speed_R = 0; // 右轮目标速度

float Current_Speed_L; // 左轮当前速度
float Current_Speed_R; // 右轮当前速度

float duty_L;	  // 左电机pid占空比
float duty_R;	  // 右电机pid占空比
float set_duty_L; // 左电机输出占空比
float set_duty_R; // 右电机输出占空比

int32 distance_L = 0; // 左轮行驶距离
int32 distance_R = 0; // 右轮行驶距离

PID_P Swerve_PID;	   // 转弯差速pid
PID_P Forward_PID;	   // 直行PID控制
PID_P Pout_PID;		   // 陀螺仪PID控制
PID_P Pout_rate_PID; //角速度PID控制
PID_P roundaboutL_PID; // 左环岛pid
PID_P roundaboutR_PID; // 右环岛pid
PID_P Speed_PID_L;	   // 左轮速度pid
PID_P Speed_PID_R;	   // 右轮速度pid
PID_P climb_PID;				//上坡pid
int templ_pluse;	  // 左轮脉冲个数
int tempr_pluse;	  // 右轮脉冲个数
int Target_speed = 0; // 车体目标速度

// float PIDOUT_speed_L;			//左轮速度环的偏差计算结果
// float PIDOUT_speed_R;			//右轮速度环的偏差计算结果

// int32 Target_pluse_L=0;			//左轮目标脉冲数目
// int32 Target_pluse_R=0;			//右轮目标脉冲数目

// int32 Motor_pluse_L;			//左轮速度环的脉冲个数
// int32 Motor_pluse_R;			//左轮速度环的脉冲个数

// float Motor_duty_L=470;			//左电机目标初始占空比用于克服死区电压，电源为满电状态
// float Motor_duty_R=360;			//右电机目标初始占空比用于克服死区电压，电源为满电状态

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid控制器输出更新内部
//  @param      pid				pid参数
//  @param
//  @return     *pid参数 内部输出结果清零
//-------------------------------------------------------------------------------------------------------------------
void pid_init(PID_P *pid) // PID的误差输出初始化
{
	pid->last_err = 0;
	pid->lastBefore_err = 0;
	pid->err = 0;
	pid->pid_out = 0;
	pid->i_out = 0;
	pid->d_out = 0;
	pid->p_out = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid位置式控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_Control(PID_P *pid, float error)
{
	if (error > 10 && error < -10)
	{
		pid->i_out += pid->KI * error;
	} // 位置式PID积分项累加
	else if (error < 10 && error > -10)
	{
		pid->i_out = 0;
	}
	pid->p_out = pid->KP * error;
	pid->d_out = pid->KD * (error - pid->last_err);
	pid->pid_out = pid->i_out + pid->p_out + pid->d_out;

	// PID输出限幅
	if (pid->pid_out > (600))
		pid->pid_out = 600;
	if (pid->pid_out < -(600))
		pid->pid_out = -(600);
	// 更新误差,用于下次计算
	pid->last_err = error;
	// 返回PID输出值
	return pid->pid_out;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid位置式控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_direction_Control(PID_P *pid, float error)
{
	if (pid->err > 4 && pid->err < -4)
	{
		pid->i_out = 0;
	} // 位置式PID积分项累加
	else if (pid->err < 4 && pid->err > -4)
	{
		pid->i_out += pid->KI * error;
	}

	// pid->i_out += pid->KI * error;	   //位置式PID积分项累加

	if (pid->i_out > (100))
		pid->i_out = 100;
	if (pid->i_out < -(100))
		pid->i_out = -(100);
	pid->p_out = pid->KP * error;
	pid->d_out = pid->KD * (error - pid->last_err);
	pid->pid_out = pid->i_out + pid->p_out + pid->d_out;

	// PID输出限幅
	if (pid->pid_out > (600))
		pid->pid_out = 600;
	if (pid->pid_out < -(600))
		pid->pid_out = -(600);
	// 更新误差,用于下次计算
	pid->last_err = error;
	// 返回PID输出值
	return pid->pid_out;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid位置式控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_angle_Control(PID_P *pid, float error)
{
	if (pid->err > 4 && pid->err < -4)
	{
		pid->i_out = 0;
	} // 位置式PID积分项累加
	else if (pid->err < 5 && pid->err > -5)
	{
		pid->i_out += pid->KI * error;
	}

	// pid->i_out += pid->KI * error;	   //位置式PID积分项累加

	if (pid->i_out > (1000))
		pid->i_out = 1000;
	if (pid->i_out < -(1000))
		pid->i_out = -(1000);
	pid->p_out = pid->KP * error;
	pid->d_out = pid->KD * (error - pid->last_err);
	pid->pid_out = pid->i_out + pid->p_out + pid->d_out;

	// PID输出限幅
	if (pid->pid_out > (100000))
		pid->pid_out = 100000;
	if (pid->pid_out < -(100000))
		pid->pid_out = -(100000);
	// 更新误差,用于下次计算
	pid->last_err = error;
	// 返回PID输出值
	return pid->pid_out;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid位置式控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_angle_rate_Control(PID_P *pid, float error)
{
	if (pid->err > 10 && pid->err < -10)
	{
		pid->i_out = 0;
	} // 位置式PID积分项累加
	else if (pid->err < 10 && pid->err > -10)
	{
		pid->i_out += pid->KI * error;
	}

	// pid->i_out += pid->KI * error;	   //位置式PID积分项累加

	if (pid->i_out > (200))
		pid->i_out = 200;
	if (pid->i_out < -(200))
		pid->i_out = -(200);
	pid->p_out = pid->KP * error;
	pid->d_out = pid->KD * (error - pid->last_err);
	pid->pid_out = pid->i_out + pid->p_out + pid->d_out;

	// PID输出限幅
	if (pid->pid_out > (600))
		pid->pid_out = 600;
	if (pid->pid_out < -(600))
		pid->pid_out = -(600);
	// 更新误差,用于下次计算
	pid->last_err = error;
	// 返回PID输出值
	return pid->pid_out;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid增量式控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Inc_Control(PID_P *pid, float error)
{
	pid->p_out = pid->KP * (error - pid->last_err);
	pid->i_out = pid->KI * error;
	pid->d_out = pid->KD * (error - 2 * pid->last_err + pid->lastBefore_err);

	pid->lastBefore_err = pid->last_err;
	pid->last_err = error;

	pid->pid_out += pid->p_out + pid->i_out + pid->d_out;
	return pid->pid_out;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid位置式控制器输出
//  @param      pid				pid参数
//  @param      error			pid输入误差
//  @return     PID输出结果
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_motor_Control(PID_P *pid, float error)
{
	pid->i_out += pid->KI * error; // 位置式PID积分项累加
	pid->p_out = pid->KP * error;
	pid->d_out = pid->KD * (error - pid->last_err);
	pid->pid_out = pid->i_out + pid->p_out + pid->d_out;

	// PID输出限幅
	if (pid->pid_out > 7500)
		pid->pid_out = 8500;
	if (pid->pid_out < -7500)
		pid->pid_out = -8500;
	// 更新误差,用于下次计算
	pid->last_err = error;
	// 返回PID输出值
	return pid->pid_out;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      无线转串口发送
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void counte_quire() // 获得编码器脉冲个数
{
	// 读取采集到的编码器脉冲数
	templ_pluse = ctimer_count_read(SPEEDL_PLUSE);
	tempr_pluse = ctimer_count_read(SPEEDR_PLUSE);

	// 计数器清零
	ctimer_count_clean(SPEEDL_PLUSE);
	ctimer_count_clean(SPEEDR_PLUSE);

	// 采集方向信息
	if (0 == SPEEDL_DIR)
	{
		templ_pluse = -templ_pluse;
	}

	if (1 == SPEEDR_DIR)
	{
		tempr_pluse = -tempr_pluse;
	}

	Current_Speed_R = 1.859375 * tempr_pluse; // 30/68 * 6.8 * 3.14 * n/1024 /0.005
	Current_Speed_L = 1.859375 * templ_pluse;
	distance_L += Current_Speed_L;
	distance_R += Current_Speed_R;
}



void motor_output() // 电机PWM输出
{
	if (motor_flag == 0)
	{
		set_duty_L = 0; // 左右电机不输出
		set_duty_R = 0; // 左右电机不输出
	}
	else if (motor_flag == 1)
	{
		Speed_PID_L.err = Target_Speed_L - Current_Speed_L;
		Speed_PID_R.err = Target_Speed_R - Current_Speed_R;
		duty_L = PID_Loc_motor_Control(&Speed_PID_L, Speed_PID_L.err); // 目标基础速度+转向速度
		duty_R = PID_Loc_motor_Control(&Speed_PID_R, Speed_PID_R.err); // 目标基础速度+转向速度

		// 积分限制幅度
		if (Speed_PID_L.i_out > 3000)
		{
			Speed_PID_L.i_out = 3000;
		}
		else if (Speed_PID_L.i_out < -3000)
		{
			Speed_PID_L.i_out = -3000;
		}
		if (Speed_PID_R.i_out > 3000)
		{
			Speed_PID_R.i_out = 3000;
		}
		else if (Speed_PID_R.i_out < -3000)
		{
			Speed_PID_R.i_out = -3000;
		}


		// duty_L=Target_Speed_L;调试方向
		// duty_R=Target_Speed_R;调试方向

		if (duty_L >= 0)
		{
			Dir_L = 0;
			set_duty_L = duty_L;
			if (set_duty_L >= 6500)
				set_duty_L = 6500;
			else if (set_duty_L <= 0)
				set_duty_L = 0;
		}
		else if (duty_L < 0)
		{
			Dir_L = 1;
			set_duty_L = -duty_L;
			if (set_duty_L >= 6500)
				set_duty_L = 6500;
			else if (set_duty_L <= 0)
				set_duty_L = 0;
		}
		
		if (duty_R >= 0)
		{
			set_duty_R = duty_R;
			Dir_R = 0;
			if (set_duty_R >= 6500)
				set_duty_R = 6500;
			else if (set_duty_R <= 0)
				set_duty_R = 0;
		}
		else if (duty_R < 0)
		{
			Dir_R = 1;
			set_duty_R = -duty_R;
			if (set_duty_R >= 6500)
				set_duty_R = 6500;
			else if (set_duty_R <= 0)
				set_duty_R = 0;
		}
    

		// if(Dir_car==1)
		// {
		// 	Dir_L=1;
		// 	Dir_R=0;
		// }
		// else if(Dir_car==0)
		// { 0
		// 	Dir_L=0;
		// 	Dir_R=1;
		// }
		// //限制幅度输出
		// if(duty_L >= 6500) duty_L = 6500;
		// else if(duty_L <= 0) duty_L = 0;
		// if(duty_R >= 6500) duty_R = 6500;
		// else if(duty_R <= 0) duty_R = 0;
	}
	DIR_1 = Dir_L;
	DIR_2 = Dir_R;
	pwm_duty(PWM_1, set_duty_L);
	pwm_duty(PWM_2, set_duty_R);
}