#include "motor.h"

uint8 brusheless_flag;	// ��ˢ����
uint16 brusheless_duty; // ��ˢ�����ֵ��ֵ��Χ��500-1000��Ӧռ�ձ�Ϊ0%-100%
uint8 motor_flag;

uint8 Dir_L = 1;   // 1��߷�����ת  0��ת
uint8 Dir_R = 0;   // 0�ұ߷�����ת  1��ת
uint8 Dir_car = 1; // ����dir_car=1 L=1 R=0  ;dir_car=0 ���� L=0 R=1

int16 templ_pluse = 0;
int16 tempr_pluse = 0;

float Target_Speed_L = 0; // ����Ŀ���ٶ�
float Target_Speed_R = 0; // ����Ŀ���ٶ�

float Current_Speed_L; // ���ֵ�ǰ�ٶ�
float Current_Speed_R; // ���ֵ�ǰ�ٶ�

float duty_L;	  // ����pidռ�ձ�
float duty_R;	  // �ҵ��pidռ�ձ�
float set_duty_L; // �������ռ�ձ�
float set_duty_R; // �ҵ�����ռ�ձ�

int32 distance_L = 0; // ������ʻ����
int32 distance_R = 0; // ������ʻ����

PID_P Swerve_PID;	   // ת�����pid
PID_P Forward_PID;	   // ֱ��PID����
PID_P Pout_PID;		   // ������PID����
PID_P Pout_rate_PID; //���ٶ�PID����
PID_P roundaboutL_PID; // �󻷵�pid
PID_P roundaboutR_PID; // �һ���pid
PID_P Speed_PID_L;	   // �����ٶ�pid
PID_P Speed_PID_R;	   // �����ٶ�pid
PID_P climb_PID;				//����pid
int templ_pluse;	  // �����������
int tempr_pluse;	  // �����������
int Target_speed = 0; // ����Ŀ���ٶ�

// float PIDOUT_speed_L;			//�����ٶȻ���ƫ�������
// float PIDOUT_speed_R;			//�����ٶȻ���ƫ�������

// int32 Target_pluse_L=0;			//����Ŀ��������Ŀ
// int32 Target_pluse_R=0;			//����Ŀ��������Ŀ

// int32 Motor_pluse_L;			//�����ٶȻ����������
// int32 Motor_pluse_R;			//�����ٶȻ����������

// float Motor_duty_L=470;			//����Ŀ���ʼռ�ձ����ڿ˷�������ѹ����ԴΪ����״̬
// float Motor_duty_R=360;			//�ҵ��Ŀ���ʼռ�ձ����ڿ˷�������ѹ����ԴΪ����״̬

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid��������������ڲ�
//  @param      pid				pid����
//  @param
//  @return     *pid���� �ڲ�����������
//-------------------------------------------------------------------------------------------------------------------
void pid_init(PID_P *pid) // PID����������ʼ��
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
//  @brief      pidλ��ʽ���������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_Control(PID_P *pid, float error)
{
	if (error > 10 && error < -10)
	{
		pid->i_out += pid->KI * error;
	} // λ��ʽPID�������ۼ�
	else if (error < 10 && error > -10)
	{
		pid->i_out = 0;
	}
	pid->p_out = pid->KP * error;
	pid->d_out = pid->KD * (error - pid->last_err);
	pid->pid_out = pid->i_out + pid->p_out + pid->d_out;

	// PID����޷�
	if (pid->pid_out > (600))
		pid->pid_out = 600;
	if (pid->pid_out < -(600))
		pid->pid_out = -(600);
	// �������,�����´μ���
	pid->last_err = error;
	// ����PID���ֵ
	return pid->pid_out;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pidλ��ʽ���������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_direction_Control(PID_P *pid, float error)
{
	if (pid->err > 4 && pid->err < -4)
	{
		pid->i_out = 0;
	} // λ��ʽPID�������ۼ�
	else if (pid->err < 4 && pid->err > -4)
	{
		pid->i_out += pid->KI * error;
	}

	// pid->i_out += pid->KI * error;	   //λ��ʽPID�������ۼ�

	if (pid->i_out > (100))
		pid->i_out = 100;
	if (pid->i_out < -(100))
		pid->i_out = -(100);
	pid->p_out = pid->KP * error;
	pid->d_out = pid->KD * (error - pid->last_err);
	pid->pid_out = pid->i_out + pid->p_out + pid->d_out;

	// PID����޷�
	if (pid->pid_out > (600))
		pid->pid_out = 600;
	if (pid->pid_out < -(600))
		pid->pid_out = -(600);
	// �������,�����´μ���
	pid->last_err = error;
	// ����PID���ֵ
	return pid->pid_out;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pidλ��ʽ���������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_angle_Control(PID_P *pid, float error)
{
	if (pid->err > 4 && pid->err < -4)
	{
		pid->i_out = 0;
	} // λ��ʽPID�������ۼ�
	else if (pid->err < 5 && pid->err > -5)
	{
		pid->i_out += pid->KI * error;
	}

	// pid->i_out += pid->KI * error;	   //λ��ʽPID�������ۼ�

	if (pid->i_out > (1000))
		pid->i_out = 1000;
	if (pid->i_out < -(1000))
		pid->i_out = -(1000);
	pid->p_out = pid->KP * error;
	pid->d_out = pid->KD * (error - pid->last_err);
	pid->pid_out = pid->i_out + pid->p_out + pid->d_out;

	// PID����޷�
	if (pid->pid_out > (100000))
		pid->pid_out = 100000;
	if (pid->pid_out < -(100000))
		pid->pid_out = -(100000);
	// �������,�����´μ���
	pid->last_err = error;
	// ����PID���ֵ
	return pid->pid_out;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      pidλ��ʽ���������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_angle_rate_Control(PID_P *pid, float error)
{
	if (pid->err > 10 && pid->err < -10)
	{
		pid->i_out = 0;
	} // λ��ʽPID�������ۼ�
	else if (pid->err < 10 && pid->err > -10)
	{
		pid->i_out += pid->KI * error;
	}

	// pid->i_out += pid->KI * error;	   //λ��ʽPID�������ۼ�

	if (pid->i_out > (200))
		pid->i_out = 200;
	if (pid->i_out < -(200))
		pid->i_out = -(200);
	pid->p_out = pid->KP * error;
	pid->d_out = pid->KD * (error - pid->last_err);
	pid->pid_out = pid->i_out + pid->p_out + pid->d_out;

	// PID����޷�
	if (pid->pid_out > (600))
		pid->pid_out = 600;
	if (pid->pid_out < -(600))
		pid->pid_out = -(600);
	// �������,�����´μ���
	pid->last_err = error;
	// ����PID���ֵ
	return pid->pid_out;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      pid����ʽ���������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
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
//  @brief      pidλ��ʽ���������
//  @param      pid				pid����
//  @param      error			pid�������
//  @return     PID������
//-------------------------------------------------------------------------------------------------------------------
float PID_Loc_motor_Control(PID_P *pid, float error)
{
	pid->i_out += pid->KI * error; // λ��ʽPID�������ۼ�
	pid->p_out = pid->KP * error;
	pid->d_out = pid->KD * (error - pid->last_err);
	pid->pid_out = pid->i_out + pid->p_out + pid->d_out;

	// PID����޷�
	if (pid->pid_out > 7500)
		pid->pid_out = 8500;
	if (pid->pid_out < -7500)
		pid->pid_out = -8500;
	// �������,�����´μ���
	pid->last_err = error;
	// ����PID���ֵ
	return pid->pid_out;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ת���ڷ���
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void counte_quire() // ��ñ������������
{
	// ��ȡ�ɼ����ı�����������
	templ_pluse = ctimer_count_read(SPEEDL_PLUSE);
	tempr_pluse = ctimer_count_read(SPEEDR_PLUSE);

	// ����������
	ctimer_count_clean(SPEEDL_PLUSE);
	ctimer_count_clean(SPEEDR_PLUSE);

	// �ɼ�������Ϣ
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



void motor_output() // ���PWM���
{
	if (motor_flag == 0)
	{
		set_duty_L = 0; // ���ҵ�������
		set_duty_R = 0; // ���ҵ�������
	}
	else if (motor_flag == 1)
	{
		Speed_PID_L.err = Target_Speed_L - Current_Speed_L;
		Speed_PID_R.err = Target_Speed_R - Current_Speed_R;
		duty_L = PID_Loc_motor_Control(&Speed_PID_L, Speed_PID_L.err); // Ŀ������ٶ�+ת���ٶ�
		duty_R = PID_Loc_motor_Control(&Speed_PID_R, Speed_PID_R.err); // Ŀ������ٶ�+ת���ٶ�

		// �������Ʒ���
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


		// duty_L=Target_Speed_L;���Է���
		// duty_R=Target_Speed_R;���Է���

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
		// //���Ʒ������
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