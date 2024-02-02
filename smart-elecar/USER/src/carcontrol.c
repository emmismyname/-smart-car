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
/****************************************************************************************/



/****************************************************************************************/


//-------------------------------------------------------------------------------------------------------------------
//  @brief      �˶�ģʽ�л�
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void mode_choose(int move_mode) // �˶�ģʽ�л�
{
	// int temp_counter=1;
	if (trace_flag == 0) // ��ȫͣ��
	{
		// ��������⵽ʱ�����trace_flag==0ֹͣѰ��
		Target_speed = 0;
		Target_Speed_L = 0;
		Target_Speed_R = 0;
		motor_flag = 0;
	}
	else if (trace_flag == 1)
	{
		switch (move_mode)
		{
		case 1: // ����
		{
		}
		break;
		case 2: // ֹͣpid
		{
			Target_Speed_L = 0;
			Target_Speed_R = 0;
		}
		break;
		case 3: // ֱ��pid
		{
			motor_flag = 1; // ������
			Target_Speed_L = Target_speed - PID_Loc_Control(&Forward_PID, err) - z_gyro * gyro_d;
			Target_Speed_R = Target_speed + PID_Loc_Control(&Forward_PID, err) + z_gyro * gyro_d;
		}
		break;
		case 4: // ������
		{
			motor_flag = 1; // ������
			Target_Speed_L = Target_speed - PID_Loc_Control(&Swerve_PID, err) - z_gyro * gyro_d * 0.7;
			Target_Speed_R = Target_speed + PID_Loc_Control(&Swerve_PID, err) + z_gyro * gyro_d * 0.7;
		}
		break;
		case 5: // ���������һ���
		{
			motor_flag = 1; // ������
			Target_Speed_L = Target_speed - PID_Loc_Control(&roundaboutR_PID, err) - z_gyro * gyro_d * 0.7;
			Target_Speed_R = Target_speed + PID_Loc_Control(&roundaboutR_PID, err) + z_gyro * gyro_d * 0.7;
		}
		case 6: // ���������󻷵�
		{
			motor_flag = 1; // ������
			Target_Speed_L = Target_speed - PID_Loc_Control(&roundaboutL_PID, err) - z_gyro * gyro_d * 0.7;
			Target_Speed_R = Target_speed + PID_Loc_Control(&roundaboutL_PID, err) + z_gyro * gyro_d * 0.7;
		}
		break;
		case 7: // �뻷��������
		{
			motor_flag = 1; // ������
			Target_Speed_L = Target_speed - PID_Loc_Control(&Forward_PID, err) - z_gyro * gyro_d * 1.2;
			Target_Speed_R = Target_speed + PID_Loc_Control(&Forward_PID, err) + z_gyro * gyro_d * 1.2;
		}
		case 8: // �ǶȻ�����
		{
			motor_flag = 1; // ������
			Target_Speed_L = Target_speed - PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
			Target_Speed_R = Target_speed + PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
		}
		break;
		case 9: // ���µ���
		{
			motor_flag = 1; // ������
			Target_Speed_L = Target_speed - PID_Loc_Control(&Forward_PID, err) - z_gyro * gyro_d * 1.4;
			Target_Speed_R = Target_speed + PID_Loc_Control(&Forward_PID, err) + z_gyro * gyro_d * 1.4;
		}
		break;
		case 15: // �������뻷����
		{
			motor_flag = 1; // ������
			Target_Speed_L = Target_speed - PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
			Target_Speed_R = Target_speed + PID_Loc_angle_rate_Control(&Pout_rate_PID, err_angle_rate);
			// Target_Speed_L = Target_speed - PID_Loc_Control(&roundaboutR_PID, Err_Inclined_right) - z_gyro * gyro_d * 0.7;
			// Target_Speed_R = Target_speed + PID_Loc_Control(&roundaboutR_PID, Err_Inclined_right) + z_gyro * gyro_d * 0.7;
		}
		break;
		case 16: // �������뻷����
		{
			motor_flag = 1; // ������
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
	case 1: // ����
		in_angle = -60;
		in_distance = 50;
		out_angle = 30;
		out_distance = 70;
		break;
	case 2: // �ҳ��Һ�
		in_angle = -60;
		in_distance = 50;
		out_angle = 30;
		out_distance = 70;
		break;

	default:
		break;
	}
} // void obstacle_avoidance(float out_distance,float in_distance,float out_angle,float in_angle) //���ϴ���

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
int obstacle_avoidance(float out_angle1, float in_angle1, float target_out_distance1, float target_in_distance1) // ���ϴ���
{
	int res = 1;
	if (distance_L + distance_R < 2000 * target_out_distance1)
	{
		running_mode = 8;
		Target_speed = base_speed - 20;
		motor_flag = 1; // ������
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
		motor_flag = 1; // ������
		yaw_target = in_angle1;
		mode_choose(running_mode);
	}
	else if (distance_L + distance_R > 2000 * (target_out_distance1 + target_in_distance1))
	{
		res = 0;
		TOF_object = 0; // �ر�
		outtoin_flag = 0;
	}

	return res;
}



