#include "carcontrol.h"

// int running_mode = 1;		// 1�Ǵ���motor����0 2��ͣ�� 3��ͨѰ������ʱ����
// uint8 trace_flag = 0;		// Ѱ������
// int current_number = 0; // ɨ�迪ʼ��־λ
// int trigflag_beep = 1;	// ����������
// int outward_basin = 0;	// �����־λ
// uint8 avoid_move = 0;		// ���Ͽ���
// uint8 TOF_object = 0;		// δ��⵽ 1��⵽�ϰ��� 2����ϰ����ʼ���� 3���Ͻ���
// int TOF_barrier[9] = {0};
// int OUTP_flag = 0;		// ��ʱ���������� 0�������˶����� 1�ҳ��� 2����� 3����ֱ��
// uint8 start_flag = 0; // ������־λ
// uint8 stop_flag = 0;	// ͣ����־λ
// uint8 small_radiu = 0;

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

// // ���
// float intop_angle = 80;		// ���Ŀ��Ƕ�
// float intop_distance = 15; // ������
// float intop_radiu = 0;		// ���Բ��
// /***************************************************************************************/

// // ����
// float outp_angle = 70;		 // ����Ŀ��Ƕ�
// float outp_distance = 13; // ������
// float outp_radiu = -22;		 // ���Բ��
// /****************************************************************************************/


/**************************************����һ��************************************/
//����
int16 route_distance=0;//��������
bit start_flag = 0; // ������־λ
bit stop_flag = 0;	// ͣ����־λ



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
	if (jiangxing_No_1.trace_flag == 0) // ��ȫͣ��
	{
		// ��������⵽ʱ�����trace_flag==0ֹͣѰ��
		jiangxing_No_1.car_speed = 0;
		jiangxing_No_1.left_target_speed = 0;
		jiangxing_No_1.right_target_speed = 0;
		motor_flag = 0;
	}
	else if (jiangxing_No_1.trace_flag == 1)
	{
		switch (move_mode)
		{
		case 1: // ����
		{
		}
		break;
		case 2: // ֹͣpid
		{
			jiangxing_No_1.left_target_speed = 0;
			jiangxing_No_1.right_target_speed = 0;
		}
		break;
		case 3: // ֱ��pid
		{
			motor_flag = 1; // ������
			jiangxing_No_1.left_target_speed = jiangxing_No_1.car_speed - PID_Loc_Control(&Forward_PID, get_error1) + z_gyro * gyro_d;
			jiangxing_No_1.right_target_speed = jiangxing_No_1.car_speed + PID_Loc_Control(&Forward_PID, get_error1) - z_gyro * gyro_d;
		}
		break;
		// case 4: // ������
		// {
		// 	motor_flag = 1; // ������
		// 	Target_Speed_L = Target_speed - PID_Loc_Control(&Forward_PID, get_error1) - z_gyro * gyro_d;
		// 	Target_Speed_R = Target_speed + PID_Loc_Control(&Forward_PID, get_error1) + z_gyro * gyro_d;
		// }
		// break;
		// case 8: // �ǶȻ�����
		// {
		// 	motor_flag = 1; // ������
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
//  @brief      Ԫ��ʶ��
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void Element_Idef() // Ԫ��ʶ��
{
  
        if (jiangxing_No_1.trace_flag== 1) //	trace_flag==1;//Ѱ����־λ
        {   
					  // if(jiangxing_No_1.start_flag==1)
						// {
							enablemotor();
            	jiangxing_No_1.running_mode=3;
						// }

						if ((float)(SPI_float>= 500)) // ����ͣ��
						{
//							jiangxing_No_1.trace_flag = 0;
					    jiangxing_No_1.start_flag=0;
							jiangxing_No_1.stop_flag = 1;
							jiangxing_No_1.running_mode=2;
							jiangxing_No_1.motor_flag = 1;
							pid_all_init();
						}
        }
				if ((jiangxing_No_1.stop_flag == 1)) // ��λ��ͣ��
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
 if(jiangxing_No_1.start_mode==0)//��������
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
	case 0://��������
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