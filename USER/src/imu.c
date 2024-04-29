#include "imu.h"

int16 gyro_flag = 0; // 0Ϊ��Ư״̬��1Ϊ���ڼ����־��2��ȡ��������  3��ȡ��ֵ������
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
float pitch_last = 0.0f, roll_last = 0.0f, yaw_last = 0.0f;
float pitch_err = 0.0f, roll_err = 0.0f, yaw_err = 0.0f;
float pitch_circle = 0.0f, roll_circle = 0.0f, yaw_circle = 0.0f;
float pitch_result = 0.0f, roll_result = 0.0f, yaw_result = 0.0f;
static int first_flag = 0;//��һ�μ������ֵ

int Zero_Offset_Count = 0; // ��ʼ���ۼ�ֵ
float move_z_gyro;
float y_gyro_offset, z_gyro_offset, x_gyro_offset;	  // ��ʼ����������Ư��ֵ
float y_acc, z_acc, x_acc;							  // ��ʼ�����ٶȼ���ֵ
float y_acc_offset, z_acc_offset, x_acc_offset;		  // ��ʼ����������Ư��ֵ
float dt = 0.01f;									  // 0.01ϵ��
int Zero_Offset_End;								  // 100��
float yaw_result_last = 0, org_yaw = 0, gyro_yaw = 0; // ���ֵ����һ״̬��ֵ
float det_yaw = 0;									  // yaw��΢����
float yaw_output = 0;								  // yaw�ļ�¼���ֵ
float gyro[11] = {0}, gyro_sum;
float gyro_d=1;
int16 y_gyro = 0, z_gyro = 0, x_gyro = 0;			  // ��ʼ����������ֵ
uint8 mpu_flag=0; //mpu�Ĺ�����־λ

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ֵ����
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
int myabs(int number) // ����ȡ����ֵ
{
	if (number < 0)
		return -number;
	else
		return number;
}
float myfabs(float number) // ������ȡ����ֵ
{
	if (number < 0)
		return -1.0 * number;
	else
		return number;
}
float squre_sum(float number1,float number2) // ������ȡ����ֵ
{
	float result;
  result=number1*number1+number2*number2;
  result=sqrt(result);
  return result;

}

//-------------------------------------------------------------------------------------------------------------------
// @brief		�����Ǿ�ֵ�˲�
//
//-------------------------------------------------------------------------------------------------------------------
// float gyro_aquire(float org_gyro)
// {
// 	static int i = 0;
// 	gyro[1] = org_gyro;
// 	for (i = 1; i <=10; i++)
// 	{
// 		gyro[i+1] = gyro[i];
// 	}
// 	for (i = 1; i <=10; i++)
// 	{
// 		gyro_sum += gyro[i];
// 	}
// 	gyro[11] = gyro_sum / 10;
// 	gyro_sum = 0;
// 	return gyro[11];
// }


//-------------------------------------------------------------------------------------------------------------------
//  @brief      MPU��ȡ����ʹ��
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void enableMPU()
{
   mpu_flag=1;///��־λ��1
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      MPU��ȡ����ʹ��
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void disableMPU()
{
   mpu_flag=0;///��־λ����
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      MPU���ݻ�ȡ
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void MPU_Acquire()
{
	if(mpu_flag==1)
	{
		MPU6050_Refresh_DMP();
		MPU_Get_Gyroscope(&mpu6050_gyro_x, &mpu6050_gyro_y, &mpu6050_gyro_z);
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      MPU���ݼ���
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void MPU_Process()
{
		if(mpu_flag==1)
	{
		z_gyro=mpu6050_gyro_z-z_gyro_offset;
		// z_gyro = mpu6050_gyro_z;
		gyro_yaw += z_gyro;
		yaw_last = yaw;
		yaw_result_last = yaw_result;
		q0 = Q[0] / q30;
		q1 = Q[1] / q30;
		q2 = Q[2] / q30;
		q3 = Q[3] / q30;																	// ��Ԫ��
		pitch = asin(-2 * Q[1] * Q[3] + 2 * Q[0] * Q[2]) * 57.3;													// pitch����������
		roll = atan2(2 * Q[2] * Q[3] + 2 * Q[0] * Q[1], -2 * Q[1] * Q[1] - 2 * Q[2] * Q[2] + 1) * 57.3;				// roll�����������
		yaw = atan2(2 * (Q[1] * Q[2] + Q[0] * Q[3]), Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3]) * 57.3; // yaw �����ƫ����

		// if (pitch < 0)
		// 	pitch += 360;
		// if (roll < 0)
		// 	roll += 360;
		org_yaw = yaw;
		if (yaw < 0)
			yaw += 360;
		if (first_flag == 0)
		{
			pitch_last = pitch;
			roll_last = roll;
			yaw_last = yaw;
			first_flag++;
		}
		// pitch_err = pitch - pitch_last;
		// roll_err = roll - roll_last;
		yaw_err = yaw - yaw_last;
		// if (pitch_err < -180)
		// 	pitch_circle++;
		// else if (pitch_err > 180)
		// 	pitch_circle--;	
		// if (roll_err < -180)
		// 	roll_circle++;
		// else if (roll_err > 180)
		// 	roll_circle--;
		if (yaw_err < -180)
			yaw_circle++;
		else if (yaw_err > 180)
			yaw_circle--;
		// pitch_last = pitch;
		// roll_last = roll;
		// pitch_result = pitch + 360 * pitch_circle;
		// roll_result = roll + 360 * roll_circle;
		yaw_result = yaw + 360 * yaw_circle;
		det_yaw = yaw_result - yaw_result_last;
		if (myabs(det_yaw) < 100)
			yaw_output += det_yaw;
		
	}
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������dmp�����Զ�����ʱ��
//  @param      none
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Zero_Offset_set()
{
	static int i = 1;
		delay_ms(1000);
	while (i < Zero_Offset_time)
	{
		enableMPU();
		pitch = asin(-2 * Q[1] * Q[3] + 2 * Q[0] * Q[2]) * 57.3;													// pitch
		roll = atan2(2 * Q[2] * Q[3] + 2 * Q[0] * Q[1], -2 * Q[1] * Q[1] - 2 * Q[2] * Q[2] + 1) * 57.3;				// roll
		yaw = atan2(2 * (Q[1] * Q[2] + Q[0] * Q[3]), Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3]) * 57.3; // yaw
		 y_gyro_offset+=mpu6050_gyro_y;
		 z_gyro_offset+=mpu6050_gyro_z;
	   x_gyro_offset+=mpu6050_gyro_x;
		i++;
		delay_ms(5);
	}
	delay_ms(100);
		printf("yuanshi%f,%f,%f\n", y_gyro_offset, z_gyro_offset, x_gyro_offset);
	 z_gyro_offset=z_gyro_offset/Zero_Offset_time;
	 y_gyro_offset=y_gyro_offset/Zero_Offset_time;
	 x_gyro_offset=x_gyro_offset/Zero_Offset_time;
	printf("chulihou%f,%f,%f\n", y_gyro_offset, z_gyro_offset, x_gyro_offset);
	delay_ms(100);
} // ��ʼ������������