#include "eeprom.h"

/********************************PID相关变量**********/
int rule_d[7] = {6, 5, 3, 2, 3, 5, 6}; // 模糊规则表 D
float kp_m;							   // 模糊kp最大值
float kd_m;							   // 模糊kp最大值
float err = 0;						   // 寻迹误差
float err_last = 0;					   // 上一次的寻迹误查
float angle_err = 0;
float Kp, Kd, output;
float EC = 0, E = 0;
float gyro_d; // 陀螺仪系数
/***************************************************************************************************/

//-------------------------------------------------------------------------------------------------------------------
//  @brief      无线转串口发送
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void read_all_par() // 开机读取数据
{
	uint8 read_pid1_strline[256]; // 读取PID的缓存内容，后续将转化为float
	uint8 read_pid2_strline[256]; // 读取PID的缓存内容，后续将转化为float
	uint8 read_pid3_strline[256]; // 读取PID的缓存内容，后续将转化为float
	uint8 read_pid4_strline[256]; // 读取PID的缓存内容，后续将转化为float
	iap_read_bytes(0x00, read_pid1_strline, 256);
	printf("[read1%s])", read_pid1_strline); // 上位机串口检查写入内容
	sscanf(read_pid1_strline, "%f,%f,%f,%f,%f,%f,%d", &Speed_PID_L.KP, &Speed_PID_L.KI, &Speed_PID_L.KD, &Speed_PID_R.KP, &Speed_PID_R.KI, &Speed_PID_R.KD, &uart_mode);

	iap_read_bytes(0x100, read_pid2_strline, 256);
	printf("[read2%s])", read_pid2_strline); // 上位机串口检查写入内容
	sscanf(read_pid2_strline, "%f,%f,%f,%f,%f,%f,%f", &Forward_PID.KP, &Forward_PID.KI, &Forward_PID.KD, &Swerve_PID.KP, &Swerve_PID.KI, &Swerve_PID.KD,&base_speed);

	iap_read_bytes(0x200, read_pid3_strline, 256);
	printf("[read3%s])", read_pid3_strline); // 上位机串口检查写入内容
	sscanf(read_pid3_strline, "%f,%f,%f,%f,%f,%f,%f", &gyro_d, &Pout_PID.KP, &Pout_PID.KI, &Pout_PID.KD, &Pout_rate_PID.KP, &Pout_rate_PID.KI, &Pout_rate_PID.KD);

	iap_read_bytes(0x300, read_pid4_strline, 256);
	printf("[read4%s])", read_pid4_strline); // 上位机串口检查写入内容
	sscanf(read_pid4_strline,"%f,%f,%f,%f,%f,%f,%d",&roundaboutL_PID.KI,&roundaboutL_PID.KI,&roundaboutL_PID.KD,&roundaboutR_PID.KP,&roundaboutR_PID.KI,&roundaboutR_PID.KD,&pack_mode);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      利用eeporm写入数据
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void write_all_par() // 利用eeporm写入数据
{
	uint8 eeprom_pid1_strline[256]; // 用静态储存储存pid参数，将浮点转换成字符串传到E^2prom
	uint8 eeprom_pid2_strline[256]; // 用静态储存储存pid参数，将浮点转换成字符串传到E^2prom
	uint8 eeprom_pid3_strline[256]; // 用静态储存储存pid参数，将浮点转换成字符串传到E^2prom
	uint8 eeprom_pid4_strline[256]; // 用静态储存储存pid参数，将浮点转换成字符串传到E^2prom
	iap_erase_page(0x20);			// 擦除0-0X200
	iap_erase_page(0x40);			// 擦除0X400
	iap_erase_page(0x60);			// 擦除0X400
	iap_erase_page(0x80);			// 擦除0X400
	sprintf(eeprom_pid1_strline, "%f,%f,%f,%f,%f,%f,%d", Speed_PID_L.KP, Speed_PID_L.KI, Speed_PID_L.KD, Speed_PID_R.KP, Speed_PID_R.KI, Speed_PID_R.KD, uart_mode);
	printf("(write1%s)", eeprom_pid1_strline); // 上位机串口检查写入内容
	extern_iap_write_bytes(0x00, eeprom_pid1_strline, 256);

	sprintf(eeprom_pid2_strline, "%f,%f,%f,%f,%f,%f,%f", Forward_PID.KP, Forward_PID.KI, Forward_PID.KD, Swerve_PID.KP, Swerve_PID.KI, Swerve_PID.KD,base_speed);
	printf("(write2%s)", eeprom_pid2_strline); // 上位机串口检查写入内容
	extern_iap_write_bytes(0x100, eeprom_pid2_strline, 256);

	sprintf(eeprom_pid3_strline, "%f,%f,%f,%f,%f,%f,%f", gyro_d, Pout_PID.KP, Pout_PID.KI, Pout_PID.KD,Pout_rate_PID.KP, Pout_rate_PID.KI, Pout_rate_PID.KD);
	printf("(write3%s)", eeprom_pid3_strline); // 上位机串口检查写入内容
	extern_iap_write_bytes(0x200, eeprom_pid3_strline, 256);

	sprintf(eeprom_pid4_strline, "%f,%f,%f,%f,%f,%f,%d",roundaboutL_PID.KI,roundaboutL_PID.KI,roundaboutL_PID.KD,roundaboutR_PID.KP,roundaboutR_PID.KI,roundaboutR_PID.KD,pack_mode);
	printf("(write4%s)", eeprom_pid4_strline); // 上位机串口检查写入内容
	extern_iap_write_bytes(0x300, eeprom_pid4_strline, 256);
}
