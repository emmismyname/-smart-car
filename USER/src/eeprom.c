#include "eeprom.h"


/********************************PID��ر������������־λ**********/
uint8 para_change_flag=0;



/********************************PID��ر���**********/
float err = 0;						   // Ѱ�����
float err_last = 0;					   // ��һ�ε�Ѱ�����
float angle_err = 0;
float Kp, Kd, output;
float EC = 0, E = 0;
/***************************************************************************************************/

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ת���ڷ���
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void read_all_par() // ������ȡ����
{
	uint8 read_pid1_strline[256]; // ��ȡPID�Ļ������ݣ�������ת��Ϊfloat
	uint8 read_pid2_strline[256]; // ��ȡPID�Ļ������ݣ�������ת��Ϊfloat
	uint8 read_pid3_strline[256]; // ��ȡPID�Ļ������ݣ�������ת��Ϊfloat
	// uint8 read_pid4_strline[256]; // ��ȡPID�Ļ������ݣ�������ת��Ϊfloat
	iap_read_bytes(0x00, read_pid1_strline, 256);
	printf("[read1%s])", read_pid1_strline); // ��λ�����ڼ��д������
	sscanf(read_pid1_strline, "%f,%f,%f,%f,%f,%f,%c", &Speed_PID_L.KP, &Speed_PID_L.KI, &Speed_PID_L.KD, &Speed_PID_R.KP, &Speed_PID_R.KI, &Speed_PID_R.KD, &jiangxing_No_1.uart_mode);

	iap_read_bytes(0x100, read_pid2_strline, 256);
	printf("[read2%s])", read_pid2_strline); // ��λ�����ڼ��д������
	sscanf(read_pid2_strline, "%f,%f,%f", &Forward_PID.KP, &Forward_PID.KI, &Forward_PID.KD);

	iap_read_bytes(0x200, read_pid3_strline, 256);
	printf("[read3%s])", read_pid3_strline); // ��λ�����ڼ��д������
	sscanf(read_pid3_strline, "%f", &gyro_d);

	// iap_read_bytes(0x300, read_pid4_strline, 256);
	// printf("[read4%s])", read_pid4_strline); // ��λ�����ڼ��д������
	// sscanf(read_pid4_strline,"%f,%f,%f,%f,%f,%f",&roundaboutL_PID.KI,&roundaboutL_PID.KI,&roundaboutL_PID.KD,&roundaboutR_PID.KP,&roundaboutR_PID.KI,&roundaboutR_PID.KD);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����eepormд������
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void write_all_par() // ����eepormд������
{
	uint8 eeprom_pid1_strline[256]; // �þ�̬���洢��pid������������ת�����ַ�������E^2prom
	uint8 eeprom_pid2_strline[256]; // �þ�̬���洢��pid������������ת�����ַ�������E^2prom
	uint8 eeprom_pid3_strline[256]; // �þ�̬���洢��pid������������ת�����ַ�������E^2prom
//	uint8 eeprom_pid4_strline[256]; // �þ�̬���洢��pid������������ת�����ַ�������E^2prom
	iap_erase_page(0x0000);			// ����0-0X200
	iap_erase_page(0x0200);			// ����200-0X400
	sprintf(eeprom_pid1_strline, "%f,%f,%f,%f,%f,%f,%c", Speed_PID_L.KP, Speed_PID_L.KI, Speed_PID_L.KD, Speed_PID_R.KP, Speed_PID_R.KI, Speed_PID_R.KD, jiangxing_No_1.uart_mode);
	printf("(write1%s)", eeprom_pid1_strline); // ��λ�����ڼ��д������
	iap_write_bytes(0x00, eeprom_pid1_strline, 256);

	sprintf(eeprom_pid2_strline, "%f,%f,%f,", Forward_PID.KP, Forward_PID.KI, Forward_PID.KD);
	printf("(write2%s)", eeprom_pid2_strline); // ��λ�����ڼ��д������
	iap_write_bytes(0x100, eeprom_pid2_strline, 256);

	sprintf(eeprom_pid3_strline, "%f", gyro_d);
	printf("(write3%s)", eeprom_pid3_strline); // ��λ�����ڼ��д������
	iap_write_bytes(0x200, eeprom_pid3_strline, 256);

	// sprintf(eeprom_pid4_strline, "%f,%f,%f,%f,%f,%f,",roundaboutL_PID.KI,roundaboutL_PID.KI,roundaboutL_PID.KD,roundaboutR_PID.KP,roundaboutR_PID.KI,roundaboutR_PID.KD);
	// printf("(write4%s)", eeprom_pid4_strline); // ��λ�����ڼ��д������
	// extern_iap_write_bytes(0x300, eeprom_pid4_strline, 256);
}
