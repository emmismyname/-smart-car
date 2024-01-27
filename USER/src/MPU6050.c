#include "MPU6050.h"
#include "Soft_I2C.h"

void MPU_Delay_Ms(uint16_t ms) // ��ֲʱ���޸�
{
	unsigned int i;
	do
	{
		// i = MAIN_Fosc / 96000L;  // STC89_12T
		// i = MAIN_Fosc / 48000L;  // STC89_6T
		// i = MAIN_Fosc / 14000;	// STC12
		// i = MAIN_Fosc / 13000;	// STC15
		// i = MAIN_Fosc / 10000;	// STC8
		i = 35000000 / 6030; // STC32
		while (--i)
			;
	} while (--ms);
}

// IIC����д
// reg:Ҫд��ļĴ�����ַ
// len:Ҫд��ĳ���
// buf:Ҫд������ݵ��׵�ַ
// ����ֵ:0,����
//     ����,�������
uint8_t MPU_Write_Len(uint8_t reg, uint8_t len, uint8_t *buf)
{
	return i2c_mem_write(MPU_ADDR, reg, buf, len);
}

// IIC������
// reg:Ҫ��ȡ�ļĴ�����ַ
// len:Ҫ��ȡ�ĳ���
// buf:��ȡ�������ݴ洢��
// ����ֵ:0,����
//     ����,�������
uint8_t MPU_Read_Len(uint8_t reg, uint8_t len, uint8_t *buf)
{
	return i2c_mem_read(MPU_ADDR, reg, buf, len);
}

// IICдһ���ֽ�
// reg:�Ĵ�����ַ
// Data:����
// ����ֵ:0,����
//     ����,�������
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t Data)
{
	return i2c_mem_write(MPU_ADDR, reg, &Data, 1);
}

// IIC��һ���ֽ�
// reg:�Ĵ�����ַ
// ����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
	i2c_mem_read(MPU_ADDR, reg, &res, 1);
	return res;
}

// д��8λ�Ĵ�����һ��λ
uint8_t MPU_Write_Bit(uint8_t addr, uint8_t bitNum, uint8_t Data)
{
	return i2c_write_bit(MPU_ADDR, addr, bitNum, Data);
}

// д��8λ�Ĵ����Ķ��λ
uint8_t MPU_Write_Bits(uint8_t addr, uint8_t bitStart, uint8_t length, uint8_t Data)
{
	return i2c_write_bits(MPU_ADDR, addr, bitStart, length, Data);
}

// ��ȡһ��λ��8λ�����ļĴ���
uint8_t MPU_Read_Bit(uint8_t addr, uint8_t bitNum, uint8_t *Data)
{
	return i2c_read_bit(MPU_ADDR, addr, bitNum, Data);
}

// ��ȡ8λ�Ĵ����Ķ��λ
uint8_t MPU_Read_Bits(uint8_t addr, uint8_t bitStart, uint8_t length, uint8_t *Data)
{
	return i2c_read_bits(MPU_ADDR, addr, bitStart, length, Data);
}

// ��ʼ��MPU6050
// ����ֵ:0,�ɹ�
//     ����,�������
uint8_t MPU_Init(void)
{
	uint8_t res;
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // ��λMPU6050
	MPU_Delay_Ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // ����MPU6050
	MPU_Set_Gyro_Fsr(3);					 // �����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					 // ���ٶȴ�����,��2g
	MPU_Set_Rate(50);						 // ���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X01);	 // ʹ�����ݾ����ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	 // �ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT���ŵ͵�ƽ��Ч
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if (res == MPU_ADDR) // ����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // ����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // ���ٶ��������Ƕ�����
		MPU_Set_Rate(200);						 // ���ò�����Ϊ200Hz
	}
	else
		return 1;
	return 0;
}

// ����MPU6050�����Ǵ����������̷�Χ
// fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
// ����ֵ:0,���óɹ�
//     ����,����ʧ��
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); // ���������������̷�Χ
}

// ����MPU6050���ٶȴ����������̷�Χ
// fsr:0,��2g;1,��4g;2,��8g;3,��16g
// ����ֵ:0,���óɹ�
//     ����,����ʧ��
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); // ���ü��ٶȴ����������̷�Χ
}

// ����MPU6050�����ֵ�ͨ�˲���
// lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
// ����ֵ:0,���óɹ�
//     ����,����ʧ��
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t Data = 0;
	if (lpf >= 188)
		Data = 1;
	else if (lpf >= 98)
		Data = 2;
	else if (lpf >= 42)
		Data = 3;
	else if (lpf >= 20)
		Data = 4;
	else if (lpf >= 10)
		Data = 5;
	else
		Data = 6;
	return MPU_Write_Byte(MPU_CFG_REG, Data); // �������ֵ�ͨ�˲���
}

// ����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
// rate:4~1000(Hz)
// ����ֵ:0,���óɹ�
//     ����,����ʧ��
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t Data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	Data = 1000 / rate - 1;
	Data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, Data); // �������ֵ�ͨ�˲���
	return MPU_Set_LPF(rate / 2);					  // �Զ�����LPFΪ�����ʵ�һ��
}

// �õ��¶�ֵ
// ����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
	uint8_t buf[2];
	short raw;
	float temp;
	MPU_Read_Len(MPU_TEMP_OUTH_REG, 2, buf);
	raw = ((uint16_t)buf[0] << 8) | buf[1];
	temp = 36.53 + ((double)raw) / 340;
	return temp * 100;
}

// �õ�������ֵ(ԭʼֵ)
// gx,gy,gz:������x,y,z���ԭʼ����(������)
// ����ֵ:0,�ɹ�
//     ����,�������
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*gx = ((uint16_t)buf[0] << 8) | buf[1];
		*gy = ((uint16_t)buf[2] << 8) | buf[3];
		*gz = ((uint16_t)buf[4] << 8) | buf[5];
	}
	return res;
	;
}

// �õ����ٶ�ֵ(ԭʼֵ)
// gx,gy,gz:������x,y,z���ԭʼ����(������)
// ����ֵ:0,�ɹ�
//     ����,�������
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*ax = ((uint16_t)buf[0] << 8) | buf[1];
		*ay = ((uint16_t)buf[2] << 8) | buf[3];
		*az = ((uint16_t)buf[4] << 8) | buf[5];
	}
	return res;
	;
}
