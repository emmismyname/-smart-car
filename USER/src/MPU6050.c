#include "MPU6050.h"
#include "Soft_I2C.h"

void MPU_Delay_Ms(uint16_t ms) // 移植时需修改
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

// IIC连续写
// reg:要写入的寄存器地址
// len:要写入的长度
// buf:要写入的数据的首地址
// 返回值:0,正常
//     其他,错误代码
uint8_t MPU_Write_Len(uint8_t reg, uint8_t len, uint8_t *buf)
{
	return i2c_mem_write(MPU_ADDR, reg, buf, len);
}

// IIC连续读
// reg:要读取的寄存器地址
// len:要读取的长度
// buf:读取到的数据存储区
// 返回值:0,正常
//     其他,错误代码
uint8_t MPU_Read_Len(uint8_t reg, uint8_t len, uint8_t *buf)
{
	return i2c_mem_read(MPU_ADDR, reg, buf, len);
}

// IIC写一个字节
// reg:寄存器地址
// Data:数据
// 返回值:0,正常
//     其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t Data)
{
	return i2c_mem_write(MPU_ADDR, reg, &Data, 1);
}

// IIC读一个字节
// reg:寄存器地址
// 返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
	i2c_mem_read(MPU_ADDR, reg, &res, 1);
	return res;
}

// 写入8位寄存器的一个位
uint8_t MPU_Write_Bit(uint8_t addr, uint8_t bitNum, uint8_t Data)
{
	return i2c_write_bit(MPU_ADDR, addr, bitNum, Data);
}

// 写入8位寄存器的多个位
uint8_t MPU_Write_Bits(uint8_t addr, uint8_t bitStart, uint8_t length, uint8_t Data)
{
	return i2c_write_bits(MPU_ADDR, addr, bitStart, length, Data);
}

// 读取一个位从8位器件的寄存器
uint8_t MPU_Read_Bit(uint8_t addr, uint8_t bitNum, uint8_t *Data)
{
	return i2c_read_bit(MPU_ADDR, addr, bitNum, Data);
}

// 读取8位寄存器的多个位
uint8_t MPU_Read_Bits(uint8_t addr, uint8_t bitStart, uint8_t length, uint8_t *Data)
{
	return i2c_read_bits(MPU_ADDR, addr, bitStart, length, Data);
}

// 初始化MPU6050
// 返回值:0,成功
//     其他,错误代码
uint8_t MPU_Init(void)
{
	uint8_t res;
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
	MPU_Delay_Ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
	MPU_Set_Gyro_Fsr(3);					 // 陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					 // 加速度传感器,±2g
	MPU_Set_Rate(50);						 // 设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X01);	 // 使能数据就绪中断
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	 // 关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if (res == MPU_ADDR) // 器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
		MPU_Set_Rate(200);						 // 设置采样率为200Hz
	}
	else
		return 1;
	return 0;
}

// 设置MPU6050陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}

// 设置MPU6050加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); // 设置加速度传感器满量程范围
}

// 设置MPU6050的数字低通滤波器
// lpf:数字低通滤波频率(Hz)
// 返回值:0,设置成功
//     其他,设置失败
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
	return MPU_Write_Byte(MPU_CFG_REG, Data); // 设置数字低通滤波器
}

// 设置MPU6050的采样率(假定Fs=1KHz)
// rate:4~1000(Hz)
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t Data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	Data = 1000 / rate - 1;
	Data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, Data); // 设置数字低通滤波器
	return MPU_Set_LPF(rate / 2);					  // 自动设置LPF为采样率的一半
}

// 得到温度值
// 返回值:温度值(扩大了100倍)
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

// 得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
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

// 得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
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
