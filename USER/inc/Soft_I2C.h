#ifndef SOFT_I2C_H_
#define SOFT_I2C_H_

#include "STC32Gxx.h"
#include "stdint.h"
#include "intrins.h"
// ����I2C���ƽ�
sbit I2C_SCL = P3 ^ 3; // SCLʱ����
sbit I2C_SDA = P3 ^ 5; // SDA������

// I2C���в�������
void i2c_start(void);       // ����I2C��ʼ�ź�
void i2c_stop(void);        // ����I2Cֹͣ�ź�
uint8_t i2c_wait_ack(void); // I2C�ȴ�ACK�ź�
void i2c_ack(void);         // I2C����ACK�ź�
void i2c_nack(void);        // I2C������ACK�ź�

void i2c_write_byte(uint8_t txd);                                                                         // I2C����һ���ֽ�
uint8_t i2c_read_byte(uint8_t ack);                                                                       // I2C��ȡһ���ֽ�
uint8_t i2c_mem_write(uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Len);              // I2C��ָ��������ָ���Ĵ�������д��
uint8_t i2c_mem_read(uint8_t DevAddress, uint8_t MemAddress, uint8_t *pBuffer, uint16_t Len);             // I2C��ָ��������ָ���Ĵ���������ȡ
uint8_t i2c_write_bit(uint8_t DevAddress, uint8_t addr, uint8_t bitNum, uint8_t Data);                    // д��8λ�Ĵ�����һ��λ
uint8_t i2c_write_bits(uint8_t DevAddress, uint8_t addr, uint8_t bitStart, uint8_t length, uint8_t Data); // д��8λ�Ĵ����Ķ��λ
uint8_t i2c_read_bit(uint8_t DevAddress, uint8_t addr, uint8_t bitNum, uint8_t *Data);                    // ��ȡһ��λ��8λ�����ļĴ���
uint8_t i2c_read_bits(uint8_t DevAddress, uint8_t addr, uint8_t bitStart, uint8_t length, uint8_t *Data); // ��ȡ8λ�Ĵ����Ķ��λ

#endif
