#ifndef _DATA_H_
#define _DATA_H_

// #include "header.h"

#include "common.h"
#include "SEEKFREE_WIRELESS.h"
#include "eeprom.h"
#include "zf_delay.h"
#include "zf_spi.h"
#include "car_state.h"
#include "carcontrol.h"
#include "eeprom.h"

#define SS_2 P22
#define RecBufSize 20


typedef union // 定义float，longint共用体
{
    float fdata;
    unsigned long int ldata;
} FloatLongType;


extern bit spi_busy;
extern uint8 Receive_buff[RecBufSize];
extern uint8 Receive_ptr;
extern const uint8 SPIFrameFloat[2];
extern const uint8 SPIFrameUint[2];
extern const uint8 SPIFrameUchar[2];
extern float SPI_float;
extern uint16 SPI_uint;
extern uint8 SPI_uchar;
extern uint8 uart_mode;

//wireless_send
void floatToByte(float fnum, unsigned char byte[]);// 浮点数转二进制数
void wireless_uart_send(float num[], uint8 len);// 串口发送 
void wireless_send(int mode, int delayTime);        //无线串口发送函数
//wireless_read 
double myatof(const char *str); // 字符串转浮点数函数
void wireless_read(uint8 flag); // 无线转串口读取调参函数

//spi connect
void spi_master_init();						// SPI初始化(作为主机)
void spi_slave_init();						// SPI初始化(作为从机)
void EnableSPIIRQ(void);					// 使能SPI中断
void spi_send_byte(uint8 dat);
void spi_send_buffer(unsigned char buff[], uint8 len);
void spi_send_float(float num);
void spi_send_uint(uint16 num);
void spi_send_uchar(uint8 num);


#endif