#ifndef _DATA_SEND_H_
#define _DATA_SEND_H_

#include "header.h"

#define SS_2 P22
#define RecBufSize 20

extern bit spi_busy;
extern uint8 Receive_buff[RecBufSize];
extern uint8 Receive_ptr;
extern const uint8 SPIFrameFloat[2];
extern const uint8 SPIFrameUint[2];
extern const uint8 SPIFrameUchar[2];
extern float SPI_float;
extern uint16 SPI_uint;
extern uint8 SPI_uchar;

void spi_send_byte(uint8 dat);
void spi_send_buffer(unsigned char buff[], uint8 len);
void spi_send_float(float num);
void spi_send_uint(uint16 num);
void spi_send_uchar(uint8 num);

void wireless_send_SBtoPC(void);

#endif