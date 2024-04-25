
/*********************************************************************************************************************
 * @file       		init
 * @date       		2024-03-06
 * @note    
 ********************************************************************************************************************/


#ifndef _INIT_H_
#define _INIT_H_

#include "headfile.h"

#define LED P50

void allADC_init(void);
void allPins_init(void);
void nvic_init(void);
void spi_master_init();						// SPI��ʼ��(��Ϊ����)
void spi_slave_init();						// SPI��ʼ��(��Ϊ�ӻ�)
void EnableSPIIRQ(void);					// ʹ��SPI�ж�

#endif