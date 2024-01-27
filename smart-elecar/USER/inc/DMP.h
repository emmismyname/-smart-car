#ifndef DMP_H_
#define DMP_H_

#include "Soft_I2C.h"
#include "MPU6050.h"
#include <stdio.h>

extern float Q[4];           // ��Ԫ��
extern uint8_t dmpdatas[42]; // DMP����

void MPU6050_DMP_Init(void);
void MPU6050_Refresh_DMP(void);

#endif
