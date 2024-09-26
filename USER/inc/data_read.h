#ifndef _DATA_READ_H_
#define _DATA_READ_H_

#include "header.h"

#define WIRELESS_BUF_SIZE 48

extern uint8 wireless_buf_f[WIRELESS_BUF_SIZE],wireless_buf_b[WIRELESS_BUF_SIZE];
extern uint8 buf_connect_flag,buf_input_flag;
extern uint8 buffer_ptr;
extern uint8 send_flag;


void wireless_read(uint8 flag);
double myatof(const char *str);

#endif