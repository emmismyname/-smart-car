#ifndef _KEY_BOARD_H_
#define _KEY_BOARD_H_

#define KEY1_PIN P73
#define KEY2_PIN P71
#define KEY3_PIN P72
#define KEY4_PIN P70
// 定义拨码开关引脚
#define SW1_PIN P75
#define SW2_PIN P76

#include "common.h"

// 拨码开关状态变量
extern uint8 sw1_status;
extern uint8 sw2_status;

// 开关状态变量
extern uint8 key1_status;
extern uint8 key2_status;
extern uint8 key3_status;
extern uint8 key4_status;

// 上一次开关状态变量
extern uint8 key1_last_status;
extern uint8 key2_last_status;
extern uint8 key3_last_status;
extern uint8 key4_last_status;

// 开关标志位
extern uint8 key1_flag;
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;

extern int button_switch; // 第三和第四个按键的开关，调参开关开关与否

extern void Button_Scan();

#endif