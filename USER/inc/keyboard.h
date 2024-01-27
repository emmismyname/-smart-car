#ifndef _KEY_BOARD_H_
#define _KEY_BOARD_H_

#define KEY1_PIN P73
#define KEY2_PIN P71
#define KEY3_PIN P72
#define KEY4_PIN P70
// ���岦�뿪������
#define SW1_PIN P75
#define SW2_PIN P76

#include "common.h"

// ���뿪��״̬����
extern uint8 sw1_status;
extern uint8 sw2_status;

// ����״̬����
extern uint8 key1_status;
extern uint8 key2_status;
extern uint8 key3_status;
extern uint8 key4_status;

// ��һ�ο���״̬����
extern uint8 key1_last_status;
extern uint8 key2_last_status;
extern uint8 key3_last_status;
extern uint8 key4_last_status;

// ���ر�־λ
extern uint8 key1_flag;
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;

extern int button_switch; // �����͵��ĸ������Ŀ��أ����ο��ؿ������

extern void Button_Scan();

#endif