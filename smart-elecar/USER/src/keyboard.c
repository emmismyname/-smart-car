#include "keyboard.h"

uint8 sw1_status;
uint8 sw2_status;

// ����״̬����
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

// ��һ�ο���״̬����
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;

// ���ر�־λ
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;

int button_switch = 1; // �����͵��ĸ������Ŀ��أ����ο��ؿ������