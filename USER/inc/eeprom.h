#ifndef _EEPROM_H_
#define _EEPROM__H_
#include "common.h"
#include "zf_pwm.h"
#include "motor.h"
#include "zf_eeprom.h"
#include "zf_pwm.h"
#include "imu.h"

/********************************PID��ر������������־λ**********/
extern uint8 para_change_flag;


/********************************PID��ر���**********/
extern int rule_d[7];  // ģ������� D
extern float kp_m;     // ģ��kp���ֵ
extern float kd_m;     // ģ��kp���ֵ
extern float err;      // Ѱ�����
extern float err_last; // ��һ�ε�Ѱ�����
extern float angle_err;
extern float Kp, Kd, output;
extern float EC, E;
/***************************************************************************************************/

/********************************��̬�����������**********/
extern void write_all_par(); // ����eepormд������
extern void read_all_par();  // ������ȡ����
/***************************************************************************************************/

#endif
