#include "uart_send.h"

typedef union // 定义float，longint共用体
{
    float fdata;
    unsigned long int ldata;
} FloatLongType;
float dat[64];                               // 定义数据体
char datetail[4] = {0x00, 0x00, 0x80, 0x7f}; // 定义帧尾

int uart_mode = 0; // 设置串口传输模式

//-------------------------------------------------------------------------------------------------------------------
//  @brief      浮点数转二进制数
//  @param      fnum			输入的浮点数
//  @param      byte[]			转换成的二进制数组
//  @return
//-------------------------------------------------------------------------------------------------------------------
void floatToByte(float fnum, unsigned char byte[]) // 浮点数转二进制数
{
    FloatLongType fl;
    fl.fdata = fnum;
    byte[0] = (unsigned char)(fl.ldata);
    byte[1] = (unsigned char)(fl.ldata >> 8);
    byte[2] = (unsigned char)(fl.ldata >> 16);
    byte[3] = (unsigned char)(fl.ldata >> 24);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      浮点数组转化为二进制数组并发送
//  @param      num[]			输入数组
//  @param      len             数组长度
//  @return
//-------------------------------------------------------------------------------------------------------------------
void wireless_uart_send(float num[], uint8 len) // 串口发送函数
{
    int i, j;
    char hexdats[256];
    // 浮点数组转化为二进制数组
    for (i = 0; i < len; i++)
    {
        uint8 bytedat[4] = {0};
        floatToByte(num[i], bytedat);
        for (j = i * 4; j <= i * 4 + 3; j++)
            hexdats[j] = bytedat[j % 4];
    }
    // 发送数据体
    wireless_uart_send_buff(hexdats, sizeof(float) * len);
    // 发送帧尾
    wireless_uart_send_buff(datetail, 4);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      无线串口发送函数
//  @param      mode			发送模式
//  @param      delayTime       单次发送延时时间(毫秒)
//  @return
//-------------------------------------------------------------------------------------------------------------------
void wireless_send(int mode, int delayTime) // 无线串口发送函数
{
    switch (mode)
    {
    case 1:
    {
        // 定义要发送的数据存入数组
        dat[0] = adc_L1;
        dat[1] = adc_L2;
        dat[2] = adc_M;
        dat[3] = adc_R2;
        dat[4] = adc_R1;

        dat[5] = adc_2_L1;
        dat[6] = adc_2_L2;
        dat[7] = adc_2_M;
        dat[8] = adc_2_R2;
        dat[9] = adc_2_R1;

        dat[10] = Target_speed;
        dat[11] = Target_Speed_L;
        dat[12] = Target_Speed_R;
        dat[13] = Current_Speed_L;
        dat[14] = Current_Speed_R;
        dat[15] = gyro_flag;
        dat[16] = yaw;
        dat[17] = pitch;
        dat[18] = roll;
        dat[19] = yaw_output;
        dat[20] = forward_err;
        dat[21] = turn_err;

        dat[22] = running_mode;
        dat[23] = trace_flag;
        dat[24] = Element_flag;
        dat[25] = z_gyro;
        // 发送数据
        wireless_uart_send(dat, 26);
        delay_ms(delayTime);
    }
    break;
    case 2:
    {
        // 定义要发送的数据存入数组
        dat[0] = (float)adc[1].data_min;
        dat[1] = (float)adc[2].data_min;
        dat[2] = (float)adc[3].data_min;
        dat[3] = (float)adc[4].data_min;
        dat[4] = (float)adc[5].data_min;

        dat[5] = (float)adc[6].data_min;
        dat[6] = (float)adc[7].data_min;
        dat[7] = (float)adc[8].data_min;
        dat[8] = (float)adc[9].data_min;
        dat[9] = (float)adc[10].data_min;

        dat[10] = (float)adc[1].data_max;
        dat[11] = (float)adc[2].data_max;
        dat[12] = (float)adc[3].data_max;
        dat[13] = (float)adc[4].data_max;
        dat[14] = (float)adc[5].data_max;

        dat[15] = (float)adc[6].data_max;
        dat[16] = (float)adc[7].data_max;
        dat[17] = (float)adc[8].data_max;
        dat[18] = (float)adc[9].data_max;
        dat[19] = (float)adc[10].data_max;

        dat[20] = (float)resetelec_count;
        dat[21] = (float)resetelec_flag;
        // 发送数据
        wireless_uart_send(dat, 22);
        delay_ms(delayTime);
    }
    break;
    case 3:
    {
        // 定义要发送的数据存入数组
        dat[0] = adc[1].data_result;
        dat[1] = adc[2].data_result;
        dat[2] = adc[3].data_result;
        dat[3] = adc[4].data_result;
        dat[4] = adc[5].data_result;

        dat[5] = adc[6].data_result;
        dat[6] = adc[7].data_result;
        dat[7] = adc[8].data_result;
        dat[8] = adc[9].data_result;
        dat[9] = adc[10].data_result;

        dat[10] = err;
        dat[11] = err_last;
        dat[12] = Err_1;
        dat[13] = Err_2;
        dat[14] = 0;

        // dat[15] = brusheless_flag;
        // dat[16] = brusheless_duty;
        // 发送数据
        wireless_uart_send(dat, 15);
        delay_ms(delayTime);
        // 发送数据
    }
    break;
    case 4:
    { // 电感
          // 定义要发送的数据存入数组
        dat[0] = adc_L1;
        dat[1] = adc_L2;
        dat[2] = adc_M;
        dat[3] = adc_R2;
        dat[4] = adc_R1;

        dat[5] = adc_2_L1;
        dat[6] = adc_2_L2;
        dat[7] = adc_2_M;
        dat[8] = adc_2_R2;
        dat[9] = adc_2_R1;
        dat[10] = adc_2_L2-adc_2_R2;
        dat[11] = adc_M-adc_2_M;
        dat[12] = Target_speed;
        dat[13] = Target_Speed_L;
        dat[14] = Target_Speed_R;
        dat[15] = Current_Speed_L;
        dat[16] = Current_Speed_R;
        dat[17] = err;
        dat[18] = Err_2_Hori;
        dat[19] = Err_2_Vert;
        dat[20] = Err_Hori;
        dat[21] = Err_Vert;

        dat[22] = yaw_output;
        dat[23] = yaw;
        dat[24] = Element_flag;
        dat[25] = z_gyro;
        // // 发送数据
        wireless_uart_send(dat, 26);
        delay_ms(delayTime);
        // // 发送数据
    }
    break;
    case 5:
    {
        dat[0] = yaw;
        dat[1] = roll;
        dat[2] = pitch;
        dat[3] = yaw_output;
        dat[4] = adc_R1;

        dat[5] = adc_2_L1;
        dat[6] = adc_2_L2;
        dat[7] = adc_2_M;
        dat[8] = adc_2_R2;
        dat[9] = adc_2_R1;

        dat[10] = pitch;
        dat[11] = yaw_output;
        dat[12] = err;
        dat[13] = Target_Speed_L;
        dat[14] = Target_Speed_R;

        dat[15] = Element_flag;
        dat[16] = leftRound;
        dat[17] = rightRound;
        dat[18] = roundcount;
        dat[19] = UpSlope;
        dat[20] = running_mode;
        dat[21] = dl1a_distance_mm;
        dat[22] = distance_L;
        dat[23] = distance_R;
        // 发送数据
        wireless_uart_send(dat, 24);
        delay_ms(delayTime);
        // 发送数据
    }
    break;
    case 6:
    {
        // 定义要发送的数据存入数组
        dat[0] = Q[0];
        dat[1] = Q[1];
        dat[2] = Q[2];
        dat[3] = Q[3];
        dat[4] = pitch;
        dat[5] = roll;
        dat[6] = yaw;
        dat[7] = pitch_result;
        dat[8] = roll_result;
        dat[9] = yaw_result;
        // 发送数据
        wireless_uart_send(dat, 10);
        delay_ms(delayTime);
        // 发送数据
    }
    break;
    case 7:
    { // 定义要发送的数据存入数组
        dat[0] = adc_L1;
        dat[1] = adc_L2;
        dat[2] = adc_M;
        dat[3] = adc_R2;
        dat[4] = adc_R1;
        dat[5] = adc_2_L1;
        dat[6] = adc_2_L2;
        dat[7] = adc_2_M;
        dat[8] = adc_2_R2;
        dat[9] = adc_2_R1;
        dat[10] = Element_flag;
        dat[11] = RoundAbout_small_left;
        dat[12] = RoundAbout_small_right;
        dat[13] = running_mode;
        dat[14] = Target_speed;
        dat[15] = Target_Speed_L;
        dat[16] = Target_Speed_R;
        dat[17] = err;
        dat[18] = yaw_output;
        dat[19] = z_gyro;
        dat[20] = UpSlope;
        wireless_uart_send(dat, 21);
        delay_ms(delayTime);
    }
    break;
    case 8:
    { // 定义要发送的数据存入数组
        dat[0] = yaw_output;
        dat[1] = yaw_target;
        dat[2] = yaw_target - yaw_output;
        dat[3] = Pout_PID.p_out;
        dat[4] = z_gyro;
        dat[5] = 0;
        dat[6] = 0;
        dat[7] = Pout_PID.p_out-z_gyro;
        dat[8] = Pout_rate_PID.p_out;
    
   
        dat[9] = 0;
        dat[10] = 0;

        dat[11] = HRTRIG_flag;
        dat[12] = outward_basin;
        dat[13] = running_mode;
        dat[14] = Target_Speed_L;
        dat[15] = Target_Speed_R;
        dat[16] = Current_Speed_L;
        dat[17] = Current_Speed_R;
        
        dat[18] = avoid_move;
        dat[19] = dl1a_distance_mm;
        dat[20] = block_timer_count;
        dat[21] = blockcount_flag;
        dat[22] = block_count;
        dat[23] = TOF_object;
        wireless_uart_send(dat, 24);
        delay_ms(delayTime);
    }
    break;
    case 9:
    { // 定义要发送的数据存入数组
        dat[0] = distance_L;
        dat[1] = distance_R;
        dat[2] = distance_break;
        dat[3] = outp_angle;
        dat[4] = outp_distance;
        dat[5] = outp_radiu;
        dat[6] = OUTP_flag;
        dat[7] = yaw_output;
        dat[8] = intop_angle;
        dat[9] = intop_distance;
        dat[10] = intop_radiu;
   
        dat[11] = HRTRIG_flag;
        dat[12] = outward_basin;
        dat[13] = running_mode;
        dat[14] = Target_Speed_L;
        dat[15] = Target_Speed_R;
        dat[16] = Current_Speed_L;
        dat[17] = Current_Speed_R;
        
        dat[18] = avoid_move;
        dat[19] = dl1a_distance_mm;
        dat[20] = block_timer_count;
        dat[21] = blockcount_flag;
        dat[22] = block_count;
        dat[23] = TOF_object;
        dat[24] = isr_count_flag;
        dat[25] = isr_counter_start;
        wireless_uart_send(dat, 26);
        delay_ms(delayTime);
    }
    break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      无线转串口发送
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
// void wireless_send(uint8 frequency, int mode)
// {
//     static int8 send_adc[200], send_velocity[100], send_duty[50], send_other[100]; // 发送的电感，速度数据，左右电机占空比，或测试的其他数据
//     if (count_flag % frequency == 0)                                               // frequency改变发送频率
//     {
//         switch (mode) // mode=1发送电感数据，mode=2发送速度数据，mode=3发送电机占空比数据
//         {
//         case 1:
//         {
//             sprintf(send_adc, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", adc[1].data_0, adc[2].data_0, adc[3].data_0, adc[4].data_0, adc[5].data_0, adc[6].data_0, adc[7].data_0, adc[8].data_0, adc[9].data_0, running_mode);
//             // sprintf(send_adc,"%f,%f,%f,%f,%f,%f,%f\n",adc_L1,adc_L2,adc_L3,adc_M,adc_R3,adc_R2,adc_R1);
//             wireless_uart_send_buff(send_adc, strlen(send_adc));
//             break;
//         }
//         case 2:
//         {
//             sprintf(send_velocity, "%d,%f,%f,%f,%f,%f,%d\n", Target_speed, Target_Speed_L, Target_Speed_R, Current_Speed_L, Current_Speed_R, err, running_mode);
//             wireless_uart_send_buff(send_velocity, strlen(send_velocity));
//             break;
//         }
//         case 3:
//         {
//             sprintf(send_duty, "%f,%f\n", duty_L, duty_R);
//             wireless_uart_send_buff(send_duty, strlen(send_duty));
//             break;
//         }
//         case 4:
//         {
//             // 目前测试目标速度，现速度，电机占空比
//             sprintf(send_other, "%d,%d,%d,%d,%d,%d,%d,%f,%f\n", Element_flag, RoundAbout, Roundflag, roundcount, OUTP_flag, gyro_flag, running_mode, yaw, pitch);
//             wireless_uart_send_buff(send_other, strlen(send_other));
//             // sprintf(send_other,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",adc_L1,adc_L2,adc_L3,adc_M,adc_R3,adc_R2,adc_R1,Err_Hori,Err_Vert,yaw);
//             // wireless_uart_send_buff(send_other,strlen(send_other));
//             break;
//         }
//         case 5:
//         {
//             // 目前测试目标速度，现速度，电机占空比
//             //  ips114_showstr(15, 2, "distance_L");		ips114_showfloat(8*16, 2, distance_L,5,2);
//             //  ips114_showstr(15, 3, "distance_R");		ips114_showfloat(8*16, 3, distance_R,4,2);
//             //  ips114_showstr(15, 4, "Tgt_Spd_L=");		ips114_showfloat(8*16, 4, Target_Speed_L,3,3);
//             //  ips114_showstr(15, 5, "Tgt_Spd_R=");		ips114_showfloat(8*16, 5, Target_Speed_R,3,3);
//             //  ips114_showstr(15, 6, "yaw");		ips114_showfloat(8*16, 6, yaw,4,2);
//             sprintf(send_other, "%d,%d,%f,%f,%f\n", distance_L, distance_R, Target_Speed_L, Target_Speed_R, yaw);
//             wireless_uart_send_buff(send_other, strlen(send_other));
//             // sprintf(send_other,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",adc_L1,adc_L2,adc_L3,adc_M,adc_R3,adc_R2,adc_R1,Err_Hori,Err_Vert,yaw);
//             // wireless_uart_send_buff(send_other,strlen(send_other));
//             break;
//         }
//         case 6:
//         {
//             sprintf(send_adc, "%f,%f,%f,%f,%f,%f,%f,%f\n", adc_L1, adc_L2, adc_L3, adc_M, adc_R3, adc_R2, adc_R1, yaw);
//             wireless_uart_send_buff(send_adc, strlen(send_adc));
//             break;
//         }
//         case 7:
//         {
//             sprintf(send_adc, "%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f\n", Element_flag, RoundAbout, Roundflag, roundcount, yaw);
//             // sprintf(send_adc,"%f,%f,%f,%f,%f,%f,%f\n",adc_L1,adc_L2,adc_L3,adc_M,adc_R3,adc_R2,adc_R1);
//             wireless_uart_send_buff(send_adc, strlen(send_adc));
//             break;
//         }
//         case 8:
//         {
//             sprintf(send_adc, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", adc[1].data_0, adc[2].data_0, adc[3].data_0, adc[4].data_0, adc[5].data_0, adc[6].data_0, adc[7].data_0, adc[8].data_0, adc[9].data_0, running_mode);
//             // sprintf(send_adc,"%f,%f,%f,%f,%f,%f,%f\n",adc_L1,adc_L2,adc_L3,adc_M,adc_R3,adc_R2,adc_R1);
//             wireless_uart_send_buff(send_adc, strlen(send_adc));
//             break;
//         }
//         case 9:
//         {
//             sprintf(send_adc, "%f,%f,%f\n", Err_Hori, Err_2_Hori, Err_Vert);
//             // sprintf(send_adc,"%f,%f,%f,%f,%f,%f,%f\n",adc_L1,adc_L2,adc_L3,adc_M,adc_R3,adc_R2,adc_R1);
//             wireless_uart_send_buff(send_adc, strlen(send_adc));
//             break;
//         }
//         default:
//         {
//             break;
//         }
//         }
//     }
// }

//-------------------------------------------------------------------------------------------------------------------
//  @brief      无线转串口接收
//  @param
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
// void wireless_read()
// {
//     uint8 read_strline[256], sss[256]; // 读取PID的缓存内容，后续将转化为float
//     int temp_motor, temp_mode;
//     // float tempf_1,tempf_2,tempf_3,tempf_4,tempf_5,tempf_6;
//     float tempf_1, tempf_2, tempf_3;
//     date_len = wireless_uart_read_buff(read_strline, sizeof(read_strline) - 1);
//     delay_ms(50);
//     if (date_len != 0)
//     {
//         strcpy(sss, read_strline);
//         printf("%s", sss);
//         sscanf(sss, "%d,%d,%d,%f,%f,%f", &key, &temp_motor, &temp_mode, &tempf_1, &tempf_2, &tempf_3);
//         if (key == 104)
//         {
//             motor_flag = temp_motor;
//             key_mode = temp_mode;
//             switch (key_mode)
//             {
//             case 1: // 方向环
//             {
//                 Forward_PID.KP = tempf_1;
//                 Forward_PID.KI = tempf_2;
//                 Forward_PID.KD = tempf_3;
//             }
//             break;
//             case 2:
//             {
//                 Swerve_PID.KP = tempf_1;
//                 Swerve_PID.KI = tempf_2;
//                 Swerve_PID.KD = tempf_3;
//             }
//             break;
//             case 3:
//             {
//                 Speed_PID_L.KP = tempf_1;
//                 Speed_PID_L.KI = tempf_2;
//                 Speed_PID_L.KD = tempf_3;
//             }
//             break;
//             case 4:
//             {
//                 Speed_PID_R.KP = tempf_1;
//                 Speed_PID_R.KI = tempf_2;
//                 Speed_PID_R.KD = tempf_3;
//             }
//             break;
//             case 5:
//             {
//                 Pout_PID.KP = tempf_1;
//                 Pout_PID.KI = tempf_2;
//                 Pout_PID.KD = tempf_3;
//             }
//             case 6:
//             {
//                 gyro_d = tempf_1;
//             }
//             break;
//             default:
//                 break;
//             }
//         }
//     }
//     wireless_uart_send_buff(test_str, sizeof(test_str));
//     // 将读取到的fifo发送出wireless uart send buff(read buf, (uint16)dat len);
//     uint8 read_strline[256],
//         sss[256]; // 读取PID的缓存内容，后续将转化为float
//     int temp_motor;
//     date_len = wireless_uart_read_buff(read_strline, sizeof(read_strline) - 1);
//     delay_ms(50);
//     if (date_len != 0)
//     {
//         strcpy(sss, read_strline);
//         printf("%s", sss);
//         sscanf(sss, "%d,%d", &key, &temp_motor);
//         if (key == 104)
//         {
//             trace_flag = temp_motor;
//             switch (key_mode)
//             {
//             case 1: // 方向环
//             {
//                 Forward_PID.KP = tempf_1;
//                 Forward_PID.KI = tempf_2;
//                 Forward_PID.KD = tempf_3;
//             }
//             break;
//             case 2:
//             {
//                 Swerve_PID.KP = tempf_1;
//                 Swerve_PID.KI = tempf_2;
//                 Swerve_PID.KD = tempf_3;
//             }
//             break;
//             case 3:
//             {
//                 Speed_PID_L.KP = tempf_1;
//                 Speed_PID_L.KI = tempf_2;
//                 Speed_PID_L.KD = tempf_3;
//             }
//             break;
//             case 4:
//             {
//                 Speed_PID_R.KP = tempf_1;
//                 Speed_PID_R.KI = tempf_2;
//                 Speed_PID_R.KD = tempf_3;
//             }
//             break;
//             case 5:
//             {
//                 Pout_PID.KP = tempf_1;
//                 Pout_PID.KI = tempf_2;
//                 Pout_PID.KD = tempf_3;
//             }
//             case 6:
//             {
//                 gyro_d = tempf_1;
//             }
//             break;
//             default:
//             {
//                 break;
//             }
//             }
//         }
//     }
// }