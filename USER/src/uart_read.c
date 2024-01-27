#include "uart_read.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ת���ڶ�ȡ����
//  @param      flag			��ȡ������ر�
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void wireless_read(uint8 flag)
{
    if (flag)
    {
        static uint8 read_buf[50];                  // ��ȡ����buffer
        static uint32 read_len = 0;                 // ��ȡ����buffer����
        static char read_dat_string[50];            // ��ȡ�����ַ���
        static float read_dat_float;                // ��ȡ�����ַ����еĸ�����
        static char send_dat[20];                   // ���ͻ���λ���������
        char mode1 = '0', mode2 = '0', mode3 = '0'; // ����ģʽ
        // ��ȡbuffer���õ�����
        read_len = wireless_uart_read_buff(read_buf, 50);
        // bufferת��Ϊ�ַ���
        strcpy(read_dat_string, read_buf);
        // ��ȡ������
        read_dat_float = myatof(read_dat_string);
        // ���յ�����ʱ
        if (read_len != 0)
        {
            // ģʽλ1
            mode1 = read_dat_string[0];
            switch (mode1)
            {
            case 'o': // ����
            {
                mode2 = read_dat_string[2]; // ģʽλ2
                switch (mode2)
                {
                case 'g': // ������־
                {
                    start_flag = 1;
                }
                break;
                case 's': // ͣ����־
                {
                    stop_flag = 1;
                }
                break;
                case 'd': // ����ͣ����־
                {
                    base_speed = read_dat_float;
                }
                break;
                }
                // ���ͻ���λ������
                sprintf(send_dat, "%c %c\n", mode1, mode2);
                wireless_uart_send_buff(send_dat, strlen(send_dat));
                write_all_par();
            }
            break;
            case 'n': // ��Ԫ�ز���
            {
                mode2 = read_dat_string[2]; // ģʽλ2
                switch (mode2)
                {
                case 's': // �ٶȻ�����
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'p':
                    {
                        Speed_PID_L.KP = read_dat_float;
                    }
                    break;
                    case 'i':
                    {
                        Speed_PID_L.KI = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        Speed_PID_L.KD = read_dat_float;
                    }
                    break;
                    }
                }
                break;
                case 'z': // �ٶȻ�����
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'p':
                    {
                        Speed_PID_R.KP = read_dat_float;
                    }
                    break;
                    case 'i':
                    {
                        Speed_PID_R.KI = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        Speed_PID_R.KD = read_dat_float;
                    }
                    break;
                    }
                }
                break;
                case 'f': // ֱ�л�
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'p':
                    {
                        Forward_PID.KP = read_dat_float;
                    }
                    break;
                    case 'i':
                    {
                        Forward_PID.KI = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        Forward_PID.KD = read_dat_float;
                    }
                    break;
                    }
                }
                break;
                case 'c': // �����
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'p':
                    {
                        Swerve_PID.KP = read_dat_float;
                    }
                    break;
                    case 'i':
                    {
                        Swerve_PID.KI = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        Swerve_PID.KD = read_dat_float;
                    }
                    break;
                    }
                }
                break;
                case 'a': // �ǶȻ�
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'p':
                    {
                        Pout_PID.KP = read_dat_float;
                    }
                    break;
                    case 'i':
                    {
                        Pout_PID.KI = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        Pout_PID.KD = read_dat_float;
                    }
                    break;
                    }
                }
                break;
                case 'o': // ����
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'a':
                    {
                        outp_angle = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        outp_distance = read_dat_float;
                    }
                    break;
                    case 'r':
                    {
                        outp_radiu = read_dat_float;
                    }
                    break;
                    }
                }
                break;
                case 'i': // ���
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'a':
                    {
                        intop_angle = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        intop_distance = read_dat_float;
                    }
                    break;
                    case 'r':
                    {
                        intop_radiu = read_dat_float;
                    }
                    break;
                    case 'b':
                    {
                        distance_break = read_dat_float;
                    }
                    }
                }
                break;
                case 't': // �ǶȻ�
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'p':
                    {
                        Pout_rate_PID.KP = read_dat_float;
                    }
                    break;
                    case 'i':
                    {
                        Pout_rate_PID.KI = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        Pout_rate_PID.KD = read_dat_float;
                    }
                    break;
                    }
                }
                break;
                }
                // ���ͻ���λ������
                sprintf(send_dat, "%c_%c_%c=%f\n", mode1, mode2, mode3, read_dat_float);
                wireless_uart_send_buff(send_dat, strlen(send_dat));
                write_all_par();
            }
            break;
            case 'e': // ��Ԫ�ز���
            {
                mode2 = read_dat_string[2]; // ģʽλ2
                switch (mode2)
                {
                case 'l': // ��С��������
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'p':
                    {
                        roundaboutL_PID.KP = read_dat_float;
                    }
                    break;
                    case 'i':
                    {
                        roundaboutL_PID.KI = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        roundaboutL_PID.KD = read_dat_float;
                    }
                    break;
                    }
                }
                break;
                case 'r': // ��С��������
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'p':
                    {
                        roundaboutR_PID.KP = read_dat_float;
                    }
                    break;
                    case 'i':
                    {
                        roundaboutR_PID.KI = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        roundaboutR_PID.KD = read_dat_float;
                    }
                    break;
                    }
                }
                break;
                case 'c':
                {
                    mode3 = read_dat_string[4]; // ģʽλ3
                    switch (mode3)
                    {
                    case 'p':
                    {
                        climb_PID.KP = read_dat_float;
                    }
                    break;
                    case 'i':
                    {
                        climb_PID.KI = read_dat_float;
                    }
                    break;
                    case 'd':
                    {
                        climb_PID.KD = read_dat_float;
                    }
                    break;
                    }
                    break;
                }
                case 't':
                {
                    yaw_target = read_dat_float;
                }
                break;
                }
                // ���ͻ���λ������
                sprintf(send_dat, "%c_%c_%c=%f\n", mode1, mode2, mode3, read_dat_float);
                wireless_uart_send_buff(send_dat, strlen(send_dat));
                write_all_par();
            }
            break;
            case 'c':
            {
                mode2 = read_dat_string[2];
                switch (mode2)
                {
                case 'l':
                {
                    mode3 = read_dat_string[4];
                    switch (mode3)
                    {
                    case 'd':
                    {
                        leftround_diff = read_dat_float;
                    }
                    break;
                    case 'r':
                    {
                        leftround_ready = read_dat_float;
                    }
                    break;
                    case 'y':
                    {
                        leftround_yawtarget = read_dat_float;
                    }
                    break;
                    case 'm':
                    {
                        lmid_del = read_dat_float;
                    }
                    break;
                    case 'p':
                    {
                        pretoreadyL_round_distance = read_dat_float;
                    }           
                    break;   
                    }
                }
                break;
                case 'r':
                {
                    mode3 = read_dat_string[4];
                    switch (mode3)
                    {
                    case 'd':
                    {
                        rigtround_diff = read_dat_float;
                    }
                    break;
                    case 'r':
                    {
                        rigtround_ready = read_dat_float;
                    }
                    break;
                    case 'y':
                    {
                        rigtround_yawtarget = read_dat_float;
                    }
                    break;         
                    case 'm':
                    {
                        rmid_del = read_dat_float;
                    }   
                    break;       
                    case 'p':
                    {
                        pretoreadyR_round_distance = read_dat_float;
                    } 
                    break;
                    }
                }
                break;
                }
            }
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ַ����еĸ������ַ���ת������
//  @param      *str			������ַ���
//  @param
//  @return     result			ת���ɵĸ�����
//-------------------------------------------------------------------------------------------------------------------
double myatof(const char *str)
{
    double part_int = 0.0;
    double part_dec = 10.0;
    double result;
    int radix = 0;
    int flag = 0;
    // ��������ֺ͸�����ָ�����
    while (!(*str >= '0' && *str <= '9') && !(*str == '-'))
    {
        str++;
    }
    // ��¼��������
    if (*str == '-')
    {
        flag = 1;
        str++;
    }
    // �����������ֵ�ֵ
    while (*str >= '0' && *str <= '9' && *str != '.')
    {
        part_int = part_int * 10.0 + (*str - '0');
        str++;
    }
    // �ж�С����
    if (*str == '.')
    {
        str++;
    }
    // ����С�����ֵ�ֵ
    while (*str >= '0' && *str <= '9')
    {
        part_int = part_int + (*str - '0') / part_dec;
        part_dec *= 10.0;
        str++;
    }
    // ���ǿ�ѧ������
    if (*str == 'e' || *str == 'E')
    {
        str++;
        // e+xx��ʽ
        if (*str == '+')
        {
            str++;
            while (*str >= '0' && *str <= '9')
            {
                radix = radix * 10 + (*str - '0');
                str++;
            }
            while (radix > 0)
            {
                part_int *= 10;
                radix--;
            }
        }
        // e-xx��ʽ
        if (*str == '-')
        {
            str++;
            while (*str >= '0' && *str <= '9')
            {
                radix = radix * 10 + (*str - '0');
                str++;
            }
            while (radix > 0)
            {
                part_int /= 10;
                radix--;
            }
        }
    }
    result = part_int;
    return result * (flag ? -1.0 : 1.0);
}
