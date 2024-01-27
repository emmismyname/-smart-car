#include "uart_read.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      无线转串口读取函数
//  @param      flag			读取开启或关闭
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void wireless_read(uint8 flag)
{
    if (flag)
    {
        static uint8 read_buf[50];                  // 读取到的buffer
        static uint32 read_len = 0;                 // 读取到的buffer长度
        static char read_dat_string[50];            // 读取到的字符串
        static float read_dat_float;                // 读取到的字符串中的浮点数
        static char send_dat[20];                   // 发送回上位机检查数据
        char mode1 = '0', mode2 = '0', mode3 = '0'; // 接收模式
        // 读取buffer并得到长度
        read_len = wireless_uart_read_buff(read_buf, 50);
        // buffer转换为字符串
        strcpy(read_dat_string, read_buf);
        // 提取浮点数
        read_dat_float = myatof(read_dat_string);
        // 当收到数据时
        if (read_len != 0)
        {
            // 模式位1
            mode1 = read_dat_string[0];
            switch (mode1)
            {
            case 'o': // 主控
            {
                mode2 = read_dat_string[2]; // 模式位2
                switch (mode2)
                {
                case 'g': // 发车标志
                {
                    start_flag = 1;
                }
                break;
                case 's': // 停车标志
                {
                    stop_flag = 1;
                }
                break;
                case 'd': // 基础停车标志
                {
                    base_speed = read_dat_float;
                }
                break;
                }
                // 发送回上位机检验
                sprintf(send_dat, "%c %c\n", mode1, mode2);
                wireless_uart_send_buff(send_dat, strlen(send_dat));
                write_all_par();
            }
            break;
            case 'n': // 无元素参数
            {
                mode2 = read_dat_string[2]; // 模式位2
                switch (mode2)
                {
                case 's': // 速度环左轮
                {
                    mode3 = read_dat_string[4]; // 模式位3
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
                case 'z': // 速度环右轮
                {
                    mode3 = read_dat_string[4]; // 模式位3
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
                case 'f': // 直行环
                {
                    mode3 = read_dat_string[4]; // 模式位3
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
                case 'c': // 弯道环
                {
                    mode3 = read_dat_string[4]; // 模式位3
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
                case 'a': // 角度环
                {
                    mode3 = read_dat_string[4]; // 模式位3
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
                case 'o': // 出库
                {
                    mode3 = read_dat_string[4]; // 模式位3
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
                case 'i': // 入库
                {
                    mode3 = read_dat_string[4]; // 模式位3
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
                case 't': // 角度环
                {
                    mode3 = read_dat_string[4]; // 模式位3
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
                // 发送回上位机检验
                sprintf(send_dat, "%c_%c_%c=%f\n", mode1, mode2, mode3, read_dat_float);
                wireless_uart_send_buff(send_dat, strlen(send_dat));
                write_all_par();
            }
            break;
            case 'e': // 有元素参数
            {
                mode2 = read_dat_string[2]; // 模式位2
                switch (mode2)
                {
                case 'l': // 左小环岛环参
                {
                    mode3 = read_dat_string[4]; // 模式位3
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
                case 'r': // 右小环岛环参
                {
                    mode3 = read_dat_string[4]; // 模式位3
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
                    mode3 = read_dat_string[4]; // 模式位3
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
                // 发送回上位机检验
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
//  @brief      字符串中的浮点数字符串转浮点数
//  @param      *str			输入的字符串
//  @param
//  @return     result			转换成的浮点数
//-------------------------------------------------------------------------------------------------------------------
double myatof(const char *str)
{
    double part_int = 0.0;
    double part_dec = 10.0;
    double result;
    int radix = 0;
    int flag = 0;
    // 如果非数字和负号则指针后移
    while (!(*str >= '0' && *str <= '9') && !(*str == '-'))
    {
        str++;
    }
    // 记录数字正负
    if (*str == '-')
    {
        flag = 1;
        str++;
    }
    // 计算整数部分的值
    while (*str >= '0' && *str <= '9' && *str != '.')
    {
        part_int = part_int * 10.0 + (*str - '0');
        str++;
    }
    // 判断小数点
    if (*str == '.')
    {
        str++;
    }
    // 计算小数部分的值
    while (*str >= '0' && *str <= '9')
    {
        part_int = part_int + (*str - '0') / part_dec;
        part_dec *= 10.0;
        str++;
    }
    // 考虑科学计数法
    if (*str == 'e' || *str == 'E')
    {
        str++;
        // e+xx形式
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
        // e-xx形式
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
