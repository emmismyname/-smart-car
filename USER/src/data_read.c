#include "data_read.h"

//----------------------data_readFrom_MotherBoard----------------------//
//----------------------data_readFrom_MotherBoard----------------------//
//----------------------data_readFrom_MOtherBoard----------------------//









//---------------------------wireless_read----------------------------//
//---------------------------wireless_read----------------------------//
//---------------------------wireless_read----------------------------//

uint8 send_flag = 0;

void wireless_read(uint8 flag)
{
//	uart_putstr(UART_3,"reading!");
//	printf("wireless reading");
    if (flag)
    {
        static uint8 read_buf[50];                  // 读取到的buffer
        static uint8 read_len = 0;                 	// 读取到的buffer长度
        static char read_dat_string[50];            // 读取到的字符串
        float read_dat_float;                // 读取到的字符串中的浮点数
//        char send_dat[20];                   // 发送回上位机检查数据
        char mode1 = '0', mode2 = '0', mode3 = '0'; // 接收模式
        
			
        read_len = wireless_uart_read_buff(read_buf, 50);	// 读取buffer并得到长度
        strcpy(read_dat_string, read_buf);								// buffer转换为字符串
        read_dat_float = myatof(read_dat_string);					// 提取浮点数

        if (read_len != 0)// 当收到数据时
        {
//					printf("1\n%s\n1\n",read_dat_string);
//					printf("2\nF:		%f\n2\n",read_dat_float);
            mode1 = read_dat_string[0];				// 模式位1
            switch (mode1)
            {
							case 'a': 											// 发送数据选择
									mode2 = read_dat_string[2]; // 模式位2
									send_flag = mode2 - 'a';
									break;
							case 'b': 											
									mode2 = read_dat_string[2]; // 模式位2
									switch (mode2)
									{
									case 'a': 									
											resetelec_flag = 1;	
											break;
									case 'b': 									

											break;
									case 'c': 			
										
											break;
									case 'd':	

											break;
									default:
											break;
									}
									break;
							case 'c': 											
									mode2 = read_dat_string[2]; // 模式位2
									switch (mode2)
									{
									case 'a': 		
											if(read_dat_float > 3.1415)	printf("\nReceive!\n");							
											break;
									case 'b': 									
											ADC_OUTOFRANGE_LIMIT = read_dat_float;
											break;
									
									
									case 'c': 		
											vector_ratio[0][0] = read_dat_float;						
											break;
									case 'd':	
											vector_ratio[0][1] = read_dat_float;
											break;
									case 'e':	
											vector_ratio[1][0] = read_dat_float;
											break;
									case 'f':	
											vector_ratio[1][1] = read_dat_float;
											break;
									
									
									case 'g':	
											curve_convert_ratio[0][0] = read_dat_float;
											break;
									case 'h':	
											curve_convert_ratio[0][1] = read_dat_float;
											break;
									case 'i':	
											curve_convert_ratio[0][2] = read_dat_float;
											break;
									case 'j':	
											curve_convert_ratio[0][3] = read_dat_float;
											break;
									
									
									case 'k':	
											straight_convert_ratio[0][0] = read_dat_float;
											break;
									case 'l':	
											straight_convert_ratio[0][1] = read_dat_float;
											break;
									case 'm':	
											straight_convert_ratio[0][2] = read_dat_float;
											break;									
									default:
											break;
									}
									break;
							case 'e': 											
									mode2 = read_dat_string[2]; // 模式位2
									switch (mode2)
									{
									case 'a': 									
											
											break;
									case 'b': 									

											break;
									case 'c': 			
										
											break;
									case 'd':	

											break;
									default:
											break;
									}
									break;
							case 'f': 											
									mode2 = read_dat_string[2]; // 模式位2
									switch (mode2)
									{
									case 'a': 									
											
											break;
									case 'b': 									

											break;
									case 'c': 			
										
											break;
									case 'd':	

											break;
									default:
											break;
									}
									break;
							default:
									break;
						}
						memset(read_buf,'\0',sizeof(read_buf));
						memset(read_dat_string,'\0',sizeof(read_dat_string));
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

















