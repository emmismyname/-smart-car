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
        static uint8 read_buf[50];                  // ��ȡ����buffer
        static uint8 read_len = 0;                 	// ��ȡ����buffer����
        static char read_dat_string[50];            // ��ȡ�����ַ���
        float read_dat_float;                // ��ȡ�����ַ����еĸ�����
//        char send_dat[20];                   // ���ͻ���λ���������
        char mode1 = '0', mode2 = '0', mode3 = '0'; // ����ģʽ
        
			
        read_len = wireless_uart_read_buff(read_buf, 50);	// ��ȡbuffer���õ�����
        strcpy(read_dat_string, read_buf);								// bufferת��Ϊ�ַ���
        read_dat_float = myatof(read_dat_string);					// ��ȡ������

        if (read_len != 0)// ���յ�����ʱ
        {
//					printf("1\n%s\n1\n",read_dat_string);
//					printf("2\nF:		%f\n2\n",read_dat_float);
            mode1 = read_dat_string[0];				// ģʽλ1
            switch (mode1)
            {
							case 'a': 											// ��������ѡ��
									mode2 = read_dat_string[2]; // ģʽλ2
									send_flag = mode2 - 'a';
									break;
							case 'b': 											
									mode2 = read_dat_string[2]; // ģʽλ2
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
									mode2 = read_dat_string[2]; // ģʽλ2
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
									mode2 = read_dat_string[2]; // ģʽλ2
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
									mode2 = read_dat_string[2]; // ģʽλ2
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

















