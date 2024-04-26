#include "data.h"

char datetail[4] = {0x00, 0x00, 0x80, 0x7f}; // 定义帧尾

bit spi_busy = 0;

uint8 Receive_buff[RecBufSize] = {0};
uint8 Receive_ptr = 0;

const uint8 SPIFrameFloat[2] = {0x07,0x0a};
const uint8 SPIFrameUint[2]  = {0x08,0x0b};
const uint8 SPIFrameUchar[2] = {0x09,0x0c};

float SPI_float = 0.0;
uint16 SPI_uint = 0;
uint8 SPI_uchar = 0;
uint8 uart_mode=0;
float dat[64];                               // 定义数据体

//----------------------data_sendTo_SignalBoard----------------------//
//----------------------data_sendTo_SignalBoard----------------------//
//----------------------data_sendTo_SignalBoard----------------------//

//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi初始化
//  @param      无				
//  @param      
//  @return     无
//-------------------------------------------------------------------------------------------------------------------
void spi_master_init()
{
		PSPIH = 1;PSPI = 1;					//SPI中断设置为最高优先级
		spi_init(SPI_CH2, SPI_CH2_SCLK_P25 , SPI_CH2_MOSI_P23, SPI_CH2_MISO_P24, 0, MASTER, SPI_SYSclk_DIV_16);	
		SS_2=1;
		spi_busy=0;
		SPSTAT = 0xC0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi接收初始化
//  @param      无				
//  @param      
//  @return     无
//-------------------------------------------------------------------------------------------------------------------
void spi_slave_init()
{
		PSPIH = 1;PSPI = 1;					//SPI中断设置为最高优先级
		spi_init(SPI_CH2, SPI_CH2_SCLK_P25 , SPI_CH2_MOSI_P23, SPI_CH2_MISO_P24, 0, SLAVE, SPI_SYSclk_DIV_16);	
		SPSTAT = 0xC0;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      使能SPI中断
//  @param      无				
//  @param      
//  @return     无
//-------------------------------------------------------------------------------------------------------------------
void EnableSPIIRQ(void)
{	
	SS_2 = 0;
	ESPI = 1;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi发送字节
//  @param      dat
//  @return			void
//  Sample usage: 	spi_send_byte(0x40);
//-------------------------------------------------------------------------------------------------------------------
void spi_send_byte(uint8 dat)
{ 
    while(spi_busy);
    spi_busy=1;
		SS_2=0;//
    SPDAT = dat;					//DATA寄存器赋值
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi发送数组
//  @param      buff        需要发送的数据地址
//  @param      len         发送长度
//  @return     uint32      剩余未发送的字节数   
//  Sample usage:	    
//-------------------------------------------------------------------------------------------------------------------
void spi_send_buffer(unsigned char buff[], uint8 len)
{   
    int i=0;
    for(i=0;i<len;i++)
    {
      spi_send_byte(buff[i]);
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi发送浮点数
//  @param      num					待发浮点数
//  @return			void
//  Sample usage:		spi_send_float(123.987);
//-------------------------------------------------------------------------------------------------------------------
void spi_send_float(float num)
{
    // 浮点数转化为二进制数组
    uint8 bytedat[4] = {0};
    floatToByte(num, bytedat);
		// 发送帧头
    spi_send_byte(SPIFrameFloat[0]);
    // 发送数据体
    spi_send_buffer(bytedat, sizeof(float));
    // 发送帧尾
    spi_send_byte(SPIFrameFloat[1]);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi发送无符号整型
//  @param      num					待发无符号整型
//  @return			void
//  Sample usage:		spi_send_uint(1234);
//-------------------------------------------------------------------------------------------------------------------
void spi_send_uint(uint16 num)
{
    // 无符号整型转化为二进制数组
		uint8 bytedat[2] = {0};
		bytedat[0] = (unsigned char)(num);
		bytedat[1] = (unsigned char)(num >> 8);
		// 发送帧头
    spi_send_byte(SPIFrameUint[0]);
    // 发送数据体
    spi_send_buffer(bytedat, sizeof(int));
    // 发送帧尾
    spi_send_byte(SPIFrameUint[1]);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi发送无符号字符型
//  @param      num					待发无符号字符型
//  @return			void
//  Sample usage:		spi_send_uchar(234);
//-------------------------------------------------------------------------------------------------------------------
void spi_send_uchar(uint8 num)
{
		// 发送帧头
		spi_send_byte(SPIFrameUchar[0]);
    // 发送数据
    spi_send_byte(num);
    // 发送帧尾
    spi_send_byte(SPIFrameUchar[1]);
}





//---------------------------wireless_read----------------------------//
//---------------------------wireless_read----------------------------//
//---------------------------wireless_read----------------------------//

//-------------------------------------------------------------------------------------------------------------------
//  @brief      无线转串口读取函数
//  @param      flag			读取开启或关闭
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void wireless_read(uint8 flag)
{
//	printf("wireless reading");
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
										 jiangxing_No_1.start_flag = 1;
											car_start();
								 }
								 break;
								 case 's': // 停车标志
								 {
										 jiangxing_No_1.stop_flag = 1;
								 }
								 break;
								 case 'd': // 基础停车标志
								 {
                                    jiangxing_No_1.car_speed = read_dat_float;
                                    // jiangxing_No_1.left_target_speed= jiangxing_No_1.car_speed;
                                    // jiangxing_No_1.right_target_speed= jiangxing_No_1.car_speed;
								 }
								 break;
                                 case 'u': // 江心一号传输模式
								 {
                                    jiangxing_No_1.uart_mode = read_dat_float;  
									uart_mode	=	jiangxing_No_1.uart_mode;	 
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
									case 'f':
									{
											mode3 = read_dat_string[4];
										 switch (mode3)
										 {
										 case 'p':
										 {
												 Forward_PID.KP=read_dat_float;
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
										 case 't':
										 {
												 gyro_d = read_dat_float;
										 }
										 break;  
										 }
									}
                                    write_all_par();
									break;
								   }
							}
							break;
							case 'e': // 
							{
									mode2 = read_dat_string[2]; // 模式位2
									switch (mode2)
									{
									case 'l':
									{
											mode3 = read_dat_string[4];
										 switch (mode3)
										 {
										 case 'p':
										 {
												 Speed_PID_L.KP= read_dat_float;
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
										 case 't':
										 {
												 jiangxing_No_1.left_target_speed = read_dat_float;
										 }
										 break;  
										 }
                                         write_all_par();
									}
									break;
									case 'r':
									{
											mode3 = read_dat_string[4];
										 switch (mode3)
										 {
										 case 'p':
										 {
												 Speed_PID_R.KP= read_dat_float;
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
										 case 't':
										 {
												 jiangxing_No_1.right_target_speed = read_dat_float;
										 }
										 break;  
										 }
                                         write_all_par();
									}
									break;
							}
							break;
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
        // 定义要发送的数据存入数组速度环无线检测
  
				dat[0] = Speed_PID_L.KP;
				dat[1] = Speed_PID_L.KI;
				dat[2] = Speed_PID_L.KD;
				dat[3] = Speed_PID_R.KP;
				dat[4] = Speed_PID_R.KI;
				dat[5] = Speed_PID_R.KD;

				dat[6] = jiangxing_No_1.car_speed;
				dat[7] = jiangxing_No_1.left_target_speed;
				dat[8] = jiangxing_No_1.right_target_speed;
				dat[9] = duty_L;
				dat[10] = duty_R;
				dat[11] = Current_Speed_L;
				dat[12] = Current_Speed_R;
				dat[13] = Speed_PID_L.err;
				dat[14] = Speed_PID_R.err;
				dat[15] = Forward_PID.err;
				dat[16] = get_error1;
				dat[17] = roll;
				dat[18] = yaw;
                dat[19] = Forward_PID.KP;
                dat[20] = Forward_PID.KI;
                dat[21] = Forward_PID.KD;
				// dat[16] = i_o;                           
        // 发送数据
        wireless_uart_send(dat, 22);
        delay_ms(delayTime);
    }
    break;
    case 2:
    {
            dat[0] = jiangxing_No_1.car_speed;
            dat[1] = jiangxing_No_1.left_target_speed;
            dat[2] = jiangxing_No_1.right_target_speed;
            dat[3] = jiangxing_No_1.start_flag;
            dat[4] = jiangxing_No_1.start_mode;
            dat[5] = jiangxing_No_1.stop_flag;
            dat[6] = jiangxing_No_1.trace_flag;
            dat[7] = jiangxing_No_1.running_mode;
            dat[8] = Forward_PID.KP;
            dat[9] = Forward_PID.KI;
            dat[10] = Forward_PID.KD;
            dat[11] = get_error1;
            dat[12] = Forward_PID.err;
            dat[13] = Forward_PID.KP;
            dat[14] = Forward_PID.KI;
            dat[15] = Forward_PID.KD;
            dat[16] = Forward_PID.pid_out;
            dat[17] = z_gyro * gyro_d;
				// dat[16] = i_o;                           
        // 发送数据
        wireless_uart_send(dat, 18);
        delay_ms(delayTime);
    }
    break;
    case 3:
    {
        // 定义要发送的数据存入数组
//        dat[0] = adc[1].data_result;
//        dat[1] = adc[2].data_result;
//        dat[2] = adc[3].data_result;
//        dat[3] = adc[4].data_result;
//        dat[4] = adc[5].data_result;

//        dat[5] = adc[6].data_result;
//        dat[6] = adc[7].data_result;
//        dat[7] = adc[8].data_result;
//        dat[8] = adc[9].data_result;
//        dat[9] = adc[10].data_result;

//        dat[10] = err;
//        dat[11] = err_last;
//        dat[12] = Err_1;
//        dat[13] = Err_2;
//        dat[14] = 0;

//        // dat[15] = brusheless_flag;
//        // dat[16] = brusheless_duty;
//        // 发送数据
//        wireless_uart_send(dat, 15);
//        delay_ms(delayTime);
        // 发送数据
    }
    break;
    case 4:
    { // 电感
          // 定义要发送的数据存入数组
      
    }
    break;
    case 5:
    {
       
    }
    break;
    case 6:
    {
        
    }
    break;
    case 7:
    { // 定义要发送的数据存入数组
       
    }
    break;
    case 8:
    { // 定义要发送的数据存入数组
       
    }
    break;
    case 9:
    { // 定义要发送的数据存入数组
        
    }
    break;
    }
//	  printf("wireless sending");
}



