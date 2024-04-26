#include "data.h"

char datetail[4] = {0x00, 0x00, 0x80, 0x7f}; // ����֡β

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
float dat[64];                               // ����������

//----------------------data_sendTo_SignalBoard----------------------//
//----------------------data_sendTo_SignalBoard----------------------//
//----------------------data_sendTo_SignalBoard----------------------//

//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi��ʼ��
//  @param      ��				
//  @param      
//  @return     ��
//-------------------------------------------------------------------------------------------------------------------
void spi_master_init()
{
		PSPIH = 1;PSPI = 1;					//SPI�ж�����Ϊ������ȼ�
		spi_init(SPI_CH2, SPI_CH2_SCLK_P25 , SPI_CH2_MOSI_P23, SPI_CH2_MISO_P24, 0, MASTER, SPI_SYSclk_DIV_16);	
		SS_2=1;
		spi_busy=0;
		SPSTAT = 0xC0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi���ճ�ʼ��
//  @param      ��				
//  @param      
//  @return     ��
//-------------------------------------------------------------------------------------------------------------------
void spi_slave_init()
{
		PSPIH = 1;PSPI = 1;					//SPI�ж�����Ϊ������ȼ�
		spi_init(SPI_CH2, SPI_CH2_SCLK_P25 , SPI_CH2_MOSI_P23, SPI_CH2_MISO_P24, 0, SLAVE, SPI_SYSclk_DIV_16);	
		SPSTAT = 0xC0;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ʹ��SPI�ж�
//  @param      ��				
//  @param      
//  @return     ��
//-------------------------------------------------------------------------------------------------------------------
void EnableSPIIRQ(void)
{	
	SS_2 = 0;
	ESPI = 1;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi�����ֽ�
//  @param      dat
//  @return			void
//  Sample usage: 	spi_send_byte(0x40);
//-------------------------------------------------------------------------------------------------------------------
void spi_send_byte(uint8 dat)
{ 
    while(spi_busy);
    spi_busy=1;
		SS_2=0;//
    SPDAT = dat;					//DATA�Ĵ�����ֵ
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi��������
//  @param      buff        ��Ҫ���͵����ݵ�ַ
//  @param      len         ���ͳ���
//  @return     uint32      ʣ��δ���͵��ֽ���   
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
//  @brief      spi���͸�����
//  @param      num					����������
//  @return			void
//  Sample usage:		spi_send_float(123.987);
//-------------------------------------------------------------------------------------------------------------------
void spi_send_float(float num)
{
    // ������ת��Ϊ����������
    uint8 bytedat[4] = {0};
    floatToByte(num, bytedat);
		// ����֡ͷ
    spi_send_byte(SPIFrameFloat[0]);
    // ����������
    spi_send_buffer(bytedat, sizeof(float));
    // ����֡β
    spi_send_byte(SPIFrameFloat[1]);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi�����޷�������
//  @param      num					�����޷�������
//  @return			void
//  Sample usage:		spi_send_uint(1234);
//-------------------------------------------------------------------------------------------------------------------
void spi_send_uint(uint16 num)
{
    // �޷�������ת��Ϊ����������
		uint8 bytedat[2] = {0};
		bytedat[0] = (unsigned char)(num);
		bytedat[1] = (unsigned char)(num >> 8);
		// ����֡ͷ
    spi_send_byte(SPIFrameUint[0]);
    // ����������
    spi_send_buffer(bytedat, sizeof(int));
    // ����֡β
    spi_send_byte(SPIFrameUint[1]);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      spi�����޷����ַ���
//  @param      num					�����޷����ַ���
//  @return			void
//  Sample usage:		spi_send_uchar(234);
//-------------------------------------------------------------------------------------------------------------------
void spi_send_uchar(uint8 num)
{
		// ����֡ͷ
		spi_send_byte(SPIFrameUchar[0]);
    // ��������
    spi_send_byte(num);
    // ����֡β
    spi_send_byte(SPIFrameUchar[1]);
}





//---------------------------wireless_read----------------------------//
//---------------------------wireless_read----------------------------//
//---------------------------wireless_read----------------------------//

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ת���ڶ�ȡ����
//  @param      flag			��ȡ������ر�
//  @param
//  @return
//-------------------------------------------------------------------------------------------------------------------
void wireless_read(uint8 flag)
{
//	printf("wireless reading");
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
										 jiangxing_No_1.start_flag = 1;
											car_start();
								 }
								 break;
								 case 's': // ͣ����־
								 {
										 jiangxing_No_1.stop_flag = 1;
								 }
								 break;
								 case 'd': // ����ͣ����־
								 {
                                    jiangxing_No_1.car_speed = read_dat_float;
                                    // jiangxing_No_1.left_target_speed= jiangxing_No_1.car_speed;
                                    // jiangxing_No_1.right_target_speed= jiangxing_No_1.car_speed;
								 }
								 break;
                                 case 'u': // ����һ�Ŵ���ģʽ
								 {
                                    jiangxing_No_1.uart_mode = read_dat_float;  
									uart_mode	=	jiangxing_No_1.uart_mode;	 
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
									mode2 = read_dat_string[2]; // ģʽλ2
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






//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������ת��������
//  @param      fnum			����ĸ�����
//  @param      byte[]			ת���ɵĶ���������
//  @return
//-------------------------------------------------------------------------------------------------------------------
void floatToByte(float fnum, unsigned char byte[]) // ������ת��������
{
    FloatLongType fl;
    fl.fdata = fnum;
    byte[0] = (unsigned char)(fl.ldata);
    byte[1] = (unsigned char)(fl.ldata >> 8);
    byte[2] = (unsigned char)(fl.ldata >> 16);
    byte[3] = (unsigned char)(fl.ldata >> 24);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��������ת��Ϊ���������鲢����
//  @param      num[]			��������
//  @param      len             ���鳤��
//  @return
//-------------------------------------------------------------------------------------------------------------------
void wireless_uart_send(float num[], uint8 len) // ���ڷ��ͺ���
{
    int i, j;
    char hexdats[256];
    // ��������ת��Ϊ����������
    for (i = 0; i < len; i++)
    {
        uint8 bytedat[4] = {0};
        floatToByte(num[i], bytedat);
        for (j = i * 4; j <= i * 4 + 3; j++)
        hexdats[j] = bytedat[j % 4];
    }
    // ����������
    wireless_uart_send_buff(hexdats, sizeof(float) * len);
    // ����֡β
    wireless_uart_send_buff(datetail, 4);
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ߴ��ڷ��ͺ���
//  @param      mode			����ģʽ
//  @param      delayTime       ���η�����ʱʱ��(����)
//  @return
//-------------------------------------------------------------------------------------------------------------------
void wireless_send(int mode, int delayTime) // ���ߴ��ڷ��ͺ���
{
    switch (mode)
    {
    case 1:
    {
        // ����Ҫ���͵����ݴ��������ٶȻ����߼��
  
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
        // ��������
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
        // ��������
        wireless_uart_send(dat, 18);
        delay_ms(delayTime);
    }
    break;
    case 3:
    {
        // ����Ҫ���͵����ݴ�������
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
//        // ��������
//        wireless_uart_send(dat, 15);
//        delay_ms(delayTime);
        // ��������
    }
    break;
    case 4:
    { // ���
          // ����Ҫ���͵����ݴ�������
      
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
    { // ����Ҫ���͵����ݴ�������
       
    }
    break;
    case 8:
    { // ����Ҫ���͵����ݴ�������
       
    }
    break;
    case 9:
    { // ����Ҫ���͵����ݴ�������
        
    }
    break;
    }
//	  printf("wireless sending");
}



