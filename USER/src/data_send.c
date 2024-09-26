#include "data_send.h"

#define DATA_SIZE 32

bit spi_busy = 0;

uint8 Receive_buff[RecBufSize] = {0};
uint8 Receive_ptr = 0;

const uint8 SPIFrameFloat[2] = {0x07,0x0a};
const uint8 SPIFrameUint[2]  = {0x08,0x0b};
const uint8 SPIFrameUchar[2] = {0x09,0x0c};

float SPI_float = 0.0;
uint16 SPI_uint = 0;
uint8 SPI_uchar = 0;

float dat[DATA_SIZE] = {0.0};


//----------------------data_sendTo_MotherBoard----------------------//
//----------------------data_sendTo_MptherBoard----------------------//
//----------------------data_sendTo_MotherBoard----------------------//

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



//---------------------------wireless_send----------------------------//
//---------------------------wireless_send----------------------------//
//---------------------------wireless_send----------------------------//

void wireless_send_SBtoPC(void)
{
	
//	dat[0] = adc[1].data_max;
//	dat[1] = adc[1].data_min;
//	dat[2] = adc[2].data_max;
//	dat[3] = adc[2].data_min;
//	dat[4] = adc[3].data_max;
//	
//	dat[5] = adc[3].data_min;
//	dat[6] = adc[4].data_max;
//	dat[7] = adc[4].data_min;
//	dat[8] = adc[5].data_max;
//	dat[9] = adc[5].data_min;
	
//	dat[10] = adc[6].data_max;
//	dat[11] = adc[6].data_min;
//	dat[12] = adc[7].data_max;
//	dat[13] = adc[7].data_min;
//	dat[14] = adc[8].data_max;

//	dat[15] = adc[8].data_min;
//	dat[16] = adc[9].data_max;
//	dat[17] = adc[9].data_min;
//	dat[18] = adc[10].data_max;
//	dat[19] = adc[10].data_min;
	
	
//		dat[0] = adc_once(ADC_P00, ADC_12BIT);
//		dat[1] = adc_once(ADC_P01, ADC_12BIT);
//		dat[2] = adc_once(ADC_P03, ADC_12BIT);
//		dat[3] = adc_once(ADC_P02, ADC_12BIT);
//		dat[4] = adc_once(ADC_P05, ADC_12BIT);

//		dat[5] = adc_once(ADC_P16, ADC_12BIT);
//		dat[6] = adc_once(ADC_P17, ADC_12BIT);
//		dat[7] = adc_once(ADC_P14, ADC_12BIT);
//		dat[8] = adc_once(ADC_P13, ADC_12BIT);
//		dat[9] = adc_once(ADC_P15, ADC_12BIT);

//			dat[0] = adc[1].data_result;
//			dat[1] = adc[2].data_result;
//			dat[2] = adc[3].data_result;
//			dat[3] = adc[4].data_result;
//			dat[4] = adc[5].data_result;
//			
//			dat[5] = adc[6].data_result;
//			dat[6] = adc[7].data_result;
//			dat[7] = adc[8].data_result;
//			dat[8] = adc[9].data_result;
//			dat[9] = adc[10].data_result;
//		
//			dat[10] = err;
//			dat[11] = ADC_OUTOFRANGE_LIMIT;
//			
//			wireless_uart_send(dat,12);
	
		switch(send_flag)
		{
				case 0:
				{
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
						dat[11] = ADC_OUTOFRANGE_LIMIT;
						wireless_uart_send(dat,12);
				}
				break;
				case 2:
				{
						dat[0] = adc1_L2_H;
						dat[1] = adc1_L1_V;
						dat[2] = adc1_M_H;
						dat[3] = adc1_R1_V;
						dat[4] = adc1_R2_H;

						dat[5] = adc2_L2_H;
						dat[6] = adc2_L1_V;
						dat[7] = adc2_M_H;
						dat[8] = adc2_R1_V;
						dat[9] = adc2_R2_H;
					
						dat[10] = err;
						dat[11] = ADC_OUTOFRANGE_LIMIT;
						
						wireless_uart_send(dat,12);
				}	
				break;
				case 3:
				{
						dat[0] = adc1_L2H_withL1V;
						dat[1] = adc1_R2H_withR1V;
						dat[2] = adc1_MH_withL1V;
						dat[3] = adc1_MH_withR1V;
						dat[4] = adc1_L1_V;
						dat[5] = adc1_R1_V;

						dat[6] = adc2_L2H_withL1V;
						dat[7] = adc2_R2H_withR1V;
						dat[8] = adc2_MH_withL1V;
						dat[9] = adc2_MH_withR1V;
						dat[10] = adc2_L1_V;
						dat[11] = adc2_R1_V;
					
						dat[12] = err;
						dat[13] = ADC_OUTOFRANGE_LIMIT;
						
						wireless_uart_send(dat,14);
				}
				case 4:
				{
						dat[0] = vector_ratio[0][0];
						dat[1] = vector_ratio[0][1];
					
						dat[2] = vector_ratio[1][0];
						dat[3] = vector_ratio[1][1];
					
						dat[4] = Err_Hori;
						dat[5] = Err_2_Hori;
					
						dat[6] = Err_Vert;
						dat[7] = Err_2_Vert;
					
						dat[8] = Err_Inclined_left;
						dat[9] = Err_Inclined_right;
					
						dat[10] = curve_convert_ratio[0][0];
						dat[11] = curve_convert_ratio[0][1];
						dat[12] = curve_convert_ratio[0][2];
						dat[13] = curve_convert_ratio[0][3];
					
						dat[14] = straight_convert_ratio[0][0];
						dat[15] = straight_convert_ratio[0][1];
						dat[16] = straight_convert_ratio[0][2];
						
						dat[17] = err;
						
						wireless_uart_send(dat,18);
				}
				break;	
		}
}

























