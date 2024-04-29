/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		eeprom
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/
#include "zf_eeprom.h"
#include "board.h"
#include "intrins.h"
#include "zf_delay.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM触发操作，
//  @param      
//  @return     void
//  Sample usage:       		内部使用用户无需关心
//-------------------------------------------------------------------------------------------------------------------
void eeprom_trig(void)
{
    F0 = EA;    //保存全局中断
    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;                    //先送5AH，再送A5H到IAP触发寄存器，每次都需要如此
                                        //送完A5H后，IAP命令立即被触发启动
                                        //CPU等待IAP完成后，才会继续执行程序。
    _nop_();   //由于STC32G是多级流水线的指令系统，触发命令后建议加4个NOP，保证IAP_DATA的数据完成准备
    _nop_();
    _nop_();
    _nop_();
	
    EA = F0;    //恢复全局中断
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化EEPROM
//  @param      NULL
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void iap_init(void)
{
	IAP_CONTR = 0x80;	 	//使能EEPROM操作
	iap_set_tps();			//设置擦除等待时间

	
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      关闭EEPROM
//  @param      NULL
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void iap_idle(void)
{
	IAP_CONTR = 0;			//失能EEPROM操作
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取EEPROM操作失败状态位，需要软件清零
//  @param      NULL
//  @return     void
//  Sample usage:           
//								操作失败返回1;
//-------------------------------------------------------------------------------------------------------------------
uint8 iap_get_cmd_state(void)
{
	return ((IAP_CONTR&0x01) == 0x01);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置IAP等待时间
//  @param      NULL
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void iap_set_tps(void)
{
	uint8 write_time;
	write_time = (sys_clk / 1000000) ;
	IAP_TPS = write_time;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM读取多个字节
//  @param      addr			需要读取的eeprom地址
//  @param      *buf			需要读取的数据地址
//  @param      len				需要读取的数据长度
//  @return     void
//  Sample usage:               uint8 str[10];
//								iap_read_bytes(0x00,str,10);
//								将0x00-0x0A地址中的数据，读取到str中。
//-------------------------------------------------------------------------------------------------------------------
void iap_read_bytes(uint32 addr, uint8 *buf, uint16 len)
{

	
	IAP_CMD = 1; 				//设置 IAP 读命令	

	while(len--)
	{
		IAP_ADDRL = addr; 		//设置 IAP 低地址
		IAP_ADDRH = addr >> 8; 	//设置 IAP 高地址
		IAP_ADDRE = addr >> 16;	//设置 IAP 最高地址
        eeprom_trig();
		*buf++ = IAP_DATA; 		//读 IAP 数据
		addr++;
		
	}
	
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM写多个字节
//  @param      addr			需要写的eeprom地址
//  @param      *buf			需要写的数据地址
//  @param      len				需要写的数据长度
//  @return     void
//  Sample usage:       		iap_write_bytes(0x00,(uint8 *)"0123456789",10);
//								将"0123456789"写入0x00-0x0A地址中;
//-------------------------------------------------------------------------------------------------------------------
void iap_write_bytes(uint32 addr, uint8 *buf, uint16 len)
{

	IAP_CMD = 2; 				//设置 IAP 读命令	
	
	while(len--)
	{
		IAP_ADDRL = addr; 		//设置 IAP 低地址
		IAP_ADDRH = addr >> 8; 	//设置 IAP 高地址
		IAP_ADDRE = addr >> 16;	//设置 IAP 最高地址
		IAP_DATA = *buf++; 		//写 IAP 数据
		addr++;

		eeprom_trig();
	}
	
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM擦除目标地址所在的一页（1扇区/512字节）
//  @param      addr			需要写的eeprom地址
//  @return     void
//  Sample usage:       		iap_erase_page(0x20);
//								擦除0x00-0x200的数据
//-------------------------------------------------------------------------------------------------------------------
void iap_erase_page(uint32 addr) 
{ 

	IAP_CMD = 3; 				//设置 IAP 擦除命令
	IAP_ADDRL = addr; 			//设置 IAP 低地址
	IAP_ADDRH = addr >> 8;  	//设置 IAP 高地址
	IAP_ADDRE = addr >> 16;		//设置 IAP 最高地址
    eeprom_trig();	
	
	
	delay_ms(10);				//擦除1扇区(512字节)：约4-6ms
}



////-------------------------------------------------------------------------------------------------------------------
////  @brief      扩展EEPROM写多个字节(无需擦除)
////  @param      addr			需要写的eeprom地址
////  @param      *buf			需要写的数据地址
////  @param      len				需要写的数据长度
////  @return     void
////  Sample usage:       		extern_iap_write_bytes(0x0000,(uint8 *)"0123456789";,10);
////								将"0123456789"写入0x00-0x0A地址中;
////	@note：						不要跨扇区使用。
////								addr地址：0-511为一个扇区,512-1023为一个扇区，1024-1535为一个扇区，依次类推。
////-------------------------------------------------------------------------------------------------------------------
//void extern_iap_write_bytes(uint16 addr, uint8 *buf, uint16 len)
//{ 
//	uint8 temp[512];
//	uint16 i;
//	
//	for(i=0; i<512 ;i++)	temp[i] = 0;			//清0
//	iap_read_bytes(addr&0xFE00, temp, 512);			//读取
//	for(i=0; i<len; i++)	temp[(addr&0x1FF) + i] = buf[i];	//改
//	iap_erase_page(addr);							//擦除
//	iap_write_bytes(addr&0xFE00, temp, 512);		//写入
//}


