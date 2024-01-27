/*********************************************************************************************************************
* MM32F527X-E9P Opensourec Library 即（MM32F527X-E9P 开源库）是一�?基于官方 SDK 接口的�??三方开源库
* Copyright (c) 2022 SEEKFREE 逐�?��?�技
* 
* �?文件�? MM32F527X-E9P 开源库的一部分
* 
* MM32F527X-E9P 开源库 �?免费�?�?
* 您可以根�?�?由软件基金会发布�? GPL（GNU General Public License，即 GNU通用�?共�?�可证）的条�?
* �? GPL 的�??3版（�? GPL3.0）或（您选择的）任何后来的版�?，重新发布和/或修改它
* 
* �?开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更�?�细节�?�参�? GPL
* 
* 您应该在收到�?开源库的同时收到一�? GPL 的副�?
* 如果没有，�?�参�?<https://www.gnu.org/licenses/>
* 
* 额�?�注明：
* �?开源库使用 GPL3.0 开源�?�可证协�? 以上许可申明为译文版�?
* 许可申明英文版在 libraries/doc 文件夹下�? GPL3_permission_statement.txt 文件�?
* 许可证副�?�? libraries 文件夹下 即�?�文件夹下的 LICENSE 文件
* 欢迎各位使用并传�?�?程序 但修改内容时必须保留逐�?��?�技的版权声明（即本声明�?
* 
* 文件名称          zf_device_dl1a
* �?司名�?          成都逐�?��?�技有限�?�?
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环�?          MDK 5.37
* 适用平台          MM32F527X_E9P
* 店铺链接          https://seekfree.taobao.com/
* 
* �?改�?�录
* 日期              作�?                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义�?
*                   ------------------------------------
*                   模块管脚            单片机�?�脚
*                   SCL                 查看 zf_device_dl1a.h �? DL1A_SCL_PIN  宏定�?
*                   SDA                 查看 zf_device_dl1a.h �? DL1A_SDA_PIN  宏定�?
*                   VCC                 5V 电源
*                   GND                 电源�?
*                   ------------------------------------
********************************************************************************************************************/


#include "zf_delay.h"
#include "SEEKFREE_DL1A.h"


#pragma warning disable = 183


uint8 dl1a_finsh_flag;
uint16 dl1a_distance_mm;



#define GET_DL1A_SDA   		 	DL1A_SDA_PIN
#define DL1A_SDA_LOW()         	DL1A_SDA_PIN = 0		//IO口输出低电平
#define DL1A_SDA_HIGH()        	DL1A_SDA_PIN = 1		//IO口输出高电平

#define DL1A_SCL_LOW()          	DL1A_SCL_PIN = 0		//IO口输出低电平
#define DL1A_SCL_HIGH()         	DL1A_SCL_PIN = 1		//IO口输出高电平

#define ack 1      //主应�?
#define no_ack 0   //从应�?	

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通�??失败�?以尝试�?�加j的�?
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_simiic_delay(void)
{
    uint16 j=DL1A_SOFT_IIC_DELAY;   
	while(j--);
}

//内部使用，用户无需调用
static void dl1a_simiic_start(void)
{
	DL1A_SDA_HIGH();
	DL1A_SCL_HIGH();
	dl1a_simiic_delay();
	DL1A_SDA_LOW();
	dl1a_simiic_delay();
	DL1A_SCL_LOW();
}

//内部使用，用户无需调用
static void dl1a_simiic_stop(void)
{
	DL1A_SDA_LOW();
	DL1A_SCL_LOW();
	dl1a_simiic_delay();
	DL1A_SCL_HIGH();
	dl1a_simiic_delay();
	DL1A_SDA_HIGH();
	dl1a_simiic_delay();
}

//主应�?(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
static void dl1a_simiic_sendack(unsigned char ack_dat)
{
    DL1A_SCL_LOW();
	dl1a_simiic_delay();
	if(ack_dat) DL1A_SDA_LOW();
    else    	DL1A_SDA_HIGH();

    DL1A_SCL_HIGH();
    dl1a_simiic_delay();
    DL1A_SCL_LOW();
    dl1a_simiic_delay();
}


static int dl1a_sccb_waitack(void)
{
    DL1A_SCL_LOW();

	dl1a_simiic_delay();
	
	DL1A_SCL_HIGH();
    dl1a_simiic_delay();
	
    if(GET_DL1A_SDA)           //应答为高电平，异常，通信失败
    {

        DL1A_SCL_LOW();
        return 0;
    }

    DL1A_SCL_LOW();
	dl1a_simiic_delay();
    return 1;
}

//字节发送程�?
//发送c(�?以是数据也可�?地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
static void dl1a_send_ch(uint8 c)
{
	uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)	DL1A_SDA_HIGH();//SDA 输出数据
        else			DL1A_SDA_LOW();
        c <<= 1;
        dl1a_simiic_delay();
        DL1A_SCL_HIGH();                //SCL 拉高，采集信�?
        dl1a_simiic_delay();
        DL1A_SCL_LOW();                //SCL 时钟线拉�?
    }
	dl1a_sccb_waitack();
}


//字节接收程序
//接收器件传来的数�?，�?�程序应配合|主应答函数|使用
//内部使用，用户无需调用
static uint8 dl1a_read_ch(uint8 ack_x)
{
    uint8 i;
    uint8 c;
    c=0;
    DL1A_SCL_LOW();
    dl1a_simiic_delay();
    DL1A_SDA_HIGH();             

    for(i=0;i<8;i++)
    {
        dl1a_simiic_delay();
        DL1A_SCL_LOW();         //�?时钟线为低，准�?�接收数�?�?
        dl1a_simiic_delay();
        DL1A_SCL_HIGH();         //�?时钟线为高，使数�?线上数据有效
        dl1a_simiic_delay();
        c<<=1;
        if(GET_DL1A_SDA) 
        {
            c+=1;   //读数�?位，将接收的数据存c
        }
    }

	DL1A_SCL_LOW();
	dl1a_simiic_delay();
	dl1a_simiic_sendack(ack_x);
	
    return c;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC写数�?到�?��?�寄存器函数
//  @param      dev_add			设�?�地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat				写入的数�?
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_simiic_write_dats(uint8 dev_add, uint8 *dat, uint32 len)
{
	dl1a_simiic_start();
    dl1a_send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写�?
	while(len--)
	{
		dl1a_send_ch( *dat++ );   				 //发送需要写入的数据
	}

	
	dl1a_simiic_stop();
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC写数�?到�?��?�寄存器函数
//  @param      dev_add			设�?�地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat				写入的数�?
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	dl1a_simiic_start();
    dl1a_send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写�?
	dl1a_send_ch( reg );   				 //发送从机寄存器地址
	dl1a_send_ch( dat );   				 //发送需要写入的数据
	dl1a_simiic_stop();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC从�?��?�寄存器读取数据
//  @param      dev_add			设�?�地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_simiic_read_reg(uint8 dev_add, uint8 reg)
{
	uint8 dat;
	dl1a_simiic_start();
    dl1a_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写�?
	dl1a_send_ch( reg );   				//发送从机寄存器地址

	
	dl1a_simiic_start();
	dl1a_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加�?�位
	dat = dl1a_read_ch(no_ack);   				//读取数据
	dl1a_simiic_stop();
	
	return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC读取多字节数�?
//  @param      dev_add			设�?�地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat_add			数据保存的地址指针
//  @param      num				读取字节数量
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint32 num)
{
	dl1a_simiic_start();
    dl1a_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写�?
	dl1a_send_ch( reg );   				//发送从机寄存器地址

	
	dl1a_simiic_start();
	dl1a_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加�?�位
    while(--num)
    {
        *dat_add = dl1a_read_ch(ack); //读取数据
        dat_add++;
    }
    *dat_add = dl1a_read_ch(no_ack); //读取数据
	dl1a_simiic_stop();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC读取多字节数�?
//  @param      dev_add			设�?�地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat_add			数据保存的地址指针
//  @param      num				读取字节数量
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_simiic_read_regs_1(uint8 dev_add, uint8 reg, uint8 *dat_add, uint32 num)
{
	dl1a_simiic_start();
    dl1a_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写�?
	dl1a_send_ch( reg );   				//发送从机寄存器地址

	
	dl1a_simiic_start();
	dl1a_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加�?�位
    while(--num)
    {
        *dat_add = dl1a_read_ch(ack); //读取数据
        dat_add++;
    }
    *dat_add = dl1a_read_ch(no_ack); //读取数据
	dl1a_simiic_stop();
}


#define dl1a_write_array(dat, len)          (dl1a_simiic_write_dats(DL1A_DEV_ADDR, (dat), (len)))
#define dl1a_write_register(reg, dat)       (dl1a_simiic_write_reg(DL1A_DEV_ADDR, (reg), (dat)))
#define dl1a_read_register(reg)             (dl1a_simiic_read_reg (DL1A_DEV_ADDR, (reg)))
#define dl1a_read_registers(reg, dat, len)  (dl1a_simiic_read_regs(DL1A_DEV_ADDR, (reg), (dat), (len)))
#define dl1a_read_registers_1(reg, dat, len)  (dl1a_simiic_read_regs_1(DL1A_DEV_ADDR, (reg), (dat), (len)))

// 这个速率表示从目标反射并�?设�?��?�测到的信号的�?�?
// 设置此限制可以确定传感器报告有效读数所需的最小测量�?
// 设置一�?较低的限制可以�?�加传感器的测量范围
// 但似乎也增加�? <由于来自�?标以外的物体的不需要的反射导致> 得到不准�?读数的可能�?
// 默�?�为 0.25 MCPS �?预�?�范围为 0 - 511.99
#define DL1A_DEFAULT_RATE_LIMIT  (0.25)

// 从寄存器数据解码 PCLKs �? VCSEL (vertical cavity surface emitting laser) 的脉宽周�?
#define decode_vcsel_period(reg_val)            (((reg_val) + 1) << 1)

// �? PCLK �?�? VCSEL 周期计算宏周�? (�? *纳�?�为单位)
// PLL_period_ps = 1655
// macro_period_vclks = 2304
#define calc_macro_period(vcsel_period_pclks)   ((((uint32)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     获取设�?? SPAD 信息
// 参数说明     index           索引
// 参数说明     type            类型�?
// 返回参数     uint8           �?否成�? 0-成功 1-失败
// 使用示例     dl1a_get_spad_info(index, type_is_aperture);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_get_spad_info (uint8 *index, uint8 *type_is_aperture)
{
    uint8 tmp = 0;
    uint8 return_state = 0;
    volatile uint16 loop_count = 0;

    do
    {
        dl1a_write_register(0x80, 0x01);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x00);

        dl1a_write_register(0xFF, 0x06);
        dl1a_read_registers(0x83, &tmp, 1);
        dl1a_write_register(0x83, tmp | 0x04);
        dl1a_write_register(0xFF, 0x07);
        dl1a_write_register(0x81, 0x01);

        dl1a_write_register(0x80, 0x01);

        dl1a_write_register(0x94, 0x6b);
        dl1a_write_register(0x83, 0x00);

        tmp = 0x00;
        while(tmp != 0x10)
        {
            dl1a_read_registers(0x83, &tmp, 1);
            loop_count++;
            if(loop_count == DL1A_TIMEOUT_COUNT)
            {
                return_state = 1;
                break;
            }
        }
		
        if(return_state)
        {
            break;
        }
        dl1a_write_register(0x83, 0x01);
        dl1a_read_registers(0x92, &tmp, 1);

        *index = tmp & 0x7f;
        *type_is_aperture = (tmp >> 7) & 0x01;

        dl1a_write_register(0x81, 0x00);
        dl1a_write_register(0xFF, 0x06);
        dl1a_read_registers(0x83, &tmp, 1);
        dl1a_write_register(0x83, tmp);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x01);

        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x00);
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     将超时数值从 MCLKs �?换到对应�? ms
// 参数说明     timeout_period_mclks    超时周期 MCLKs
// 参数说明     vcsel_period_pclks      PCLK �?
// 返回参数     uint32                  返回超时数�?
// 使用示例     dl1a_timeout_mclks_to_microseconds(timeout_period_mclks, vcsel_period_pclks);
// 备注信息     将序列�?��?�超时从具有给定 VCSEL 周期�? MCLK (�? PCLK 为单�?)�?�?为微�?
//-------------------------------------------------------------------------------------------------------------------
static uint32 dl1a_timeout_mclks_to_microseconds (uint16 timeout_period_mclks, uint8 vcsel_period_pclks)
{
    uint32 macro_period_ns = calc_macro_period(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     将超时数值从 ms �?换到对应�? MCLKs
// 参数说明     timeout_period_us   超时周期 �?秒单�?
// 参数说明     vcsel_period_pclks  PCLK �?
// 返回参数     uint32              返回超时数�?
// 使用示例     dl1a_timeout_microseconds_to_mclks(timeout_period_us, vcsel_period_pclks);
// 备注信息     将序列�?��?�超时从�?秒转�?为具有给�? VCSEL 周期�? MCLK (�? PCLK 为单�?)
//-------------------------------------------------------------------------------------------------------------------
static uint32 dl1a_timeout_microseconds_to_mclks (uint32 timeout_period_us, uint8 vcsel_period_pclks)
{
    uint32 macro_period_ns = calc_macro_period(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     对超时数值进行解�?
// 参数说明     reg_val         超时时长 寄存器�?
// 返回参数     uint16          返回超时数�?
// 使用示例     dl1a_decode_timeout(reg_val);
// 备注信息     从寄存器值解�? MCLK �?的序列�?��?�超�?   
//-------------------------------------------------------------------------------------------------------------------
static uint16 dl1a_decode_timeout (uint16 reg_val)
{
  // 格式: (LSByte * 2 ^ MSByte) + 1
    return  (uint16)((reg_val & 0x00FF) <<
            (uint16)((reg_val & 0xFF00) >> 8)) + 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     对超时数值进行编�?
// 参数说明     timeout_mclks   超时时长 -MCLKs �?
// 返回参数     uint16          返回编码�?
// 使用示例     dl1a_encode_timeout(timeout_mclks);
// 备注信息     �? MCLK �?对超时的序列步�?�超时寄存器值进行编�?
//-------------------------------------------------------------------------------------------------------------------
static uint16 dl1a_encode_timeout (uint16 timeout_mclks)
{
    uint32 ls_byte = 0;
    uint16 ms_byte = 0;
    uint16 return_data = 0;

    if (timeout_mclks > 0)
    {
        // 格式: (LSByte * 2 ^ MSByte) + 1
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }
        return_data = (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return return_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     获取序列步�?�使能�?�置
// 参数说明     enables         序列使能步�?�结构体
// 返回参数     void
// 使用示例     dl1a_get_sequence_step_enables(enables);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_get_sequence_step_enables(dl1a_sequence_enables_step_struct *enables)
{
    uint8 sequence_config = 0;
    dl1a_read_registers(DL1A_SYSTEM_SEQUENCE_CONFIG, &sequence_config, 1);

    enables->tcc          = (sequence_config >> 4) & 0x1;
    enables->dss          = (sequence_config >> 3) & 0x1;
    enables->msrc         = (sequence_config >> 2) & 0x1;
    enables->pre_range    = (sequence_config >> 6) & 0x1;
    enables->final_range  = (sequence_config >> 7) & 0x1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     获取脉冲周期
// 参数说明     type            预量程类�?
// 返回参数     uint8           返回的周期�?
// 使用示例     dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_PER_RANGE);
// 备注信息     �? PCLKs �?获取给定周期类型�? VCSEL 脉冲周期
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_get_vcsel_pulse_period (dl1a_vcsel_period_type_enum type)
{
    uint8 data_buffer = 0;
    if (type == DL1A_VCSEL_PERIOD_PER_RANGE)
    {
        dl1a_read_registers(DL1A_PRE_RANGE_CONFIG_VCSEL_PERIOD, &data_buffer, 1);
        data_buffer = decode_vcsel_period(data_buffer);
    }
    else if (type == DL1A_VCSEL_PERIOD_FINAL_RANGE)
    {
        dl1a_read_registers(DL1A_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &data_buffer, 1);
        data_buffer = decode_vcsel_period(data_buffer);
    }
    else
    {
        data_buffer = 255;
    }
    return data_buffer;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     获取序列步�?�超时�?�置
// 参数说明     enables         序列使能步�?�结构体
// 参数说明     timeouts        序列超时步�?�结构体
// 返回参数     void
// 使用示例     dl1a_get_sequence_step_timeouts(enables, timeouts);
// 备注信息     获取所有超时而不仅仅�?请求的超�? 并且还存储中间�?
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_get_sequence_step_timeouts (dl1a_sequence_enables_step_struct const *enables, dl1a_sequence_timeout_step_struct *timeouts)
{
    uint8 reg_buffer[2];
    uint16 reg16_buffer = 0;

    timeouts->pre_range_vcsel_period_pclks = dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_PER_RANGE);

    dl1a_read_registers(DL1A_MSRC_CONFIG_TIMEOUT_MACROP, reg_buffer, 1);
    timeouts->msrc_dss_tcc_mclks = reg_buffer[0] + 1;
    timeouts->msrc_dss_tcc_us = dl1a_timeout_mclks_to_microseconds(timeouts->msrc_dss_tcc_mclks, (uint8)timeouts->pre_range_vcsel_period_pclks);

    dl1a_read_registers(DL1A_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, reg_buffer, 2);
    reg16_buffer = ((uint16) reg_buffer[0] << 8) | reg_buffer[1];
    timeouts->pre_range_mclks = dl1a_decode_timeout(reg16_buffer);
    timeouts->pre_range_us = dl1a_timeout_mclks_to_microseconds(timeouts->pre_range_mclks, (uint8)timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_FINAL_RANGE);

    dl1a_read_registers(DL1A_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, reg_buffer, 2);
    reg16_buffer = ((uint16) reg_buffer[0] << 8) | reg_buffer[1];
    timeouts->final_range_mclks = dl1a_decode_timeout(reg16_buffer);

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = dl1a_timeout_mclks_to_microseconds(timeouts->final_range_mclks, (uint8)timeouts->final_range_vcsel_period_pclks);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     执�?�单次参考校�?
// 参数说明     vhv_init_byte   预�?�校准�?
// 返回参数     uint8           操作�?否成�? 0-成功 1-失败
// 使用示例     dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_PER_RANGE);
// 备注信息     �? PCLKs �?获取给定周期类型�? VCSEL 脉冲周期
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_perform_single_ref_calibration (uint8 vhv_init_byte)
{
    uint8 return_state = 0;
    uint8 data_buffer = 0;
    volatile uint16 loop_count = 0;
    do
    {
        dl1a_write_register(DL1A_SYSRANGE_START, 0x01 | vhv_init_byte);
        dl1a_read_registers(DL1A_MSRC_CONFIG_TIMEOUT_MACROP, &data_buffer, 1);
        while ((data_buffer & 0x07) == 0)
        {
            if (loop_count > 0x8fe0)
            {
                return_state = 1;
                break;
            }
            if (loop_count++ % 0x10 == 0)
            {
                dl1a_read_registers(DL1A_MSRC_CONFIG_TIMEOUT_MACROP, &data_buffer, 1);
            }
        }
        if(return_state)
        {
            break;
        }
        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01);
        dl1a_write_register(DL1A_SYSRANGE_START, 0x00);
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     设置测量定时预算 (以微秒为单位)
// 参数说明     budget_us       设定的测量允许的时间
// 返回参数     uint8           操作结果 0-成功 1-失败
// 使用示例     dl1a_set_measurement_timing_budget(measurement_timing_budget_us);
// 备注信息     这是一次测量允许的时间
//              即在测距序列的子步�?�之间分配时间�?�算
//              更长的时间�?�算允�?�更精确的测�?
//              增加一个N倍的预算�?以减少一个sqrt(N)倍的范围测量标准偏差
//              默�?�为33�?�? 最小值为20 ms
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_set_measurement_timing_budget (uint32 budget_us)
{
    uint8 return_state = 0;
    uint8 data_buffer[3];
    uint16 dat = 0;
	uint32 used_budget_us;
	uint32 final_range_timeout_us;
	uint16 final_range_timeout_mclks;
	
    dl1a_sequence_enables_step_struct enables;
    dl1a_sequence_timeout_step_struct timeouts;

    do
    {
        if (budget_us < DL1A_MIN_TIMING_BUDGET)
        {
            return_state = 1;
            break;
        }

        used_budget_us = DL1A_SET_START_OVERHEAD + DL1A_END_OVERHEAD;
        dl1a_get_sequence_step_enables(&enables);
        dl1a_get_sequence_step_timeouts(&enables, &timeouts);

        if (enables.tcc)
        {
            used_budget_us += (timeouts.msrc_dss_tcc_us + DL1A_TCC_OVERHEAD);
        }

        if (enables.dss)
        {
            used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DL1A_DSS_OVERHEAD);
        }
        else if (enables.msrc)
        {
            used_budget_us += (timeouts.msrc_dss_tcc_us + DL1A_MSRC_OVERHEAD);
        }

        if (enables.pre_range)
        {
            used_budget_us += (timeouts.pre_range_us + DL1A_PRERANGE_OVERHEAD);
        }

        if (enables.final_range)
        {
            // 请注�? 最终范围超时由计时预算和序列中所有其他超时的总和决定
            // 如果没有空间用于最终范围超�? 则将设置错�??
            // 否则 剩余时间将应用于最终范�?
            used_budget_us += DL1A_FINALlRANGE_OVERHEAD;
            if (used_budget_us > budget_us)
            {
                // 请求的超时太�?
                return_state = 1;
                break;
            }

            // 对于最终超时范�? 必须添加预量程范围超�?
            // 为�?? 最终超时和预量程超时必须以宏周�? MClks 表示
            // 因为它们具有不同�? VCSEL 周期
            final_range_timeout_us = budget_us - used_budget_us;
            final_range_timeout_mclks =
            dl1a_timeout_microseconds_to_mclks(final_range_timeout_us,
                     (uint8)timeouts.final_range_vcsel_period_pclks);

            if (enables.pre_range)
            {
                final_range_timeout_mclks += timeouts.pre_range_mclks;
            }

            dat = dl1a_encode_timeout(final_range_timeout_mclks);
            data_buffer[0] = DL1A_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI;
            data_buffer[1] = ((dat >> 8) & 0xFF);
            data_buffer[2] = (dat & 0xFF);
            dl1a_write_array(data_buffer, 3);
        }
    }while(0);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     获取测量定时预算 (以微秒为单位)
// 参数说明     void
// 返回参数     uint32          已�?�定的测量允许的时间
// 使用示例     dl1a_get_measurement_timing_budget();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint32 dl1a_get_measurement_timing_budget (void)
{
    dl1a_sequence_enables_step_struct enables;
    dl1a_sequence_timeout_step_struct timeouts;

    // 开始和结束开销时间始终存在
    uint32 budget_us = DL1A_GET_START_OVERHEAD + DL1A_END_OVERHEAD;

    dl1a_get_sequence_step_enables(&enables);
    dl1a_get_sequence_step_timeouts(&enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + DL1A_TCC_OVERHEAD);
    }

    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DL1A_DSS_OVERHEAD);
    }
    else if (enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + DL1A_MSRC_OVERHEAD);
    }

    if (enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + DL1A_PRERANGE_OVERHEAD);
    }

    if (enables.final_range)
    {
        budget_us += (timeouts.final_range_us + DL1A_FINALlRANGE_OVERHEAD);
    }

    return budget_us;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     设置返回信号速率限制 该值单位为 MCPS (百万次每�?)
// 参数说明     limit_mcps      设置的最小速率
// 返回参数     void
// 使用示例     dl1a_set_signal_rate_limit(0.25);
// 备注信息     这个速率表示从目标反射并�?设�?��?�测到的信号的�?�?
//              设置此限制可以确定传感器报告有效读数所需的最小测量�?
//              设置一�?较低的限制可以�?�加传感器的测量范围
//              但似乎也增加�? <由于来自�?标以外的物体的不需要的反射导致> 得到不准�?读数的可能�?
//              默�?�为 0.25 MCPS �?预�?�范围为 0 - 511.99
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_set_signal_rate_limit (float limit_mcps)
{
	uint8 data_buffer[3];
    uint16 limit_mcps_16bit = (limit_mcps * (1 << 7));
    //zf_assert(limit_mcps >= 0 || limit_mcps <= 511.99);


    data_buffer[0] = DL1A_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT;
    data_buffer[1] = ((limit_mcps_16bit >> 8) & 0xFF);
    data_buffer[2] = (limit_mcps_16bit & 0xFF);

    dl1a_write_array(data_buffer, 3);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     返回以�??米为单位的范围�?�数
// 参数说明     void
// 返回参数     uint8           0-数据无效 1-数据有效
// 使用示例     dl1a_get_distance();
// 备注信息     在开始单次射程测量后也调用�?�函�?
//-------------------------------------------------------------------------------------------------------------------
void dl1a_get_distance (void)
{
    uint8 reg_databuffer[3];

    dl1a_read_registers_1(DL1A_RESULT_INTERRUPT_STATUS, reg_databuffer, 1);
    if((reg_databuffer[0] & 0x07) == 0)
    {
        dl1a_finsh_flag = 0;
    }
    else
    {
        // 假�?�线性度校�?��?�益为默认�? 1000 且未�?用分数范�?
        dl1a_read_registers_1(DL1A_RESULT_RANGE_STATUS + 10, reg_databuffer, 2);
        dl1a_distance_mm = ((uint16)reg_databuffer[0] << 8);
        dl1a_distance_mm |= reg_databuffer[1];

        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01);
        dl1a_finsh_flag = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     初�?�化 DL1A
// 参数说明     void
// 返回参数     uint8           1-初�?�化失败 0-初�?�化成功
// 使用示例     dl1a_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 dl1a_init (void)
{
    uint32 measurement_timing_budget_us = 0;
    uint8 stop_variable = 0;
    uint8 return_state = 0;
    uint8 reg_data_buffer = 0;
    uint8 ref_spad_map[6];
    uint8 data_buffer[7];
	uint8 i = 0;
	
    memset(ref_spad_map, 0, 6);
    memset(data_buffer, 0, 7);



    do
    {
        delay_ms(100);
        DL1A_XSHUT_PIN = 0;
        delay_ms(50);
        DL1A_XSHUT_PIN = 1;
        delay_ms(100);

        // -------------------------------- DL1A �?动初始化 --------------------------------
        reg_data_buffer = dl1a_read_register(DL1A_IO_VOLTAGE_CONFIG);         // 传感器默�? IO �? 1.8V 模式
        dl1a_write_register(DL1A_IO_VOLTAGE_CONFIG, reg_data_buffer | 0x01);  // 配置 IO �? 2.8V 模式

        dl1a_write_register(0x88, 0x00);                                         // 设置为标�? IIC 模式

        dl1a_write_register(0x80, 0x01);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x00);

        dl1a_read_registers(0x91, &stop_variable , 1);

        dl1a_write_register(0x00, 0x01);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x00);

        // 禁用 SIGNAL_RATE_MSRC(bit1) �? SIGNAL_RATE_PRE_RANGE(bit4) 限制检�?
        reg_data_buffer = dl1a_read_register(DL1A_MSRC_CONFIG);
        dl1a_write_register(DL1A_MSRC_CONFIG, reg_data_buffer | 0x12);

        dl1a_set_signal_rate_limit(DL1A_DEFAULT_RATE_LIMIT);                  // 设置信号速率限制
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0xFF);
        // -------------------------------- DL1A �?动初始化 --------------------------------

        // -------------------------------- DL1A 配置初�?�化 --------------------------------
        if (dl1a_get_spad_info(&data_buffer[0], &data_buffer[1]))
        {
			return_state = 1;
            // 如果程序在输出了�?言信息 并且提示出错位置在这�?
            // 那么就是 dl1a �?检出错并超时退出了
            // 检查一下接线有没有�?�? 如果没问题可能就�?坏了

			printf("dl1a init error.\r\n");
			break;
        }

        // �? GLOBAL_CONFIG_SPAD_ENABLES_REF_[0-6] 获取 SPAD map (RefGoodSpadMap) 数据
        dl1a_read_registers(DL1A_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(DL1A_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
        dl1a_write_register(DL1A_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(DL1A_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

        data_buffer[2] = data_buffer[1] ? 12 : 0; // 12 is the first aperture spad
        for (i = 0; i < 48; i++)
        {
            if (i < data_buffer[2] || data_buffer[3] == data_buffer[0])
            {
                // 此位低于应启用的�?一�?�?
                // 或�? (eference_spad_count) 位已�?�?
                // 因�?��?�位为零
                ref_spad_map[i / 8] &= ~(1 << (i % 8));
            }
            else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
            {
                data_buffer[3]++;
            }
        }

        data_buffer[0] = DL1A_GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
        for(i = 1; i < 7; i++)
        {
            data_buffer[1] = ref_spad_map[i-1];
        }
        dl1a_write_array(data_buffer, 7);

        // 默�?�转换�?�置 version 02/11/2015_v36
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x00);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x09, 0x00);
        dl1a_write_register(0x10, 0x00);
        dl1a_write_register(0x11, 0x00);
        dl1a_write_register(0x24, 0x01);
        dl1a_write_register(0x25, 0xFF);
        dl1a_write_register(0x75, 0x00);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x4E, 0x2C);
        dl1a_write_register(0x48, 0x00);
        dl1a_write_register(0x30, 0x20);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x30, 0x09);
        dl1a_write_register(0x54, 0x00);
        dl1a_write_register(0x31, 0x04);
        dl1a_write_register(0x32, 0x03);
        dl1a_write_register(0x40, 0x83);
        dl1a_write_register(0x46, 0x25);
        dl1a_write_register(0x60, 0x00);
        dl1a_write_register(0x27, 0x00);
        dl1a_write_register(0x50, 0x06);
        dl1a_write_register(0x51, 0x00);
        dl1a_write_register(0x52, 0x96);
        dl1a_write_register(0x56, 0x08);
        dl1a_write_register(0x57, 0x30);
        dl1a_write_register(0x61, 0x00);
        dl1a_write_register(0x62, 0x00);
        dl1a_write_register(0x64, 0x00);
        dl1a_write_register(0x65, 0x00);
        dl1a_write_register(0x66, 0xA0);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x22, 0x32);
        dl1a_write_register(0x47, 0x14);
        dl1a_write_register(0x49, 0xFF);
        dl1a_write_register(0x4A, 0x00);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x7A, 0x0A);
        dl1a_write_register(0x7B, 0x00);
        dl1a_write_register(0x78, 0x21);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x23, 0x34);
        dl1a_write_register(0x42, 0x00);
        dl1a_write_register(0x44, 0xFF);
        dl1a_write_register(0x45, 0x26);
        dl1a_write_register(0x46, 0x05);
        dl1a_write_register(0x40, 0x40);
        dl1a_write_register(0x0E, 0x06);
        dl1a_write_register(0x20, 0x1A);
        dl1a_write_register(0x43, 0x40);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x34, 0x03);
        dl1a_write_register(0x35, 0x44);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x31, 0x04);
        dl1a_write_register(0x4B, 0x09);
        dl1a_write_register(0x4C, 0x05);
        dl1a_write_register(0x4D, 0x04);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x44, 0x00);
        dl1a_write_register(0x45, 0x20);
        dl1a_write_register(0x47, 0x08);
        dl1a_write_register(0x48, 0x28);
        dl1a_write_register(0x67, 0x00);
        dl1a_write_register(0x70, 0x04);
        dl1a_write_register(0x71, 0x01);
        dl1a_write_register(0x72, 0xFE);
        dl1a_write_register(0x76, 0x00);
        dl1a_write_register(0x77, 0x00);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x0D, 0x01);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x01);
        dl1a_write_register(0x01, 0xF8);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x8E, 0x01);
        dl1a_write_register(0x00, 0x01);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x00);

        // 将中�?配置设置为新样品就绪
        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_GPIO_CONFIG, 0x04);
        reg_data_buffer = dl1a_read_register(DL1A_GPIO_HV_MUX_ACTIVE_HIGH);
        dl1a_write_register(DL1A_GPIO_HV_MUX_ACTIVE_HIGH, reg_data_buffer & ~0x10);
        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01);

        measurement_timing_budget_us  = dl1a_get_measurement_timing_budget();

        // 默�?�情况下禁用 MSRC �? TCC
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0xE8);
        dl1a_set_measurement_timing_budget(measurement_timing_budget_us);    // 重新计算时序预算
        // -------------------------------- DL1A 配置初�?�化 --------------------------------

        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0x01);
        if (dl1a_perform_single_ref_calibration(0x40))
        {
            return_state = 1;
            break;
        }
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0x02);
        if (dl1a_perform_single_ref_calibration(0x00))
        {
            return_state = 1;
            break;
        }
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0xE8);           // 恢�?�以前的序列配置

        delay_ms(100);

        dl1a_write_register(0x80, 0x01);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x00);
        dl1a_write_register(0x91, stop_variable);
        dl1a_write_register(0x00, 0x01);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x00);

        dl1a_write_register(DL1A_SYSRANGE_START, 0x02);
    }while(0);

    return return_state;
}

