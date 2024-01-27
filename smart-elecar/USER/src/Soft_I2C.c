#include "Soft_I2C.h"

#define I2C_TIMEOUT_TIMES 100 // ��ʱ����

// ��ʱ ���ڵȴ�Ӧ��ʱ�ĳ�ʱ�ж� ��ֲʱ���޸�
void i2c_timeout_delay(void)
{
    unsigned char i, j;

    i = 2;
    j = 109;
    do
    {
        while (--j)
            ;
    } while (--i);
}

void i2c_delay() // ÿ���ļ�� ���ڵȴ���ƽ�ȶ��Ϳ���ͨѶ����
{
    unsigned char i;

    _nop_();
    i = 5;
    while (--i)
        ;
}

// SCL���� ��ֲʱ���޸�
void I2C_SCL_H(void)
{
    I2C_SCL = 1;
}

// SCL���� ��ֲʱ���޸�
void I2C_SCL_L(void)
{
    I2C_SCL = 0;
}

// SDA���� ��ֲʱ���޸�
void I2C_SDA_H(void)
{
    I2C_SDA = 1;
}

// SDA���� ��ֲʱ���޸�
void I2C_SDA_L(void)
{
    I2C_SDA = 0;
}

// ��ȡSDA ��ֲʱ���޸�
uint8_t I2C_SDA_Read(void)
{
    return I2C_SDA;
}

/*******************************************************************************
 * �� �� ��       : i2c_start
 * ��������		 : ����I2C��ʼ�ź�
 * ��    ��       : ��
 * ��    ��    	 : ��
 *******************************************************************************/
void i2c_start(void)
{
    I2C_SDA_H();
    I2C_SCL_H();
    i2c_delay();

    I2C_SDA_L(); // ��SCLΪ�ߵ�ƽʱ��SDA�ɸ߱�Ϊ��
    i2c_delay();
    I2C_SCL_L(); // ǯסI2C���ߣ�׼�����ͻ��������
}

/*******************************************************************************
 * �� �� ��         : i2c_stop
 * ��������		   : ����I2Cֹͣ�ź�
 * ��    ��         : ��
 * ��    ��         : ��
 *******************************************************************************/
void i2c_stop(void)
{
    I2C_SDA_L();
    I2C_SCL_H();
    i2c_delay();

    I2C_SDA_H(); // ��SCLΪ�ߵ�ƽʱ��SDA�ɵͱ�Ϊ��
    i2c_delay();
}

/*******************************************************************************
 * �� �� ��         : i2c_ack
 * ��������		   : ����ACKӦ��
 * ��    ��         : ��
 * ��    ��         : ��
 *******************************************************************************/
void i2c_ack(void)
{
    I2C_SCL_L();
    I2C_SDA_L(); // SDAΪ�͵�ƽ
    i2c_delay();

    I2C_SCL_H();
    i2c_delay();
    I2C_SCL_L();
    I2C_SDA_H();
}

/*******************************************************************************
 * �� �� ��         : i2c_nack
 * ��������		   : ����NACK��Ӧ��
 * ��    ��         : ��
 * ��    ��         : ��
 *******************************************************************************/
void i2c_nack(void)
{
    I2C_SCL_L();
    I2C_SDA_H(); // SDAΪ�ߵ�ƽ
    i2c_delay();

    I2C_SCL_H();
    i2c_delay();
    I2C_SCL_L();
}

/*******************************************************************************
* �� �� ��         : i2c_wait_ack
* ��������		   : �ȴ�Ӧ���źŵ���
* ��    ��         : ��
* ��    ��         : 1������Ӧ��ʧ��
                     0������Ӧ��ɹ�
*******************************************************************************/
uint8_t i2c_wait_ack(void)
{
    uint16_t time_temp = 0;

    I2C_SCL_H();
    i2c_delay();
    while (I2C_SDA_Read()) // �ȴ�SDAΪ�͵�ƽ
    {
        time_temp++;
        i2c_timeout_delay();
        if (time_temp > I2C_TIMEOUT_TIMES) // ��ʱ��ǿ�ƽ���I2Cͨ��
        {
            i2c_stop();
            return 1;
        }
    }
    I2C_SCL_L();
    return 0;
}

/*******************************************************************************
 * �� �� ��         : i2c_write_byte
 * ��������		   : I2C����һ���ֽ�
 * ��    ��         : dat������һ���ֽ�
 * ��    ��         : ��
 *******************************************************************************/
void i2c_write_byte(uint8_t dat)
{
    uint8_t i = 0;

    I2C_SCL_L();
    for (i = 0; i < 8; i++) // ѭ��8�ν�һ���ֽڴ������ȴ����ٴ���λ
    {
        if ((dat & 0x80) > 0)
            I2C_SDA_H();
        else
            I2C_SDA_L();
        dat <<= 1;
        i2c_delay();
        I2C_SCL_H();
        i2c_delay();
        I2C_SCL_L();
        i2c_delay();
    }
}

/*******************************************************************************
 * �� �� ��         : i2c_read_byte
 * ��������		   : I2C��һ���ֽ�
 * ��    ��         : ack = 1ʱ������ACK��ack = 0������nACK
 * ��    ��         : Ӧ����Ӧ��
 *******************************************************************************/
uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i = 0, receive = 0;

    for (i = 0; i < 8; i++) // ѭ��8�ν�һ���ֽڶ������ȶ����ٴ���λ
    {
        I2C_SCL_L();
        i2c_delay();
        I2C_SCL_H();
        receive <<= 1;
        if (I2C_SDA_Read())
            receive++;
        i2c_delay();
    }
    if (!ack)
        i2c_nack();
    else
        i2c_ack();

    return receive;
}

/*******************************************************************************
 * �� �� ��         : i2c_mem_write
 * ��������		   : I2C��ָ��������ָ���Ĵ�������д��
 * ��    ��         : ������ַ�������Ĵ�����ַ�������������׵�ַ�����������ݳ���
 * ��    ��         : 0: �ɹ� 1��ʧ��
 *******************************************************************************/
uint8_t i2c_mem_write(uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Len)
{
    i2c_start();
    i2c_write_byte(DevAddress << 1);
    if (i2c_wait_ack())
        return 1;
    i2c_write_byte(MemAddress);
    if (i2c_wait_ack())
        return 1;
    while (Len--)
    {
        i2c_write_byte(*pData++);
        if (i2c_wait_ack())
            return 1;
    }
    i2c_stop();
    return 0;
}

/*******************************************************************************
 * �� �� ��         : i2c_mem_read
 * ��������		   : I2C��ָ��������ָ���Ĵ���������ȡ
 * ��    ��         : ������ַ�������Ĵ�����ַ�����ݻ������׵�ַ�����ݳ���
 * ��    ��         : 0: �ɹ� 1��ʧ��
 *******************************************************************************/
uint8_t i2c_mem_read(uint8_t DevAddress, uint8_t MemAddress, uint8_t *pBuffer, uint16_t Len)
{
    i2c_start();
    i2c_write_byte(DevAddress << 1); // ����д����
    if (i2c_wait_ack())
        return 1;
    i2c_write_byte(MemAddress); // �����ֵ�ַ
    if (i2c_wait_ack())
        return 1;
    i2c_start();
    i2c_write_byte(DevAddress << 1 | 1); // �������ģʽ
    if (i2c_wait_ack())
        return 1;
    while (Len--)
    {
        *pBuffer++ = i2c_read_byte(Len != 0); // ��ȡ�ֽ�
    }
    i2c_stop(); // ����һ��ֹͣ����
    return 0;
}

/**д��8λ�Ĵ�����һ��λ��
 * @���� DevAddress	I2C��������ַ
 * @���� addr    	I2C�������ڲ���ַ
 * @���� bitNum  	д��ı���λ(0-7)
 * @���� data    	д������
 * @����ֵ ����״̬ (0=�ɹ�)
 */
uint8_t i2c_write_bit(uint8_t DevAddress, uint8_t addr, uint8_t bitNum, uint8_t Data)
{
    uint8_t b;
    if (!i2c_mem_read(DevAddress, addr, &b, 1))
    {
        b = (Data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
        return i2c_mem_write(DevAddress, addr, &b, 1); // д������
    }
    else
        return 1;
}

/**д��8λ�Ĵ����Ķ��λ��
 * @���� DevAddress	I2C��������ַ
 * @���� addr     I2C�������ڲ���ַ
 * @���� bitStart ��һλ��д��λ�ã�0-7��
 * @���� length   д�ı�����(������8)
 * @���� Data     д������
 * @����ֵ ����״̬ (0=�ɹ�)
 */
uint8_t i2c_write_bits(uint8_t DevAddress, uint8_t addr, uint8_t bitStart, uint8_t length, uint8_t Data)
{
    //      010 Ҫд���ֵ
    // 76543210 ����λ
    //    xxx   args: bitStart=4, length=3
    // 00011100 �����ֽ�
    // 10101111 ԭʼֵ��������
    // 10100011 ԭʼֵ & ~����
    // 10101011 ���� | ԭʼֵ
    uint8_t b, mask = 0;
    if (!i2c_mem_read(DevAddress, addr, &b, 1))
    {
        mask = (((1 << length) - 1) << (bitStart - length + 1)); // ����
        Data <<= (bitStart - length + 1);                        // ��д��������ƶ���λ
        Data &= mask;
        b &= ~(mask);
        b |= Data;

        return i2c_mem_write(DevAddress, addr, &b, 1); // д������
    }
    else
        return 1;
}
/**��ȡһ��λ��8λ�����ļĴ�����
 * @���� DevAddress	I2C��������ַ
 * @���� addr    I2C�������ڲ���ַ
 * @���� bitNum  λ��λ������ȡ��0-7��
 * @���� *data   ���ݴ洢��ַ
 * @����ֵ��0=�ɹ���
 */
uint8_t i2c_read_bit(uint8_t DevAddress, uint8_t addr, uint8_t bitNum, uint8_t *Data)
{
    uint8_t b;
    if (!i2c_mem_read(DevAddress, addr, &b, 1))
    {
        *Data = b & (1 << bitNum);
        return 0;
    }
    else
    {
        return 1;
    }
}
/**��ȡ8λ�Ĵ����Ķ��λ��
 * @���� DevAddress	I2C��������ַ
 * @���� addr    I2C�������ڲ���ַ
 * @���� bitStart��һλ��λ�ö�ȡ��0-7��
 * @���� length  λ��ȡ@������������������8��
 * @���� *data   ���ݴ洢��ַ����'101'�κ�bitStartλ�ö�ȡ������0X05��
 * @����ֵ��0=�ɹ���
 */
uint8_t i2c_read_bits(uint8_t DevAddress, uint8_t addr, uint8_t bitStart, uint8_t length, uint8_t *Data)
{
    // 01101001 ��ȡ�ֽ�
    // 76543210 ����λ
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t b, mask = 0;
    if (!i2c_mem_read(DevAddress, addr, &b, 1))
    {

        mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *Data = b;
        return 0;
    }
    else
        return 1;
}
