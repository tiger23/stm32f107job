#include <rtthread.h>
#include <stm32f10x.h>
#include <stm32f10x_usart.h>

#include "rs485.h"

rt_device_t rs485_device;
static char rs485_device_TX_buf[RT_CONSOLEBUF_SIZE];
static char rs485_device_RX_buf[RT_CONSOLEBUF_SIZE];
rt_uint8_t rs485_device1_TX_flag;
rt_uint8_t rs485_device2_TX_flag;
rt_uint8_t rs485_device3_TX_flag;

void RS485_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE); //��λ����2
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE); //ֹͣ��λ

    PA1_RS485_RX_EN;        //Ĭ��Ϊ����ģʽ
    PB8_RS485_RX_EN;        //Ĭ��Ϊ����ģʽ
    PC9_RS485_RX_EN;        //Ĭ��Ϊ����ģʽ

}

//RS485����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ����64���ֽ�)
void RS485_Send_Data(u8 *buf, u8 len)
{
    rt_size_t length;
    rt_uint16_t old_flag;

    if (rs485_device1_TX_flag)
    {
        /*uart2*/
        rs485_device = rt_device_find("uart2");
        old_flag = rs485_device->flag;

        length = 1;

        rs485_device->flag |= RT_DEVICE_FLAG_STREAM;
        rt_device_write(rs485_device, 0, rs485_device_TX_buf, length);
        rs485_device->flag = old_flag;
    }
    if (rs485_device2_TX_flag)
    {
        /*uart3*/
        rs485_device = rt_device_find("uart3");
        old_flag = rs485_device->flag;

        length = 2;

        rs485_device->flag |= RT_DEVICE_FLAG_STREAM;
        rt_device_write(rs485_device, 0, rs485_device_TX_buf, length);
        rs485_device->flag = old_flag;
    }
    if (rs485_device3_TX_flag)
    {
        /*uart4*/
        rs485_device = rt_device_find("uart4");
        old_flag = rs485_device->flag;

        length = 3;

        rs485_device->flag |= RT_DEVICE_FLAG_STREAM;
        rt_device_write(rs485_device, 0, rs485_device_TX_buf, length);
        rs485_device->flag = old_flag;
    }
}
//RS485��ѯ���յ�������
//buf:���ջ����׵�ַ
//len:���������ݳ���
void RS485_Receive_Data(u8 *buf, u8 *len)
{
    rt_uint16_t old_flag;

    rs485_device = rt_device_find("uart4");
    old_flag = rs485_device->flag;

    rs485_device->flag |= RT_DEVICE_FLAG_STREAM;
    rt_device_read(rs485_device, 0, rs485_device_TX_buf, length);
    rs485_device->flag = old_flag;
}


void rt_hw_RS485_init(void)
{
    RS485_GPIO_Config();
}
void RS485Poll(void)
{
    RS485_Send_Data();
}
