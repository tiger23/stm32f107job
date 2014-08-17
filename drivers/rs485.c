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

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE); //复位串口2
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE); //停止复位

    PA1_RS485_RX_EN;        //默认为接收模式
    PB8_RS485_RX_EN;        //默认为接收模式
    PC9_RS485_RX_EN;        //默认为接收模式

}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
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
//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
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
