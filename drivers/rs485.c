#include <rtthread.h>
#include <stm32f10x.h>
#include <stm32f10x_usart.h>

#include "rs485.h"

static struct serial_rs485  rs485;

void RS485_Init(void)
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

/* RT-Thread Device Driver Interface */
static rt_err_t RS485_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t RS485_open(rt_device_t dev, rt_uint16_t oflag)
{

    return RT_EOK;
}

static rt_err_t RS485_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t RS485_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    return RT_EOK;
}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485_Send_Data(u8 *buf, u8 len)
{
    u8 t;
    PA1_RS485_TX_EN;        //设置为发送模式
    for (t = 0; t < len; t++) //循环发送数据
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        USART_SendData(USART2, buf[t]);
    }

    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    RS485_RX_CNT = 0;
    PA1_RS485_RX_EN;            //设置为接收模式
}
//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485_Receive_Data(u8 *buf, u8 *len)
{
    u8 rxlen = RS485_RX_CNT;
    u8 i = 0;
    *len = 0;           //默认为0
    delay_ms(10);       //等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
    if (rxlen == RS485_RX_CNT && rxlen) //接收到了数据,且接收完成了
    {
        for (i = 0; i < rxlen; i++)
        {
            buf[i] = RS485_RX_BUF[i];
        }
        *len = RS485_RX_CNT; //记录本次数据长度
        RS485_RX_CNT = 0;   //清零
    }
}

rt_err_t rs485_init(const char *rs485_device_name, const char *serial_device_name)
{
    struct rt_serial_device *rt_serial_device;

    rt_serial_device = (struct rt_serial_device *)rt_device_find(serial_device_name);
    if (rt_serial_device == RT_NULL)
    {
        rt_kprintf("serial_rs485 device %s not found!\r\n", serial_device_name);
        return -RT_ENOSYS;
    }
    rs485.rt_serial_device = rt_serial_device;

    /* config serial rs485 */
    {
        struct serial_configure cfg;
        cfg.baud_rate = BAUD_RATE_115200;
        cfg.data_bits = DATA_BITS_8; /*  */
        cfg.stop_bits = STOP_BITS_1; /*  */
        cfg.parity    = PARITY_NONE;
        //rt_spi_configure(rs485.rt_serial_device, &cfg);
    }

    /* register device */
    rs485.rs485_device.type    = RT_Device_Class_Char;
    rs485.rs485_device.init    = RS485_init;
    rs485.rs485_device.open    = RS485_open;
    rs485.rs485_device.close   = RS485_close;
    rs485.rs485_device.control = RS485_control;


    rs485.rs485_device.read    = RS485_Send_Data;
    rs485.rs485_device.write   = RS485_Receive_Data;


    /* no private */
    rs485.rs485_device.user_data = RT_NULL;

    rt_device_register(&rs485.rs485_device, rs485_device_name,
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return RT_EOK;
}

void rt_hw_RS485_init(void)
{

}
void RS485Poll(void)
{

}
void USART2_IRQHandler(void)
{

}
