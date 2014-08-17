/*
 * File      : rtdef.h
 * This file is part of RT-Thread RTOS
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-08-16     astiger      the first version
 */

#include <stdint.h>
#include "spi_595.h"

static struct spi_74hc595_74hc165  spi_m74hc595;

/* RT-Thread Device Driver Interface */
static rt_err_t M74HC595_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t M74HC595_open(rt_device_t dev, rt_uint16_t oflag)
{

    return RT_EOK;
}

static rt_err_t M74HC595_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t M74HC595_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    return RT_EOK;
}

static rt_size_t spi_read_165(rt_device_t dev,
                          rt_off_t    pos, void *buffer, rt_size_t size)
{
    uint32_t index, nr;
    uint8_t *read_buffer = buffer;

    nr = size;

    for (index = 0; index < nr; index++)
    {
        uint8_t send_buffer[8];
        uint32_t i;

        for (i = 0; i < sizeof(send_buffer); i++)
        {
            send_buffer[i] = 0;
        }

        rt_spi_send_then_recv(spi_m74hc595.rt_spi_device, send_buffer, 8, read_buffer, 256);
        //read_buffer += 256;
        
    }

    return size;
}

static rt_size_t spi_write_595( rt_device_t dev,
                          rt_off_t    pos,const void *buffer, rt_size_t size)
{
    rt_uint32_t index, nr;
    const uint8_t *write_buffer = buffer;

    nr = size;

    for (index = 0; index < nr; index++)
    {
        
        uint8_t send_buffer[4];

        send_buffer[0] = 1;
        send_buffer[1] = 2;
        send_buffer[2] = 3;
        send_buffer[3] = 0;

        rt_spi_send_then_send(spi_m74hc595.rt_spi_device, send_buffer, 4, write_buffer, 256);

        //write_buffer += 256;
        
        //wait_busy();
    }

    return size;
}

rt_err_t m74hc595_init(const char *m74hc595_device_name, const char *spi_device_name)
{
    struct rt_spi_device *rt_spi_device;

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if (rt_spi_device == RT_NULL)
    {
        rt_kprintf("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    spi_m74hc595.rt_spi_device = rt_spi_device;

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 and 3 */
        cfg.max_hz = 66000000; /* Atmel RapidS Serial Interface: 66MHz Maximum Clock Frequency */
        rt_spi_configure(spi_m74hc595.rt_spi_device, &cfg);
    }

    /* register device */
    spi_m74hc595.m74hc595_device.type    = RT_Device_Class_Block;
    spi_m74hc595.m74hc595_device.init    = M74HC595_init;
    spi_m74hc595.m74hc595_device.open    = M74HC595_open;
    spi_m74hc595.m74hc595_device.close   = M74HC595_close;
    spi_m74hc595.m74hc595_device.control = M74HC595_control;


    spi_m74hc595.m74hc595_device.read    = spi_read_165;
    spi_m74hc595.m74hc595_device.write   = spi_write_595;


    /* no private */
    spi_m74hc595.m74hc595_device.user_data = RT_NULL;

    rt_device_register(&spi_m74hc595.m74hc595_device, m74hc595_device_name,
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return RT_EOK;
}

void rt_hw_spi_m74hc595_init(void)
{
m74hc595_init("m74hc595", "SPI1");
}
void spi_m74hc595_Poll(void)
{
rt_size_t length=5;
    static char rt_log_buf[5];
rt_device_write(&spi_m74hc595.m74hc595_device, 0, rt_log_buf, length);
//spi_m74hc595.m74hc595_device.write();
}

