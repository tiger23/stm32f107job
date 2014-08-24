#ifndef SPI_595_H
#define SPI_595_H

#include <rtthread.h>
#include <rtdef.h>
#include <drivers/spi.h>

struct spi_74hc595_74hc165
{
    struct rt_device                m74hc595_device;
    struct rt_spi_device *          rt_spi_device;
};

rt_err_t m74hc595_init(const char *m74hc595_device_name, const char *spi_device_name);
void spi_m74hc595_Poll(void);
void rt_hw_spi_m74hc595_init(void);

#endif // SPI_595_H
