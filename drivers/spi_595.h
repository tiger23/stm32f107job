#ifndef SPI_595_H
#define SPI_595_H

#include <rtthread.h>
#include <drivers/spi.h>

struct spi_74hc595_74hc165
{
    struct rt_device                flash_device;
    struct rt_device_blk_geometry   geometry;
    struct rt_spi_device *          rt_spi_device;
};

extern rt_err_t m74hc595_init(const char * flash_device_name, const char * spi_device_name);


#endif // SPI_595_H
