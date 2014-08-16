/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>

#include "spi_595.h"
#include "rs485.h"
#include "can_network.h"

#ifdef RT_USING_DFS
#include <dfs_fs.h>
#endif

#ifdef RT_USING_COMPONENTS_INIT
#include <components.h>
#endif /* RT_USING_COMPONENTS_INIT */

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led_stack[ 512 ];
static struct rt_thread led_thread;
static void led_thread_entry(void *parameter)
{
    rt_uint8_t led0 = 0;
    rt_uint8_t led1 = 1;

    rt_hw_led_init();

    while (1)
    {
        /* led1 on */
#ifndef RT_USING_FINSH
        rt_kprintf("led on, count : %d\r\n", count);
#endif
        //count++;
        rt_hw_led_on(led0);
        rt_thread_delay(RT_TICK_PER_SECOND / 2); /* sleep 0.5 second and switch to other thread */

        /* led1 off */
#ifndef RT_USING_FINSH
        rt_kprintf("led off\r\n");
#endif
        rt_hw_led_off(led0);
        rt_thread_delay(RT_TICK_PER_SECOND / 2);



    }
}

rt_uint8_t spi_m74hc595_WrBuf[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
rt_uint8_t spi_m74hc595_RdBuf[10];
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t spi_m74hc595_WrRd_stack[ 512 ];
static struct rt_thread spi_m74hc595_WrRd_thread;
static void spi_m74hc595_WrRd_thread_entry(void *parameter)
{
    rt_hw_spi_m74hc595_init();

    while (1)
    {

#ifndef RT_USING_FINSH
        rt_kprintf("SPI wrhite succeess.\r\n");
#endif
        rt_thread_delay(RT_TICK_PER_SECOND / 2); /* sleep 0.5 second and switch to other thread */

        spi_m74hc595_Poll();
    }
}

//====================操作系统各线程优先级==================================
#define thread_RS485Poll_Prio         10

ALIGN(RT_ALIGN_SIZE)
//====================操作系统各线程堆栈====================================
static rt_uint8_t thread_RS485Poll_stack[512];

struct rt_thread thread_RS485Poll;
//************************ RS485轮训线程***************************
//函数定义: void thread_entry_RS485Poll(void* parameter)
//入口参数：无
//出口参数：无
//备    注：Editor：Astiger   2014-08-15
//******************************************************************
void thread_entry_RS485Poll(void *parameter)
{
    rt_hw_RS485_init();

    while (1)
    {
        RS485Poll();
        rt_thread_delay(DELAY_RS485_POLL);
    }
}

//====================操作系统各线程优先级==================================
#define thread_CANPoll_Prio         11

ALIGN(RT_ALIGN_SIZE)
//====================操作系统各线程堆栈====================================
static rt_uint8_t thread_CANPoll_stack[512];

struct rt_thread thread_CANPoll;
//************************ CAN总线轮训线程***************************
//函数定义: void thread_entry_CANPoll(void* parameter)
//入口参数：无
//出口参数：无
//备    注：Editor：Astiger   2014-08-15
//******************************************************************
void thread_entry_CANPoll(void *parameter)
{
    rt_hw_CAN_init(void);
    //eMBEnable();
    while (1)
    {
        CANPoll();
        rt_thread_delay(DELAY_CAN_POLL);
    }
}

void rt_init_thread_entry(void *parameter)
{
    {
        extern void rt_platform_init(void);
        rt_platform_init();
    }

#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif

    /* Filesystem Initialization */
#if defined(RT_USING_DFS) && defined(RT_USING_DFS_ELMFAT)
    {
        /* mount sd card fat partition 1 as root directory */
        if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
        {
            rt_kprintf("File System initialized!\n");
        }
        else
        {
            rt_kprintf("File System initialzation failed!\n");
        }
    }
#endif /* RT_USING_DFS && RT_USING_DFS_ELMFAT */
}

int rt_application_init(void)
{
    rt_thread_t init_thread;

    rt_err_t result;

    /* init led thread */
    result = rt_thread_init(&led_thread,
                            "led",
                            led_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&led_stack[0],
                            sizeof(led_stack),
                            21,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led_thread);
    }


#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif

    if (init_thread != RT_NULL)
    {
        rt_thread_startup(init_thread);
    }

    return 0;
}

/*@}*/
