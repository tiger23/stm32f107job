/*******************************************************************************
 * 文件名  ：can_network.c
 * 描述    ：
 *
 * 实验平台：STM32开发板
 * 硬件连接：------------------------
 *          |       PA11-CAN-RX1       |
 *          |       PA12-CAN-TX1       |
 *          |       PB12-CAN-RX2       |
 *          |       PB13-CAN-TX2       |
 *           ------------------------
 * 库版本  ：
 * 作者    ：astiger (astiger@foxmail.com)

**********************************************************************************/


#include "stm32f10x.h"
#include <stdio.h>
#include "bsp_button.h"
//#include "bsp_timer.h"
#include "bsp_led.h"
#include "can_network.h"

/* 定义全局变量 */
__IO uint32_t CANBus_flag;
CanRxMsg g_CanRxMessage;
CanTxMsg g_CanTxMessage;

/* Private define ------------------------------------------------------------*/

#define CAN_BAUDRATE  1000      /* 1MBps   */
/* #define CAN_BAUDRATE  500*/  /* 500kBps */
/* #define CAN_BAUDRATE  250*/  /* 250kBps */
/* #define CAN_BAUDRATE  125*/  /* 125kBps */
/* #define CAN_BAUDRATE  100*/  /* 100kBps */
/* #define CAN_BAUDRATE  50*/   /* 50kBps  */
/* #define CAN_BAUDRATE  20*/   /* 20kBps  */
/* #define CAN_BAUDRATE  10*/   /* 10kBps  */
/* Private macro -------------------------------------------------------------*/

/*
 * 函数名：CAN_GPIO_Config
 * 描述  ：CAN的GPIO 配置,PB8上拉输入，PB9推挽输出
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure CAN1 and CAN2 IOs **********************************************/
    /* GPIOB, GPIOD and AFIO clocks enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* Configure CAN1 RX pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure CAN2 RX pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure CAN1 TX pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure CAN2 TX pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Remap CAN1 and CAN2 GPIOs */
    //GPIO_PinRemapConfig(GPIO_Remap2_CAN1 , ENABLE);
    //GPIO_PinRemapConfig(GPIO_Remap_CAN2, ENABLE);
    /* Configure CAN1 and CAN2 **************************************************/
    /* CAN1 and CAN2 Periph clocks enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
}

/*
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /*中断设置*/
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;     //CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;             //子优先级为3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Mode_Config(void)
{
    CAN_InitTypeDef        CAN_InitStructure;
    /************************CAN通信参数设置**********************************/
    /*CAN寄存器初始化*/
    /* CAN1 and CAN2 register init */
    CAN_DeInit(CAN1);
    CAN_DeInit(CAN2);

    /* Struct init*/
    CAN_StructInit(&CAN_InitStructure);

    /* CAN1 and CAN2  cell init */
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;

#if CAN_BAUDRATE == 1000 /* 1MBps */
    CAN_InitStructure.CAN_Prescaler = 6;
#elif CAN_BAUDRATE == 500 /* 500KBps */
    CAN_InitStructure.CAN_Prescaler = 12;
#elif CAN_BAUDRATE == 250 /* 250KBps */
    CAN_InitStructure.CAN_Prescaler = 24;
#elif CAN_BAUDRATE == 125 /* 125KBps */
    CAN_InitStructure.CAN_Prescaler = 48;
#elif  CAN_BAUDRATE == 100 /* 100KBps */
    CAN_InitStructure.CAN_Prescaler = 60;
#elif  CAN_BAUDRATE == 50 /* 50KBps */
    CAN_InitStructure.CAN_Prescaler = 120;
#elif  CAN_BAUDRATE == 20 /* 20KBps */
    CAN_InitStructure.CAN_Prescaler = 300;
#elif  CAN_BAUDRATE == 10 /* 10KBps */
    CAN_InitStructure.CAN_Prescaler = 600;
#else
#error "Please select first the CAN Baudrate in Private defines in main.c "
#endif  /* CAN_BAUDRATE == 1000 */

    /*Initializes the CAN1  and CAN2 */
    CAN_Init(CAN1, &CAN_InitStructure);
    CAN_Init(CAN2, &CAN_InitStructure);

}

/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Filter_Config(void)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    /* CAN1 filter init */
    CAN_FilterInitStructure.CAN_FilterNumber = 1;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x6420;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* CAN2 filter init */
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x2460;
    CAN_FilterInitStructure.CAN_FilterNumber = 15;//CAN2 的开始滤波器编号 n 是通过写入CAN FMR 寄存器的 CAN2SB[5:0]配置的
    CAN_FilterInit(&CAN_FilterInitStructure);

}

/*
 * 函数名：CAN_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN_Config(void)
{
    CAN_GPIO_Config();
    CAN_NVIC_Config();
    CAN_Mode_Config();
    CAN_Filter_Config();
    /*CAN通信中断使能*/
    /* IT Configuration for CAN1 */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    /* IT Configuration for CAN2 */
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

}


/*
 * 函数名：CAN_SetMsg
 * 描述  ：CAN通信报文内容设置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN_SetMsg(void)
{
    g_CanTxMessage.StdId=0x1314;                    //使用的标准ID
    g_CanTxMessage.ExtId = 0x1314;                  //使用的扩展ID
    g_CanTxMessage.IDE = CAN_ID_STD;                //标准模式
    g_CanTxMessage.RTR = CAN_RTR_DATA;              //发送的是数据
    g_CanTxMessage.DLC = 2;                         //数据长度为2字节
    g_CanTxMessage.Data[0] = 0xAB;
    g_CanTxMessage.Data[1] = 0xCD;
}

/******************************************************************************/
/*can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
/*len:数据长度(最大为8)
/*msg:数据指针,最大为8个字节.
/*返回值:0,成功;
/*       其他,失败;
/******************************************************************************/
u8 CAN_Send_Msg(u8 *msg, u8 len)
{
    u8 mbox;
    u16 i = 0;
    
    g_CanTxMessage.StdId = 0x1314;                  // 标准标识符为0x1314
    g_CanTxMessage.ExtId = 0x1314;              // 设置扩展标示符（29位）
    g_CanTxMessage.IDE = CAN_ID_STD;        // 使用标准标识符
    g_CanTxMessage.RTR = CAN_RTR_DATA;      // 消息类型为数据帧，一帧8位
    g_CanTxMessage.DLC = len;                           // 发送两帧信息
    for (i = 0; i < len; i++)
        g_CanTxMessage.Data[i] = msg[i];            // 第一帧信息
    mbox = CAN_Transmit(CAN1, &g_CanTxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))i++; //等待发送结束
    if (i >= 0XFFF)return 1;
    return 0;

}

/******************************************************************************/
/*can口接收数据查询
/*buf:数据缓存区;
/*返回值:0,无数据被收到;
/*       其他,接收的数据长度;
/******************************************************************************/
u8 CAN_Receive_Msg(u8 *buf)
{
    u32 i;
    //CanRxMsg RxMessage;
    if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0)return 0;   //没有接收到数据,直接退出
    CAN_Receive(CAN1, CAN_FIFO0, &g_CanRxMessage);//读取数据
    for (i = 0; i < 8; i++)
        buf[i] = g_CanRxMessage.Data[i];
    return g_CanRxMessage.DLC;
}

void rt_hw_CAN_init(void)
{
CAN_Config();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles CAN1 RX0 Handler.
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
  CAN_Receive(CAN1, CAN_FIFO0, &g_CanRxMessage);

  if ((g_CanRxMessage.StdId == 0x321)&&(g_CanRxMessage.IDE == CAN_ID_STD)&&(g_CanRxMessage.DLC == 1)&&(g_CanRxMessage.Data[0] == 0xAA))
  {
    /* Turn On LED3 */
    //LED_Display(0x03); /* OK */
  }
  else
  {
    /* Turn Off LED3 */
    //LED_Display(0x05); /* Error */
  }
}

/**
  * @brief  This function handles CAN2 RX0 Handler.
  * @param  None
  * @retval None
  */

void CAN2_RX0_IRQHandler(void)
{
  CAN_Receive(CAN2, CAN_FIFO0, &g_CanRxMessage);

  if ((g_CanRxMessage.StdId == 0x321)&&(g_CanRxMessage.IDE == CAN_ID_STD)&&(g_CanRxMessage.DLC == 1)&&(g_CanRxMessage.Data[0] == 0x55))
  {
    /* Turn On LED4 */
    //LED_Display(0x04); /* OK */
  }
  else
  {
    /* Turn Off LED4 */
    //LED_Display(0x06); /* Error */
  }
}

/**************************END OF FILE************************************/
