/*******************************************************************************
 * 文件名  ：can_network.c
 * 描述    ：
 *
 * 实验平台：STM32开发板
 * 硬件连接：------------------------
 *          |       PB8-CAN-RX       |
 *          |       PB9-CAN-TX       |
 *           ------------------------
 * 库版本  ：
 * 作者    ：astiger (astiger@foxmail.com)

**********************************************************************************/


#include "stm32f10x.h"
#include <stdio.h>
#include "bsp_button.h"
#include "bsp_timer.h"
#include "bsp_led.h"
#include "can_network.h"

/* 定义全局变量 */
__IO uint32_t CANBus_flag;
CanRxMsg g_CanRxMessage;
CanTxMsg g_CanTxMessage;


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

    /*外设时钟设置*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /*IO设置*/
    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
    /* Configure CAN pin: RX */                                                // PB8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Configure CAN pin: TX */                                                // PB9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;              // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

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
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;     //CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;             //子优先级为3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
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
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    /*CAN单元初始化*/
    CAN_InitStructure.CAN_TTCM = DISABLE;          //MCR-TTCM  关闭时间触发通信模式使能
    CAN_InitStructure.CAN_ABOM = ENABLE;           //MCR-ABOM  自动离线管理
    CAN_InitStructure.CAN_AWUM = ENABLE;           //MCR-AWUM  使用自动唤醒模式
    CAN_InitStructure.CAN_NART = DISABLE;          //MCR-NART  禁止报文自动重传   DISABLE-自动重传
    CAN_InitStructure.CAN_RFLM = DISABLE;          //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文
    CAN_InitStructure.CAN_TXFP = DISABLE;          //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符
    //CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
    CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;       //BTR-SJW 重新同步跳跃宽度 2个时间单元
    CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;       //BTR-TS1 时间段1 占用了6个时间单元
    CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;       //BTR-TS1 时间段2 占用了3个时间单元
    CAN_InitStructure.CAN_Prescaler = 4;       ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+6+3)/4=0.8Mbps
    CAN_Init(CAN1, &CAN_InitStructure);
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

    /*CAN过滤器初始化*/
    CAN_FilterInitStructure.CAN_FilterNumber = 0;                   //过滤器组0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //工作在标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //过滤器位宽为单个32位。
    /* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

    CAN_FilterInitStructure.CAN_FilterIdHigh = (((u32)0x1314 << 3) & 0xFFFF0000) >> 16;         //要过滤的ID高位
    CAN_FilterInitStructure.CAN_FilterIdLow = (((u32)0x1314 << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF; //要过滤的ID低位
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;          //过滤器高16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;           //过滤器低16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0 ;           //过滤器被关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;          //使能过滤器
    CAN_FilterInit(&CAN_FilterInitStructure);
    /*CAN通信中断使能*/
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
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
    //g_CanTxMessage.StdId=0x00;
    g_CanTxMessage.ExtId = 0x1314;                  //使用的扩展ID
    g_CanTxMessage.IDE = CAN_ID_EXT;                //扩展模式
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
    //CanTxMsg g_CanTxMessage;
    g_CanTxMessage.StdId = 0x1314;                  // 标准标识符为0
    g_CanTxMessage.ExtId = 0x1314;              // 设置扩展标示符（29位）
    g_CanTxMessage.IDE = CAN_ID_EXT;        // 使用扩展标识符
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

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{

    /*从邮箱中读出报文*/
    CAN_Receive(CAN1, CAN_FIFO0, &g_CanRxMessage);

    /* 比较ID和数据是否为0x1314及DCBA */
    if ((g_CanRxMessage.ExtId == 0x1314) && (g_CanRxMessage.IDE == CAN_ID_EXT)
            && (g_CanRxMessage.DLC == 2) && ((g_CanRxMessage.Data[1] | g_CanRxMessage.Data[0] << 8) == 0xDCBA))
    {
        CANBus_flag = 0;                           //接收成功
        rt_kprintf("CAN bus running.");
    }
    else
    {
        CANBus_flag = 0xff;                        //接收失败
    }
}
/**************************END OF FILE************************************/
