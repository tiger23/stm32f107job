/*******************************************************************************
 * �ļ���  ��can_network.c
 * ����    ��
 *
 * ʵ��ƽ̨��STM32������
 * Ӳ�����ӣ�------------------------
 *          |       PA11-CAN-RX1       |
 *          |       PA12-CAN-TX1       |
 *          |       PB12-CAN-RX2       |
 *          |       PB13-CAN-TX2       |
 *           ------------------------
 * ��汾  ��
 * ����    ��astiger (astiger@foxmail.com)

**********************************************************************************/


#include "stm32f10x.h"
#include <stdio.h>
#include "bsp_button.h"
//#include "bsp_timer.h"
#include "bsp_led.h"
#include "can_network.h"

/* ����ȫ�ֱ��� */
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
 * ��������CAN_GPIO_Config
 * ����  ��CAN��GPIO ����,PB8�������룬PB9�������
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
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
 * ��������CAN_NVIC_Config
 * ����  ��CAN��NVIC ����,��1���ȼ��飬0��0���ȼ�
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /*�ж�����*/
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;     //CAN1 RX0�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;             //�����ȼ�Ϊ3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * ��������CAN_Mode_Config
 * ����  ��CAN��ģʽ ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Mode_Config(void)
{
    CAN_InitTypeDef        CAN_InitStructure;
    /************************CANͨ�Ų�������**********************************/
    /*CAN�Ĵ�����ʼ��*/
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
 * ��������CAN_Filter_Config
 * ����  ��CAN�Ĺ����� ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
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
    CAN_FilterInitStructure.CAN_FilterNumber = 15;//CAN2 �Ŀ�ʼ�˲������ n ��ͨ��д��CAN FMR �Ĵ����� CAN2SB[5:0]���õ�
    CAN_FilterInit(&CAN_FilterInitStructure);

}

/*
 * ��������CAN_Config
 * ����  ����������CAN�Ĺ���
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void CAN_Config(void)
{
    CAN_GPIO_Config();
    CAN_NVIC_Config();
    CAN_Mode_Config();
    CAN_Filter_Config();
    /*CANͨ���ж�ʹ��*/
    /* IT Configuration for CAN1 */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    /* IT Configuration for CAN2 */
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

}


/*
 * ��������CAN_SetMsg
 * ����  ��CANͨ�ű�����������
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void CAN_SetMsg(void)
{
    g_CanTxMessage.StdId=0x1314;                    //ʹ�õı�׼ID
    g_CanTxMessage.ExtId = 0x1314;                  //ʹ�õ���չID
    g_CanTxMessage.IDE = CAN_ID_STD;                //��׼ģʽ
    g_CanTxMessage.RTR = CAN_RTR_DATA;              //���͵�������
    g_CanTxMessage.DLC = 2;                         //���ݳ���Ϊ2�ֽ�
    g_CanTxMessage.Data[0] = 0xAB;
    g_CanTxMessage.Data[1] = 0xCD;
}

/******************************************************************************/
/*can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)
/*len:���ݳ���(���Ϊ8)
/*msg:����ָ��,���Ϊ8���ֽ�.
/*����ֵ:0,�ɹ�;
/*       ����,ʧ��;
/******************************************************************************/
u8 CAN_Send_Msg(u8 *msg, u8 len)
{
    u8 mbox;
    u16 i = 0;
    
    g_CanTxMessage.StdId = 0x1314;                  // ��׼��ʶ��Ϊ0x1314
    g_CanTxMessage.ExtId = 0x1314;              // ������չ��ʾ����29λ��
    g_CanTxMessage.IDE = CAN_ID_STD;        // ʹ�ñ�׼��ʶ��
    g_CanTxMessage.RTR = CAN_RTR_DATA;      // ��Ϣ����Ϊ����֡��һ֡8λ
    g_CanTxMessage.DLC = len;                           // ������֡��Ϣ
    for (i = 0; i < len; i++)
        g_CanTxMessage.Data[i] = msg[i];            // ��һ֡��Ϣ
    mbox = CAN_Transmit(CAN1, &g_CanTxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))i++; //�ȴ����ͽ���
    if (i >= 0XFFF)return 1;
    return 0;

}

/******************************************************************************/
/*can�ڽ������ݲ�ѯ
/*buf:���ݻ�����;
/*����ֵ:0,�����ݱ��յ�;
/*       ����,���յ����ݳ���;
/******************************************************************************/
u8 CAN_Receive_Msg(u8 *buf)
{
    u32 i;
    //CanRxMsg RxMessage;
    if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0)return 0;   //û�н��յ�����,ֱ���˳�
    CAN_Receive(CAN1, CAN_FIFO0, &g_CanRxMessage);//��ȡ����
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
