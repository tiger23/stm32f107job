/*******************************************************************************
 * �ļ���  ��can_network.c
 * ����    ��
 *
 * ʵ��ƽ̨��STM32������
 * Ӳ�����ӣ�------------------------
 *          |       PB8-CAN-RX       |
 *          |       PB9-CAN-TX       |
 *           ------------------------
 * ��汾  ��
 * ����    ��astiger (astiger@foxmail.com)

**********************************************************************************/


#include "stm32f10x.h"
#include <stdio.h>
#include "bsp_button.h"
#include "bsp_timer.h"
#include "bsp_led.h"
#include "can_network.h"

/* ����ȫ�ֱ��� */
__IO uint32_t CANBus_flag;
CanRxMsg g_CanRxMessage;
CanTxMsg g_CanTxMessage;


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

    /*����ʱ������*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /*IO����*/
    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
    /* Configure CAN pin: RX */                                                // PB8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                // ��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Configure CAN pin: TX */                                                // PB9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;              // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

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
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;     //CAN1 RX0�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;             //�����ȼ�Ϊ3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
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
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    /*CAN��Ԫ��ʼ��*/
    CAN_InitStructure.CAN_TTCM = DISABLE;          //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
    CAN_InitStructure.CAN_ABOM = ENABLE;           //MCR-ABOM  �Զ����߹���
    CAN_InitStructure.CAN_AWUM = ENABLE;           //MCR-AWUM  ʹ���Զ�����ģʽ
    CAN_InitStructure.CAN_NART = DISABLE;          //MCR-NART  ��ֹ�����Զ��ش�   DISABLE-�Զ��ش�
    CAN_InitStructure.CAN_RFLM = DISABLE;          //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���
    CAN_InitStructure.CAN_TXFP = DISABLE;          //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ��
    //CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //��������ģʽ
    CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;       //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;       //BTR-TS1 ʱ���1 ռ����6��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;       //BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_Prescaler = 4;       ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36/(1+6+3)/4=0.8Mbps
    CAN_Init(CAN1, &CAN_InitStructure);
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

    /*CAN��������ʼ��*/
    CAN_FilterInitStructure.CAN_FilterNumber = 0;                   //��������0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //�����ڱ�ʶ������λģʽ
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //������λ��Ϊ����32λ��
    /* ʹ�ܱ��ı�ʾ�����������ձ�ʾ�������ݽ��бȶԹ��ˣ���չID�������µľ����������ǵĻ��������FIFO0�� */

    CAN_FilterInitStructure.CAN_FilterIdHigh = (((u32)0x1314 << 3) & 0xFFFF0000) >> 16;         //Ҫ���˵�ID��λ
    CAN_FilterInitStructure.CAN_FilterIdLow = (((u32)0x1314 << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF; //Ҫ���˵�ID��λ
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;          //��������16λÿλ����ƥ��
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;           //��������16λÿλ����ƥ��
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0 ;           //��������������FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;          //ʹ�ܹ�����
    CAN_FilterInit(&CAN_FilterInitStructure);
    /*CANͨ���ж�ʹ��*/
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
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
    //g_CanTxMessage.StdId=0x00;
    g_CanTxMessage.ExtId = 0x1314;                  //ʹ�õ���չID
    g_CanTxMessage.IDE = CAN_ID_EXT;                //��չģʽ
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
    //CanTxMsg g_CanTxMessage;
    g_CanTxMessage.StdId = 0x1314;                  // ��׼��ʶ��Ϊ0
    g_CanTxMessage.ExtId = 0x1314;              // ������չ��ʾ����29λ��
    g_CanTxMessage.IDE = CAN_ID_EXT;        // ʹ����չ��ʶ��
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

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{

    /*�������ж�������*/
    CAN_Receive(CAN1, CAN_FIFO0, &g_CanRxMessage);

    /* �Ƚ�ID�������Ƿ�Ϊ0x1314��DCBA */
    if ((g_CanRxMessage.ExtId == 0x1314) && (g_CanRxMessage.IDE == CAN_ID_EXT)
            && (g_CanRxMessage.DLC == 2) && ((g_CanRxMessage.Data[1] | g_CanRxMessage.Data[0] << 8) == 0xDCBA))
    {
        CANBus_flag = 0;                           //���ճɹ�
        rt_kprintf("CAN bus running.");
    }
    else
    {
        CANBus_flag = 0xff;                        //����ʧ��
    }
}
/**************************END OF FILE************************************/
