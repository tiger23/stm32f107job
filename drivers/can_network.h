/*
*********************************************************************************************************
*	                                  
*	ģ������ : CAN������ʾ����
*	�ļ����� : can_network.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v1.0    2014-08-09 astiger rt-thread RTOS porting
*
*
*********************************************************************************************************
*/


#ifndef _CAN_NETWORK_H
#define _CAN_NETWORK_H

#define DELAY_CAN_POLL     4

/* ���ⲿ���õĺ������� */
void CAN_Config(void);
void CAN_SetMsg(void);
u8 CAN_Send_Msg(u8 *msg, u8 len);
u8 CAN_Receive_Msg(u8 *buf);
void rt_hw_CAN_init(void);
void CANPoll(void);

#endif


