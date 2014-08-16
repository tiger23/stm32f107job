/*
*********************************************************************************************************
*	                                  
*	模块名称 : CAN网络演示程序。
*	文件名称 : can_network.h
*	版    本 : V1.0
*	说    明 : 头文件
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2014-08-09 astiger rt-thread RTOS porting
*
*
*********************************************************************************************************
*/


#ifndef _CAN_NETWORK_H
#define _CAN_NETWORK_H

#define DELAY_CAN_POLL     4

/* 供外部调用的函数声明 */
void CAN_Config(void);
void CAN_SetMsg(void);
u8 CAN_Send_Msg(u8 *msg, u8 len);
u8 CAN_Receive_Msg(u8 *buf);
void rt_hw_CAN_init(void);
void CANPoll(void);

#endif


