#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"	 								  
//////////////////////////////////////////////////////////////////////////////////	 
//����:2014-8-14
//����:ASTiger
//RS485���� ����	   
//////////////////////////////////////////////////////////////////////////////////
	  		  	
extern u8 RS485_RX_BUF[64]; 		//���ջ���,���64���ֽ�
extern u8 RS485_RX_CNT;   			//���յ������ݳ���

//ģʽ����
#define PA1_RS485_RX_EN		PGout(9)	//485ģʽ����.0,����;1,����.
#define PB8_RS485_RX_EN		PGout(9)	//485ģʽ����.0,����;1,����.
#define PC9_RS485_RX_EN		PGout(9)	//485ģʽ����.0,����;1,����.
//����봮���жϽ��գ��벻Ҫע�����º궨��
#define EN_USART2_RX 	1			//0,������;1,����.

void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data(u8 *buf,u8 *len);


#endif	   
















