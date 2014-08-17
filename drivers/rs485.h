#ifndef __RS485_H__
#define __RS485_H__			 
#include <drivers\serial.h>	 								  
//////////////////////////////////////////////////////////////////////////////////	 
//����:2014-8-14
//����:ASTiger
//RS485���� ����	   
//////////////////////////////////////////////////////////////////////////////////
	  		  	
extern u8 RS485_RX_BUF[64]; 		//���ջ���,���64���ֽ�
extern u8 RS485_RX_CNT;   			//���յ������ݳ���

struct serial_rs485
{
    struct rt_device                rs485_device;
    struct rt_serial_device *       rt_serial_device;
};

#define DELAY_RS485_POLL    5
//ģʽ����
#define PA1_RS485_TX_EN		GPIO_SetBits(GPIOA,GPIO_Pin_1)	//485ģʽ����.0,����;1,����.
#define PA1_RS485_RX_EN		GPIO_ResetBits(GPIOA,GPIO_Pin_1)
#define PB8_RS485_TX_EN		GPIO_SetBits(GPIOB,GPIO_Pin_8)	//485ģʽ����.0,����;1,����.
#define PB8_RS485_RX_EN		GPIO_ResetBits(GPIOB,GPIO_Pin_8)
#define PC9_RS485_TX_EN		GPIO_SetBits(GPIOC,GPIO_Pin_9)	//485ģʽ����.0,����;1,����.
#define PC9_RS485_RX_EN	    GPIO_ResetBits(GPIOC,GPIO_Pin_9)
//����봮���жϽ��գ��벻Ҫע�����º궨��
#define EN_USART2_RX 	1			//0,������;1,����.

void RS485_GPIO_Config(void);
void RS485_Send_Data(const char *name, rt_size_t length);
void RS485_Receive_Data(const char *name,u8 *buf, rt_size_t length);
void rt_hw_RS485_init(void);
void RS485Poll(void);

#endif	   
















