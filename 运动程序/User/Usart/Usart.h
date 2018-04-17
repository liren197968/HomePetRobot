#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "common.h"

#define USART_REC_LEN           200                     //�����������ֽ���200

void USART1_Init(void);
void Usart1Data_Send(uint8_t *FlowData, uint8_t DataLen);


extern uint8_t  USART_RX_BUF[USART_REC_LEN];            //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USART_RX_STA;                           //����״̬���

#endif
