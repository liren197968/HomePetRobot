#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "common.h"

#define USART_REC_LEN           200                     //定义最大接收字节数200

void USART1_Init(void);
void Usart1Data_Send(uint8_t *FlowData, uint8_t DataLen);


extern uint8_t  USART_RX_BUF[USART_REC_LEN];            //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;                           //接收状态标记

#endif
