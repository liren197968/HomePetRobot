#include "Usart.h"

uint16_t        USART_RX_STA = 0;                                                    //����״̬���
uint8_t         USART_RX_BUF[USART_REC_LEN];                                         //���ջ���,���USART_REC_LEN���ֽ�

#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE 
{ 
    int handle;
};

FILE __stdout;

//����_sys_exit()�Ա���ʹ�ð�����ģʽ
_sys_exit(int x) 
{ 
    x = x;
}

//�ض���fputc���� 
int fputc(int ch, FILE *f)
{
    while((USART1->SR & 0X40) == 0);                                                //ѭ������,ֱ���������
    USART1->DR = (uint8_t) ch;
    return ch;
}

void USART1_Init(void)
{
   //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                           //ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                          //ʹ��USART1ʱ��

    //����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);                       //GPIOA9����ΪUSART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);                      //GPIOA10����ΪUSART1

    //USART1�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;                         //GPIOA9��GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                    //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                               //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                  //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                    //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                          //��ʼ��PA9��PA10

    //USART1 ��ʼ������
    USART_InitStructure.USART_BaudRate = 115200;                                    //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure);                                       //��ʼ������1
    USART_Cmd(USART1, ENABLE);                                                      //ʹ�ܴ���1
    USART_ClearFlag(USART1, USART_FLAG_TC);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                                  //��������ж�

    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                               //����1�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                       //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;                              //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                 //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);                                                 //����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

void Usart1Data_Send(uint8_t *FlowData, uint8_t DataLen)
{
    uint8_t     i = 0;

    while(i < DataLen)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
        USART_SendData(USART1, FlowData[i]);

        i++;
    }
}

void USART1_IRQHandler(void)                                                        //����1�жϷ������
{
    uint8_t     Res = 0;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)                           //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
    {
        Res = USART_ReceiveData(USART1);                                            //��ȡ���յ�������

        if((USART_RX_STA & 0x8000) == 0)                                            //����δ���
        {
            if(USART_RX_STA & 0x4000)                                               //���յ���0x0d
            {
                if(Res != 0x0a)
                {
                    USART_RX_STA = 0;                                               //���մ���,���¿�ʼ
                }
                else
                {
                    USART_RX_STA |= 0x8000;                                         //���������
                }
            }
            else                                                                    //��û�յ�0x0d
            {
                if(Res == 0x0d)
                {
                    USART_RX_STA |= 0x4000;
                }
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;

                    USART_RX_STA++;

                    if(USART_RX_STA > (USART_REC_LEN - 1))
                    {
                        USART_RX_STA = 0;                                           //�������ݴ���,���¿�ʼ����
                    }
                }
            }
        }
    }
}

//static void UsartRevData_Print(void)
//{
//    uint8_t     i = 0;
//    uint16_t    DataLength = 0;

//    if(USART_RX_STA & 0x8000)
//    {
//        DataLength = USART_RX_STA & 0x3fff;                                         //�õ��˴ν��յ������ݳ���

//        printf("\r\n�����͵���ϢΪ:\r\n");

//        for(i = 0; i < DataLength; i++)
//        {
//            USART1->DR = USART_RX_BUF[i];
//            while((USART1->SR & 0X40) == 0);                                        //�ȴ����ͽ���
//        }

//        printf("\r\n\r\n");                                                         //���뻻��

//        USART_RX_STA = 0;
//    }
//}
