#include "Speech.h"

uint8_t     SpeechReturnData = 0;

/*****************************���ڳ�ʼ������****************************************/
void Speech_Init(void)
{
   //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);                           //ʹ��GPIOCʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);                           //ʹ��UART4ʱ��

    //����4��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);                       //GPIOC10����ΪUSART4
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);                       //GPIOC11����ΪUSART4

    //UART4�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;                        //GPIOC10��GPIOC11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                    //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                               //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                  //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                    //����
    GPIO_Init(GPIOC, &GPIO_InitStructure);                                          //��ʼ��PC10��PC11

    //USART4 ��ʼ������
    USART_InitStructure.USART_BaudRate = 115200;                                    //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //�շ�ģʽ
    USART_Init(UART4, &USART_InitStructure);                                        //��ʼ������4
    USART_Cmd(UART4, ENABLE);                                                       //ʹ�ܴ���4
    USART_ClearFlag(UART4, USART_FLAG_TC);

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);                                   //��������ж�

    //USART4 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;                                //����4�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;                       //��ռ���ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;                              //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                 //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);                                                 //����ָ���Ĳ�����ʼ��VIC�Ĵ���
}
/**********************************************************************************/
/*****************************�������ݷ��ͺ���**************************************/
static void SpeechData_Send(char *StrData)
{
    uint8_t     i = 0;

    while(StrData[i] != '\0')
    {
        while(USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET);
        USART_SendData(UART4, StrData[i]);

        i++;
    }
}
/**********************************************************************************/
/*****************************����������ʾ��������**********************************/
void StandBySpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0001$");
}
/**********************************************************************************/
/*****************************����ǰ����ʾ��������**********************************/
void ForwardSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0002$");
}
/**********************************************************************************/
/*****************************����������ʾ��������**********************************/
void BackwrdSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0003$");
}
/**********************************************************************************/
/*****************************����˳ת��ʾ��������**********************************/
void ClckWisSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0004$");
}
/**********************************************************************************/
/*****************************������ת��ʾ��������**********************************/
void AtiClckSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0005$");
}
/**********************************************************************************/
/*****************************����������ʾ��������**********************************/
void TrasfrmSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0006$");
}
/**********************************************************************************/
/*****************************����ǰ����ʾ��������**********************************/
void FrdRollSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0007$");
}
/**********************************************************************************/
/*****************************���������ʾ��������**********************************/
void BakRollSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0008$");
}
/**********************************************************************************/
/*****************************����������ʾ��������**********************************/
void GestureSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0009$");
}
/**********************************************************************************/
/********************************ֹͣ�������ź���***********************************/
void Mp3SpeechStop_Play(void)
{
    SpeechData_Send("@StopPlaying#$");
}
/**********************************************************************************/
/*****************************����Ӧ�����ָ���**********************************/
void TxtUpdaSpeech_Play(void)
{
    SpeechData_Send("@WriteKeywords#���� 001|ǰ�� 002|���� 003|˳ת 004|��תȦ 005|���� 006|ǰ�� 007|��� 008|��������ʶ�� 009|�������� 010|�������� 011|$");
    delay_ms(5000);
    SpeechData_Send("@WriteFlashText#$");
    delay_ms(1000);
}
/**********************************************************************************/
/********************************�����жϺ���***************************************/
void UART4_IRQHandler(void)                                                         //����4�жϷ������
{
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)                            //�����ж�(���յ�������Ϊ�����ֽ�)
    {
        SpeechReturnData = USART_ReceiveData(UART4);                                //��ȡ���յ�������
    }
}
/**********************************************************************************/
