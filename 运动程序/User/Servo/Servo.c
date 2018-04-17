#include "string.h"
#include "Oled.h"
#include "Error.h"
#include "Servo.h"
#include "Mpu6050.h"
#include "Bluetooth.h"
#include "RobotControl.h"

static uint8_t      TorsionDisableFlag = 0;
static uint8_t      AngleReceiveFinshedFlag = 0;
static uint8_t      ServoReceiveBuffer[RECEIVE_ANGLE_STR_MAX_SIZE] = {0};
uint8_t             ServoAngleReadData[MAX_SERVO_NUM][RECEIVE_ANGLE_STR_MAX_SIZE] = {0};
/*******************************LED��ʼ������***************************************/
static void LED_Init(void)
{
    //GPIO�˿�����
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                                   //ʹ��GPIOAʱ��

    //LED�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;                                               //GPIOA5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                                           //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                       //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                          //�������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                            //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                                  //��ʼ��PA5

    GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}
/**********************************************************************************/
/*******************************���ڳ�ʼ������**************************************/
static void USART3_Init(void)
{
    //GPIO�˿�����
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);                                   //ʹ��GPIODʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);                                  //ʹ��USART3ʱ��

    //����3��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);                               //GPIOD8����ΪUSART3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);                               //GPIOD9����ΪUSART3

    //USART3�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;                                  //GPIOD8��GPIOD9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                            //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                       //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                          //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                            //����
    GPIO_Init(GPIOD, &GPIO_InitStructure);                                                  //��ʼ��PD8��P9

    //USART3 ��ʼ������
    USART_InitStructure.USART_BaudRate = 115200;                                            //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                             //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                                  //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                                     //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;         //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                         //�շ�ģʽ
    USART_Init(USART3, &USART_InitStructure);                                               //��ʼ������3

    USART_Cmd(USART3, ENABLE);                                                              //ʹ�ܴ���3
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                          //�����ж�
    USART_ClearFlag(USART3, USART_IT_RXNE);                                                 //����жϱ�־λ�������ʼ������Ƚ����ж�

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/**********************************************************************************/
/********************************��ʱ��2��ʼ������*********************************/
static void Timer2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);                                    //ʹ��TIM2ʱ��

    TIM_TimeBaseInitStructure.TIM_Period = SERVO_RUN_TIME_DEFAULT * 10;                     //ÿSERVO_RUN_TIME_DEFAULT ms���һ��
    TIM_TimeBaseInitStructure.TIM_Prescaler = 8399;                                         //��ʱ����Ƶ
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;                         //���ϼ���ģʽ
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);                                     //��ʼ��TIM2
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                                              //����ʱ��2�����ж�
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);                                             //����жϱ�־λ�������ʼ������Ƚ����ж�
    TIM_Cmd(TIM2, DISABLE);                                                                 //ʹ�ܶ�ʱ��2

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                                         //��ʱ��2�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;                            //��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;                                   //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/**********************************************************************************/
/*****************************�������ݷ��ͺ���**************************************/
static void Usart3Data_Send(char *StrData)
{
    uint8_t     i = 0;

    while(StrData[i] != '\0')
    {
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET);
        USART_SendData(USART3, StrData[i]);

        i++;
    }
}
/**********************************************************************************/
/*****************************�����жϴ�����**************************************/
void USART3_IRQHandler(void)                                                                //����3�жϷ������
{
    static uint8_t  ReceiveCounter = 0;
    static uint8_t  TempBuffer[RECEIVE_ANGLE_STR_MAX_SIZE];

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)                                   //�����ж���Ч,���������ݼĴ�����
    {
        TempBuffer[ReceiveCounter] = USART_ReceiveData(USART3);                             //��������

        if(ReceiveCounter == 0 && TempBuffer[0] != '#')
        {
            ReceiveCounter = 0;

            return;                                                                         //��0�����ݲ���֡ͷ������
        }

        if(TempBuffer[ReceiveCounter] == '\n')                                              //���յ����һ������
        {
            memcpy(ServoReceiveBuffer, TempBuffer, RECEIVE_ANGLE_STR_MAX_SIZE);
            memset(TempBuffer, 0, RECEIVE_ANGLE_STR_MAX_SIZE);

            ReceiveCounter = 0;                                                             //���¸�ֵ��׼����һ֡���ݵĽ���

            AngleReceiveFinshedFlag = 1;
        }
        else
        {
            ReceiveCounter++;
        }
    }
}
/**********************************************************************************/
/*****************************���Ť��ʹ�ܺ���**************************************/
int8_t ServoTorsion_Enable(uint8_t ServoNum)
{
    uint8_t     i = 0;
    uint8_t     StrSize = 0;
    char        StrData[30] = {0};

    for(i = 0; i < ServoNum; i++)
    {
        StrSize = snprintf(StrData, 30, "WR_enable#%dP1T1\r\n", i + 1);

        if(StrSize == ERROR)
        {
            return RUN_ERROR;
        }
        else
        {
            Usart3Data_Send(StrData);
            delay_ms(10);
        }
    }

    TorsionDisableFlag = 0;

    return RUN_SUCCESS;
}
/**********************************************************************************/
/*****************************���Ť��ж������**************************************/
int8_t ServoTorsion_Disable(uint8_t ServoNum)
{
    uint8_t     i = 0;
    uint8_t     StrSize = 0;
    char        StrData[30] = {0};

    for(i = 0; i < ServoNum; i++)
    {
        StrSize = snprintf(StrData, 30, "WR_enable#%dP0T1\r\n", i + 1);

        if(StrSize == ERROR)
        {
            return RUN_ERROR;
        }
        else
        {
            Usart3Data_Send(StrData);
            delay_ms(10);
        }
    }

    TorsionDisableFlag = 1;

    return RUN_SUCCESS;
}
/**********************************************************************************/
/*****************************����Ƕȷ��ͺ���**************************************/
int8_t ServoPwmData_Send(uint16_t *ServoPwmData, uint8_t ServoNum, uint16_t ServoRunTime)
{
    uint8_t     i = 0;
    uint8_t     StrSize = 0, AddrIndex = 0;
    char        StrData[SEND_ANGLE_STR_MAX_SIZE] = {0};

    if(ServoNum > MAX_SERVO_NUM)
    {
        return RUN_ERROR;
    }

    if(TorsionDisableFlag)
    {
        ServoTorsion_Enable(ServoNum);
    }

    StrSize = snprintf(StrData, SEND_ANGLE_STR_MAX_SIZE, "Line");

    for(i = 0; i < ServoNum; i++)
    {
        AddrIndex = AddrIndex + StrSize;

        StrSize = snprintf(StrData + AddrIndex, SEND_ANGLE_STR_MAX_SIZE, "#%dP%d", i + 1, ServoPwmData[i]);
    }

    AddrIndex = AddrIndex + StrSize;
    StrSize = snprintf(StrData + AddrIndex, SEND_ANGLE_STR_MAX_SIZE, "T%d\r\n", ServoRunTime);

    if(StrSize == ERROR)
    {
        return RUN_ERROR;
    }
    else
    {
        Usart3Data_Send(StrData);
        return RUN_SUCCESS;
    }
}
/**********************************************************************************/
/*****************************����Ƕȶ�ȡ����**************************************/
int8_t ServoAngleData_Read(uint8_t ServoNum)
{
    uint8_t     i = 0;
    uint8_t     StrSize = 0;
    char        StrData[30] = {0};

    if(!TorsionDisableFlag)
    {
        TorsionDisableFlag = 1;

        ServoTorsion_Disable(ServoNum);
    }

    for(i = 0; i < ServoNum; i++)
    {
        StrSize = snprintf(StrData, 30, "RE_pos#%dP0T1\r\n", i + 1);

        if(StrSize == ERROR)
        {
            return RUN_ERROR;
        }
        else
        {
            Usart3Data_Send(StrData);
            while(!AngleReceiveFinshedFlag);
            AngleReceiveFinshedFlag = 0;

            memcpy(ServoAngleReadData[i], ServoReceiveBuffer, RECEIVE_ANGLE_STR_MAX_SIZE);
        }
    }

    return RUN_SUCCESS;
}
/**********************************************************************************/
/*****************************�����ȡ�Ƕ���ʾ����**********************************/
void ServoAngleRead_Disp(uint8_t ServoNum)
{
    uint8_t     i = 0, j = 0, k = 0;
    uint8_t     TempDatBuf[5] = {0};
    uint8_t     SpaceStr[5] = "    ";

    for(i  = 0; i < ServoNum; i++)
    {
        while(ServoAngleReadData[i][j] != 'P')
        {
            j++;
        }

        j++;

        while(ServoAngleReadData[i][j] != '\r')
        {
            TempDatBuf[k] = ServoAngleReadData[i][j];

            j++;

            k++;
        }

        k = 0;

        OLED_P6x8Str(i % 3 * 40 + 5, i / 3, SpaceStr);
        OLED_P6x8Str(i % 3 * 40 + 5, i / 3, TempDatBuf);
    }
}
/**********************************************************************************/
/****************************��ʱ��2�жϺ���****************************************/
void TIM2_IRQHandler(void)
{
    static uint8_t CountTimes = 0;

    if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)                                         //����ж�
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);                                         //����жϱ�־λ

        CountTimes++;

        RobotInstruct_Control();

        if(CountTimes == 1)
        {
            GPIO_SetBits(GPIOA, GPIO_Pin_5);
        }
        else
        {
            CountTimes = 0;

            GPIO_ResetBits(GPIOA, GPIO_Pin_5);
        }

        ServoPwmData_Send(ServoPwmData, MAX_SERVO_NUM, TIM2->ARR / 10);
    }
}
/**********************************************************************************/
/*****************************���ת���ٶȿ��ƺ���**********************************/
void ServoRunTime_Set(uint16_t ServoRunTime)
{
    TIM_SetAutoreload(TIM2, ServoRunTime * 10);
}
/*********************************************************************************/
/***********************************������ʱ������**********************************/
void Servo_Run(void)
{
    TIM_Cmd(TIM2, ENABLE);
    ServoTorsion_Enable(MAX_SERVO_NUM);
}
/**********************************************************************************/
/**********************************************************************************/
void Servo_Stop(void)
{
    TIM_Cmd(TIM2, DISABLE);
    ServoTorsion_Disable(MAX_SERVO_NUM);
}
/**********************************************************************************/
/***********************************�����ʼ������**********************************/
void Servo_Init(void)
{
    LED_Init();
    USART3_Init();
    Timer2_Init();
}
/**********************************************************************************/
