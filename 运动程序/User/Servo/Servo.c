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
/*******************************LED初始化函数***************************************/
static void LED_Init(void)
{
    //GPIO端口设置
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                                   //使能GPIOA时钟

    //LED端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;                                               //GPIOA5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                                           //输出功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                       //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                          //推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                            //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                                  //初始化PA5

    GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}
/**********************************************************************************/
/*******************************串口初始化函数**************************************/
static void USART3_Init(void)
{
    //GPIO端口设置
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);                                   //使能GPIOD时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);                                  //使能USART3时钟

    //串口3对应引脚复用映射
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);                               //GPIOD8复用为USART3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);                               //GPIOD9复用为USART3

    //USART3端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;                                  //GPIOD8与GPIOD9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                            //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                       //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                          //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                            //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);                                                  //初始化PD8，P9

    //USART3 初始化设置
    USART_InitStructure.USART_BaudRate = 115200;                                            //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                             //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                                  //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                                     //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;         //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                         //收发模式
    USART_Init(USART3, &USART_InitStructure);                                               //初始化串口3

    USART_Cmd(USART3, ENABLE);                                                              //使能串口3
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                          //开启中断
    USART_ClearFlag(USART3, USART_IT_RXNE);                                                 //清除中断标志位，否则初始化后会先进入中断

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/**********************************************************************************/
/********************************定时器2初始化函数*********************************/
static void Timer2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);                                    //使能TIM2时钟

    TIM_TimeBaseInitStructure.TIM_Period = SERVO_RUN_TIME_DEFAULT * 10;                     //每SERVO_RUN_TIME_DEFAULT ms溢出一次
    TIM_TimeBaseInitStructure.TIM_Prescaler = 8399;                                         //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;                         //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);                                     //初始化TIM2
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                                              //允许定时器2更新中断
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);                                             //清除中断标志位，否则初始化后会先进入中断
    TIM_Cmd(TIM2, DISABLE);                                                                 //使能定时器2

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                                         //定时器2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;                            //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;                                   //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/**********************************************************************************/
/*****************************串口数据发送函数**************************************/
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
/*****************************串口中断处理函数**************************************/
void USART3_IRQHandler(void)                                                                //串口3中断服务程序
{
    static uint8_t  ReceiveCounter = 0;
    static uint8_t  TempBuffer[RECEIVE_ANGLE_STR_MAX_SIZE];

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)                                   //接收中断有效,若接收数据寄存器满
    {
        TempBuffer[ReceiveCounter] = USART_ReceiveData(USART3);                             //接收数据

        if(ReceiveCounter == 0 && TempBuffer[0] != '#')
        {
            ReceiveCounter = 0;

            return;                                                                         //第0号数据不是帧头，跳过
        }

        if(TempBuffer[ReceiveCounter] == '\n')                                              //接收到最后一个数据
        {
            memcpy(ServoReceiveBuffer, TempBuffer, RECEIVE_ANGLE_STR_MAX_SIZE);
            memset(TempBuffer, 0, RECEIVE_ANGLE_STR_MAX_SIZE);

            ReceiveCounter = 0;                                                             //重新赋值，准备下一帧数据的接收

            AngleReceiveFinshedFlag = 1;
        }
        else
        {
            ReceiveCounter++;
        }
    }
}
/**********************************************************************************/
/*****************************舵机扭力使能函数**************************************/
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
/*****************************舵机扭力卸力函数**************************************/
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
/*****************************舵机角度发送函数**************************************/
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
/*****************************舵机角度读取函数**************************************/
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
/*****************************舵机读取角度显示函数**********************************/
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
/****************************定时器2中断函数****************************************/
void TIM2_IRQHandler(void)
{
    static uint8_t CountTimes = 0;

    if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)                                         //溢出中断
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);                                         //清除中断标志位

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
/*****************************舵机转动速度控制函数**********************************/
void ServoRunTime_Set(uint16_t ServoRunTime)
{
    TIM_SetAutoreload(TIM2, ServoRunTime * 10);
}
/*********************************************************************************/
/***********************************开启定时器函数**********************************/
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
/***********************************舵机初始化函数**********************************/
void Servo_Init(void)
{
    LED_Init();
    USART3_Init();
    Timer2_Init();
}
/**********************************************************************************/
