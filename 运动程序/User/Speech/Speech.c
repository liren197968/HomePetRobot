#include "Speech.h"

uint8_t     SpeechReturnData = 0;

/*****************************串口初始化函数****************************************/
void Speech_Init(void)
{
   //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);                           //使能GPIOC时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);                           //使能UART4时钟

    //串口4对应引脚复用映射
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);                       //GPIOC10复用为USART4
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);                       //GPIOC11复用为USART4

    //UART4端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;                        //GPIOC10与GPIOC11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                    //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                               //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                  //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                    //上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);                                          //初始化PC10，PC11

    //USART4 初始化设置
    USART_InitStructure.USART_BaudRate = 115200;                                    //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //收发模式
    USART_Init(UART4, &USART_InitStructure);                                        //初始化串口4
    USART_Cmd(UART4, ENABLE);                                                       //使能串口4
    USART_ClearFlag(UART4, USART_FLAG_TC);

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);                                   //开启相关中断

    //USART4 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;                                //串口4中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;                       //抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;                              //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                 //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                                 //根据指定的参数初始化VIC寄存器
}
/**********************************************************************************/
/*****************************串口数据发送函数**************************************/
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
/*****************************播报待机提示语音函数**********************************/
void StandBySpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0001$");
}
/**********************************************************************************/
/*****************************播报前进提示语音函数**********************************/
void ForwardSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0002$");
}
/**********************************************************************************/
/*****************************播报后退提示语音函数**********************************/
void BackwrdSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0003$");
}
/**********************************************************************************/
/*****************************播报顺转提示语音函数**********************************/
void ClckWisSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0004$");
}
/**********************************************************************************/
/*****************************播报逆转提示语音函数**********************************/
void AtiClckSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0005$");
}
/**********************************************************************************/
/*****************************播报变形提示语音函数**********************************/
void TrasfrmSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0006$");
}
/**********************************************************************************/
/*****************************播报前滚提示语音函数**********************************/
void FrdRollSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0007$");
}
/**********************************************************************************/
/*****************************播报后滚提示语音函数**********************************/
void BakRollSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0008$");
}
/**********************************************************************************/
/*****************************播报手势提示语音函数**********************************/
void GestureSpeech_Play(void)
{
    SpeechData_Send("@PlayTF#0009$");
}
/**********************************************************************************/
/********************************停止语音播放函数***********************************/
void Mp3SpeechStop_Play(void)
{
    SpeechData_Send("@StopPlaying#$");
}
/**********************************************************************************/
/*****************************内容应答更新指令函数**********************************/
void TxtUpdaSpeech_Play(void)
{
    SpeechData_Send("@WriteKeywords#启动 001|前进 002|后退 003|顺转 004|逆转圈 005|变形 006|前滚 007|后滚 008|启动手势识别 009|讲个故事 010|放首音乐 011|$");
    delay_ms(5000);
    SpeechData_Send("@WriteFlashText#$");
    delay_ms(1000);
}
/**********************************************************************************/
/********************************串口中断函数***************************************/
void UART4_IRQHandler(void)                                                         //串口4中断服务程序
{
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)                            //接收中断(接收到的数据为单个字节)
    {
        SpeechReturnData = USART_ReceiveData(UART4);                                //读取接收到的数据
    }
}
/**********************************************************************************/
