#include "Bluetooth.h"

uint8_t TxBuffer[TX_LENH] = {0};
uint8_t RxBuffer[RX_LENH] = {0};

/****************************************************************************
* 名    称: USART_GPIOInit
* 功    能：UART5初始化
* 返回参数：无
* 说    明： 
****************************************************************************/
static void UART5_GPIOInit(void)
{   //GPIO端口设置
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    //PC12(TX)设置成复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //PD2(RX)设置成复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);    

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No ;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART5,&USART_InitStruct);
    USART_Cmd(UART5,ENABLE);
}

static void UART5_DMAInit(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure ;

    //启动DMA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //DMA发射通道配置
    DMA_DeInit(DMA1_Stream7);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
    //外设地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART5->DR);
    //内存地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)TxBuffer;
    //dma传输方向
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    //设置DMA在传输时缓冲区的长度
    DMA_InitStructure.DMA_BufferSize = TX_LENH;
    //设置DMA的外设递增模式，一个外设
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //设置DMA的内存递增模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //外设数据字长
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    //内存数据字长
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    //设置DMA的传输模式
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    //设置DMA的优先级别
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    //指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式  
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    //指定了FIFO阈值水平
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    //指定的Burst转移配置内存传输 
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    //指定的Burst转移配置外围转移   
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    //配置DMA1的通道
    DMA_Init(DMA1_Stream7, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Stream7, DISABLE);

    USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);

    //DMA通道配置
    DMA_DeInit(DMA1_Stream0);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    //外设地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART5->DR);
    //内存地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RxBuffer;
    //dma传输方向
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    //设置DMA在传输时缓冲区的长度
    DMA_InitStructure.DMA_BufferSize = RX_LENH;
    //设置DMA的外设递增模式，一个外设
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //设置DMA的内存递增模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //外设数据字长
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    //内存数据字长
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    //设置DMA的传输模式
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    //设置DMA的优先级别
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    //指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    //指定了FIFO阈值水平
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    //指定的Burst转移配置内存传输 
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    //指定的Burst转移配置外围转移
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    //配置DMA1的通道
    DMA_Init(DMA1_Stream0, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
    DMA_ClearFlag(DMA1_Stream0, DMA_IT_TC);                                         //清除中断标志位，否则接收的第一个字节数据无法进入中断
    DMA_Cmd(DMA1_Stream0, ENABLE);

    USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);
}

void BluetoothData_Send(uint8_t *Buff, uint16_t Len)
{
    DMA_Cmd(DMA1_Stream7, DISABLE);
    DMA1_Stream7->M0AR = (uint32_t) Buff;
    DMA1_Stream7->NDTR = Len;
    DMA_Cmd(DMA1_Stream7, ENABLE);
}

void DMA1_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream7, DMA_FLAG_TCIF7) != RESET)
    {
        DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);
    }
}

void DMA1_Stream0_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream0, DMA_FLAG_TCIF0) != RESET)
    {
        DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
    }
}

void Bluetooth_Init(void)
{
    UART5_GPIOInit();
    UART5_DMAInit();
}
