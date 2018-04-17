#include "Bluetooth.h"

uint8_t TxBuffer[TX_LENH] = {0};
uint8_t RxBuffer[RX_LENH] = {0};

/****************************************************************************
* ��    ��: USART_GPIOInit
* ��    �ܣ�UART5��ʼ��
* ���ز�������
* ˵    ���� 
****************************************************************************/
static void UART5_GPIOInit(void)
{   //GPIO�˿�����
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    //PC12(TX)���óɸ����������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //PD2(RX)���óɸ����������
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

    //����DMAʱ��
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

    //DMA����ͨ������
    DMA_DeInit(DMA1_Stream7);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
    //�����ַ
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART5->DR);
    //�ڴ��ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)TxBuffer;
    //dma���䷽��
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    //����DMA�ڴ���ʱ�������ĳ���
    DMA_InitStructure.DMA_BufferSize = TX_LENH;
    //����DMA���������ģʽ��һ������
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //����DMA���ڴ����ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //���������ֳ�
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    //�ڴ������ֳ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    //����DMA�Ĵ���ģʽ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    //����DMA�����ȼ���
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    //ָ�����FIFOģʽ��ֱ��ģʽ������ָ������ �� ��ʹ��FIFOģʽ  
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    //ָ����FIFO��ֵˮƽ
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    //ָ����Burstת�������ڴ洫�� 
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    //ָ����Burstת��������Χת��   
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    //����DMA1��ͨ��
    DMA_Init(DMA1_Stream7, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Stream7, DISABLE);

    USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);

    //DMAͨ������
    DMA_DeInit(DMA1_Stream0);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    //�����ַ
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART5->DR);
    //�ڴ��ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RxBuffer;
    //dma���䷽��
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    //����DMA�ڴ���ʱ�������ĳ���
    DMA_InitStructure.DMA_BufferSize = RX_LENH;
    //����DMA���������ģʽ��һ������
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //����DMA���ڴ����ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //���������ֳ�
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    //�ڴ������ֳ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    //����DMA�Ĵ���ģʽ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    //����DMA�����ȼ���
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    //ָ�����FIFOģʽ��ֱ��ģʽ������ָ������ �� ��ʹ��FIFOģʽ
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    //ָ����FIFO��ֵˮƽ
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    //ָ����Burstת�������ڴ洫�� 
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    //ָ����Burstת��������Χת��
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    //����DMA1��ͨ��
    DMA_Init(DMA1_Stream0, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
    DMA_ClearFlag(DMA1_Stream0, DMA_IT_TC);                                         //����жϱ�־λ��������յĵ�һ���ֽ������޷������ж�
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
