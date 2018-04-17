#include "Adc.h"

static uint16_t     AdcConvertedValue[ADC_BUF_LENGTH] = {0};
static uint16_t     AdcFilterData[ADC_BUF_LENGTH] = {0};

void  Adc_Init(void)
{
    GPIO_InitTypeDef            GPIO_InitStructure;
    ADC_CommonInitTypeDef       ADC_CommonInitStructure;
    ADC_InitTypeDef             ADC_InitStructure;
    DMA_InitTypeDef             DMA_InitStructure; 

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                                       //使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);                                        //使能ADC1时钟

    //先初始化ADC1通道5 IO口
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                                                   //通道4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;                                                //模拟输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;                                           //不带上下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                                      //初始化

    //启动DMA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    //DMA通道配置   
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    //外设地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC1->DR);
    //内存地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)AdcConvertedValue;
    //dma传输方向
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    //设置DMA在传输时缓冲区的长度
    DMA_InitStructure.DMA_BufferSize = ADC_BUF_LENGTH;  
    //设置DMA的外设递增模式，一个外设
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //设置DMA的内存递增模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //外设数据字长
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  
    //内存数据字长
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  
    //设置DMA的传输模式
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    //设置DMA的优先级别
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
      
    //指定如果FIFO模式或直接模式将用于指定的流:不使能FIFO模式
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    //指定了FIFO阈值水平
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    //指定的Burst转移配置内存传输
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    //指定的Burst转移配置外围转移 */
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
      
    //配置DMA的通道
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    //使能通道
    DMA_Cmd(DMA2_Stream0, ENABLE); 

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);                                        //ADC1复位
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);                                       //复位结束

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                    //独立模式
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;                //两个采样阶段之间的延迟5个时钟
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;                     //DMA使能
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;                                 //预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
    ADC_CommonInit(&ADC_CommonInitStructure);                                                   //初始化

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                                      //12位模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                                //扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                                          //连续转换
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;                 //禁止触发检测，使用软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                                      //右对齐
    ADC_InitStructure.ADC_NbrOfConversion = ADC_BUF_LENGTH;                                     //设置待转换的通道数目
    ADC_Init(ADC1, &ADC_InitStructure);                                                         //ADC初始化

    //规则模式通道配置
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);

    //使能ADC的DMA
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);  
    ADC_DMACmd(ADC1, ENABLE);

    //使能ADC1
    ADC_Cmd(ADC1, ENABLE);

    //开启ADC1的软件转换
    ADC_SoftwareStartConv(ADC1);
}
///****************************************************************************
//* 名    称: uint16_t AdcData_Get(uint8_t ch) 
//* 功    能：获得ADC值
//* 入口参数：ch: 通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//* 返回参数：12位ADC有效值
//* 说    明：       
//****************************************************************************/
//static uint16_t AdcData_Get(uint8_t ch)
//{
//    return AdcConvertedValue[ch];
//    /*
//    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );                           //ADC1,ADC通道,480个周期,提高采样时间可以提高精确度

//    ADC_SoftwareStartConv(ADC1);                                                                //使能指定的ADC1的软件转换启动功能
//     
//    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                                             //等待转换结束

//    return ADC_GetConversionValue(ADC1);                                                        //返回最近一次ADC1规则组的转换结果
//    */
//}
///****************************************************************************
//* 名    称: uint16_t AdcAverageData_Get(uint8_t ch,uint8_t times) 
//* 功    能：获取通道ch的转换值，取times次,然后平均 
//* 入口参数：ch: 通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//*           times:获取次数
//* 返回参数：通道ch的times次转换结果平均值
//* 说    明：       
//****************************************************************************/
//static uint16_t AdcAverageData_Get(uint8_t ch, uint8_t times)
//{
//    uint8_t             i = 0;
//    uint32_t            temp_val=0;

//    for(i = 0; i < times; i++)
//    {
//        temp_val += AdcData_Get(ch);
//        delay_ms(5);
//    }

//    return temp_val / times;
//} 
/****************************************************************************
* 名    称: void AdcFilterData_Get(void)
* 功    能：对采集的ADC值进行滤波
* 入口参数：无
* 返回参数：无
* 说    明：
****************************************************************************/
float BatteryVoltage_Get(void)
{
    uint8_t             i;
    uint16_t            SumData = 0;
    static uint16_t     SampleData[1][FILTER_TIMES];
    static uint8_t      SampleIndex = 0, SampleTimes = 0;
    static float        BatteryVoltage = BATTERY_LIMIT + 0.1;

    SampleTimes++;

    if(SampleTimes == SAMPLE_CYCLE)
    {
        SampleTimes = 0;

        SampleData[0][SampleIndex] = AdcConvertedValue[0];

        SampleIndex++;
    }

    if(SampleIndex == FILTER_TIMES)
    {
        SampleIndex = 0;

        SumData = 0;

        for(i = 0; i < FILTER_TIMES; i++)
        {
            SumData = SumData + SampleData[0][i];
        }

        AdcFilterData[0] = SumData / FILTER_TIMES;

        BatteryVoltage = AdcFilterData[0] * 3.3 / 4096  * 4 + 0.65;

        return BatteryVoltage;
    }
    else
    {
        return BatteryVoltage;
    }
}
