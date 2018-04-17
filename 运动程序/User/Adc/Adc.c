#include "Adc.h"

static uint16_t     AdcConvertedValue[ADC_BUF_LENGTH] = {0};
static uint16_t     AdcFilterData[ADC_BUF_LENGTH] = {0};

void  Adc_Init(void)
{
    GPIO_InitTypeDef            GPIO_InitStructure;
    ADC_CommonInitTypeDef       ADC_CommonInitStructure;
    ADC_InitTypeDef             ADC_InitStructure;
    DMA_InitTypeDef             DMA_InitStructure; 

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                                       //ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);                                        //ʹ��ADC1ʱ��

    //�ȳ�ʼ��ADC1ͨ��5 IO��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                                                   //ͨ��4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;                                                //ģ������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;                                           //����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                                      //��ʼ��

    //����DMAʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    //DMAͨ������   
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    //�����ַ
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC1->DR);
    //�ڴ��ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)AdcConvertedValue;
    //dma���䷽��
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    //����DMA�ڴ���ʱ�������ĳ���
    DMA_InitStructure.DMA_BufferSize = ADC_BUF_LENGTH;  
    //����DMA���������ģʽ��һ������
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //����DMA���ڴ����ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //���������ֳ�
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  
    //�ڴ������ֳ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  
    //����DMA�Ĵ���ģʽ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    //����DMA�����ȼ���
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
      
    //ָ�����FIFOģʽ��ֱ��ģʽ������ָ������:��ʹ��FIFOģʽ
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    //ָ����FIFO��ֵˮƽ
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    //ָ����Burstת�������ڴ洫��
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    //ָ����Burstת��������Χת�� */
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
      
    //����DMA��ͨ��
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    //ʹ��ͨ��
    DMA_Cmd(DMA2_Stream0, ENABLE); 

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);                                        //ADC1��λ
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);                                       //��λ����

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                    //����ģʽ
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;                //���������׶�֮����ӳ�5��ʱ��
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;                     //DMAʹ��
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;                                 //Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
    ADC_CommonInit(&ADC_CommonInitStructure);                                                   //��ʼ��

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                                      //12λģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                                //ɨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                                          //����ת��
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;                 //��ֹ������⣬ʹ���������
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                                      //�Ҷ���
    ADC_InitStructure.ADC_NbrOfConversion = ADC_BUF_LENGTH;                                     //���ô�ת����ͨ����Ŀ
    ADC_Init(ADC1, &ADC_InitStructure);                                                         //ADC��ʼ��

    //����ģʽͨ������
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);

    //ʹ��ADC��DMA
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);  
    ADC_DMACmd(ADC1, ENABLE);

    //ʹ��ADC1
    ADC_Cmd(ADC1, ENABLE);

    //����ADC1�����ת��
    ADC_SoftwareStartConv(ADC1);
}
///****************************************************************************
//* ��    ��: uint16_t AdcData_Get(uint8_t ch) 
//* ��    �ܣ����ADCֵ
//* ��ڲ�����ch: ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
//* ���ز�����12λADC��Чֵ
//* ˵    ����       
//****************************************************************************/
//static uint16_t AdcData_Get(uint8_t ch)
//{
//    return AdcConvertedValue[ch];
//    /*
//    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );                           //ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��

//    ADC_SoftwareStartConv(ADC1);                                                                //ʹ��ָ����ADC1�����ת����������
//     
//    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                                             //�ȴ�ת������

//    return ADC_GetConversionValue(ADC1);                                                        //�������һ��ADC1�������ת�����
//    */
//}
///****************************************************************************
//* ��    ��: uint16_t AdcAverageData_Get(uint8_t ch,uint8_t times) 
//* ��    �ܣ���ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//* ��ڲ�����ch: ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
//*           times:��ȡ����
//* ���ز�����ͨ��ch��times��ת�����ƽ��ֵ
//* ˵    ����       
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
* ��    ��: void AdcFilterData_Get(void)
* ��    �ܣ��Բɼ���ADCֵ�����˲�
* ��ڲ�������
* ���ز�������
* ˵    ����
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
