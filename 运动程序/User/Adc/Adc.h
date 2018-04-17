#ifndef __ADC_H
#define __ADC_H

#include "common.h" 

#define ADC_BUF_LENGTH          1                                       //ADCͨ��ת����Ϊ1
#define FILTER_TIMES            10                                      //��ֵ�˲�����Ϊ10
#define SAMPLE_CYCLE            2                                       //���ò�������Ϊ2
#define BATTERY_LIMIT           7.40                                    //���õ����͹�����ѹΪ7.4V

void Adc_Init(void);                                                        //ADCͨ����ʼ��
float BatteryVoltage_Get(void);                                             //��ȡ��ص�ѹֵ

#endif
