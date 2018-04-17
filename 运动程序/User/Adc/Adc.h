#ifndef __ADC_H
#define __ADC_H

#include "common.h" 

#define ADC_BUF_LENGTH          1                                       //ADC通道转换数为1
#define FILTER_TIMES            10                                      //均值滤波次数为10
#define SAMPLE_CYCLE            2                                       //设置采样周期为2
#define BATTERY_LIMIT           7.40                                    //设置电池最低工作电压为7.4V

void Adc_Init(void);                                                        //ADC通道初始化
float BatteryVoltage_Get(void);                                             //获取电池电压值

#endif
