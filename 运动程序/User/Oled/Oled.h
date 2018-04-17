#ifndef _OLED_H
#define _OLED_H

#include "common.h"

#define     OLED_RST_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_2) 
#define     OLED_RST_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_2) 
#define     OLED_DC_HIGH()      GPIO_SetBits(GPIOB,GPIO_Pin_1) 
#define     OLED_DC_LOW()       GPIO_ResetBits(GPIOB,GPIO_Pin_1) 

#define     XLevelL             0x00
#define     XLevelH             0x10
#define     XLevel              ((XLevelH&0x0F)*16+XLevelL)
#define     Max_Column          128
#define     Max_Row             64
#define     Brightness          0xCF

#define     X_WIDTH             128
#define     Y_WIDTH             64

void OLED_Init(void);
void OLED_CLS(void);
void OLED_Fill(unsigned char dat);
void OLED_P6x8Str(unsigned char x,unsigned char y,unsigned char ch[]);
void OLED_P6x8Number(unsigned char x,unsigned char y, float number);
void OLED_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
void OLED_P8x16Number(unsigned char x,unsigned char y, float number);
void OLED_P14x16Ch(unsigned char x,unsigned char y,unsigned char N);

#endif
