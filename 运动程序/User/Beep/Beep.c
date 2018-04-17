#include "Beep.h"

static void BeepGpio_Init(void)
{
    //GPIO端口设置
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                                   //使能GPIOA时钟

    //LED端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;                                              //GPIOA15
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                                           //输出功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                       //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                          //推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                            //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                                  //初始化PA15
}

void Beep_BliBli(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    delay_ms(100);
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    delay_ms(100);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    delay_ms(100);
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    delay_ms(100);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    delay_ms(100);
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    delay_ms(100);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    delay_ms(100);
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    delay_ms(100);
}

void Beep_On(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
}

void Beep_Off(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}

void Beep_Init(void)
{
    BeepGpio_Init();
    Beep_BliBli();
}
