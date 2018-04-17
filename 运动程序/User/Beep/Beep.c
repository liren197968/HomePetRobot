#include "Beep.h"

static void BeepGpio_Init(void)
{
    //GPIO�˿�����
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                                   //ʹ��GPIOAʱ��

    //LED�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;                                              //GPIOA15
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                                           //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                       //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                          //�������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                            //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                                  //��ʼ��PA15
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
