#include "RgbLed.h"

static void RgbLedGpio_Init(void)
{
    //GPIO�˿�����
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);                               //ʹ��GPIODʱ��

    //LED�˿����� 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;              //GPIOD12��GPIOD13��GPIOD14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                                       //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                   //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                      //�������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                        //����
    GPIO_Init(GPIOD, &GPIO_InitStructure);                                              //��ʼ��PD12��PD13��PD14
}

void RedLight_TurnOn(void)
{
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

void GreLight_TurnOn(void)
{
    GPIO_SetBits(GPIOD, GPIO_Pin_13);
    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

void BluLight_TurnOn(void)
{
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
}

void YelLight_TurnOn(void)
{
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    GPIO_SetBits(GPIOD, GPIO_Pin_13);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

void PurLight_TurnOn(void)
{
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
}

void WhiLight_TurnOn(void)
{
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    GPIO_SetBits(GPIOD, GPIO_Pin_13);
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
}

void Light_TurnOff(void)
{
    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

void RgbLed_Init(void)
{
    RgbLedGpio_Init();

    RedLight_TurnOn();
    delay_ms(100);

    GreLight_TurnOn();
    delay_ms(100);

    BluLight_TurnOn();
    delay_ms(100);

    PurLight_TurnOn();
    delay_ms(100);

    Light_TurnOff();
}
