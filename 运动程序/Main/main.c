#include "Adc.h"
#include "Oled.h"
#include "Beep.h"
#include "Servo.h"
#include "Usart.h"
#include "Speech.h"
#include "RgbLed.h"
#include "Mpu6050.h"
#include "Bluetooth.h"
#include "RobotControl.h"

double        BatteryVoltage = 0;

void EulerAngle_Disp(void)
{
    static uint8_t      SpaceStr[7] = "       ";

    OLED_P6x8Str(87, 6, SpaceStr);
    OLED_P6x8Number(87, 6, BatteryVoltage);

    OLED_P6x8Str(10, 7, SpaceStr);
    OLED_P6x8Number(10, 7, pitch);

    OLED_P6x8Str(52, 7, SpaceStr);
    OLED_P6x8Number(52, 7, roll);

    OLED_P6x8Str(87, 7, SpaceStr);
    OLED_P6x8Number(87, 7, yaw);

    printf("%f  %f  %f \r\n", pitch, roll, yaw);
}

void RobotLed_Disp(void)
{
    RedLight_TurnOn();
    delay_ms(1000);
    GreLight_TurnOn();
    delay_ms(1000);
    BluLight_TurnOn();
    delay_ms(1000);
    YelLight_TurnOn();
    delay_ms(1000);
    PurLight_TurnOn();
    delay_ms(1000);
    WhiLight_TurnOn();
    delay_ms(1000);
    Light_TurnOff();
    delay_ms(1000);
}

int main(void)
{
    int8_t      Mpu6050Error = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    Adc_Init();
    OLED_Init();
    Servo_Init();
    Speech_Init();
    RgbLed_Init();
    USART1_Init();
    Bluetooth_Init();

    Mpu6050Error = Mpu6050_Init();

    while(Mpu6050Error)
    {
        if(Mpu6050Error == MPU_INIT_ERROR)
        {
            YelLight_TurnOn();
            delay_ms(200);

            BluLight_TurnOn();
            delay_ms(200);
        }
        else
        {
            PurLight_TurnOn();
            delay_ms(200);

            BluLight_TurnOn();
            delay_ms(200);
        }
    }

    Servo_Run();
    Beep_Init();

    RobotHexShape_Stand();
    delay_ms(4000);

    StandBySpeech_Play();
    printf("初始化成功 \r\n");

    //RxBuffer[0] = 'Y';

    while(1)
    {

        BatteryVoltage = BatteryVoltage_Get();

        if(BatteryVoltage >= BATTERY_LIMIT)
        {
            EulerAngle_Disp();
            EulerAngle_Get(&pitch, &roll, &yaw);
            delay_ms(50);
        }
        else
        {
            Servo_Stop();
            Beep_BliBli();
        }
    }
}
