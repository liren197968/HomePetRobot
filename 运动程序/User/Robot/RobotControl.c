#include "Servo.h"
#include "Speech.h"
#include "Mpu6050.h"
#include "Bluetooth.h"
#include "RobotControl.h"

uint8_t                         UPBoxLockFlag = 0;
static uint8_t                  RollFinshFlag = 0;
static uint8_t                  RollReadyFlag = 0;
static uint8_t                  StartMoveFlag = 0;                                                                  //舵机数据更新完毕标记
static uint8_t                  HandGestureFlag = 0;                                                                //触发综合功能标记
static uint8_t                  RemoteControlData = 0;

static double                   DhDegree[3];
static double                   ExpectedAngle = 0;
static double                   FAngleError = 0, SAngleError = 0;

static uint16_t                 RobotCrepSpeed = 200;
static uint16_t                 RobotTsfmSpeed = 600;
static uint16_t                 RobotRollSpeed = 500;

uint16_t                        ServoPwmData[32] = {SERVO_PWM_CENTER, SERVO_PWM_CENTER_1, SERVO_PWM_CENTER_2,
                                                    SERVO_PWM_CENTER, SERVO_PWM_CENTER_1, SERVO_PWM_CENTER_2,
                                                    SERVO_PWM_CENTER, SERVO_PWM_CENTER_1, SERVO_PWM_CENTER_2,
                                                    SERVO_PWM_CENTER, SERVO_PWM_CENTER_1, SERVO_PWM_CENTER_2,
                                                    SERVO_PWM_CENTER, SERVO_PWM_CENTER_1, SERVO_PWM_CENTER_2,
                                                    SERVO_PWM_CENTER, SERVO_PWM_CENTER_1, SERVO_PWM_CENTER_2,
                                                    UP_BOX_LEG_OPEN,  UP_BOX_FET_OPEN, UP_BOX_LEG_OPEN, UP_BOX_FET_OPEN,
                                                    UP_BOX_LEG_OPEN,  UP_BOX_FET_OPEN, UP_BOX_LEG_OPEN, UP_BOX_FET_OPEN,
                                                    UP_BOX_LEG_OPEN,  UP_BOX_FET_OPEN, UP_BOX_LEG_OPEN, UP_BOX_FET_OPEN,
                                                    UP_BOX_LEG_OPEN,  UP_BOX_FET_OPEN};
static uint16_t                 SServoPwmData[36] = {0};
static uint16_t                 FServoPwmData[36] = {0};
static uint16_t                 BServoPwmData[36] = {0};
static uint16_t                 CServoPwmData[36] = {0};
static uint16_t                 LFServoPwmData[36] = {0};
static uint16_t                 LBServoPwmData[36] = {0};
static uint16_t                 RFServoPwmData[36] = {0};
static uint16_t                 RBServoPwmData[36] = {0};
/*************************D-H参数法反解计算函数*************************/
static void DhAlgorithm_Reverse(double x, double y, double z)
{
    double                      a = 0, e = 0, f = 0, h = 0, j = 0;

    DhDegree[0] = (atan(y / x)) * 180 / 3.141593;

    a = x * cos( (DhDegree[0] * 3.141593) / 180) + y * sin( (DhDegree[0] * 3.141593) / 180) - A1_LENH;

    DhDegree[2] = ( (acos( ( pow( a, 2 ) + pow ( ( z - D1_LENH ), 2) - pow (A3_LENH, 2)
                    - pow(A2_LENH, 2) ) / ( 2 * A2_LENH * A3_LENH ) ) ) ) * 180 / 3.141593;

    if( (DhDegree[2] > 0) || (DhDegree[2] < -180) )                                                                 //多解的转换到预定范围内 [0~(-180)]
    { 
        if( (DhDegree[2] > 0) && (DhDegree[2] <= 180) )
        {
            DhDegree[2] = -DhDegree[2];
        }

        if( (DhDegree[2] > 180) && (DhDegree[2] <= 360) )
        {
            DhDegree[2] = DhDegree[2] - 360;
        }

        if( (DhDegree[2] < -180) && (DhDegree[2] >= -360) )
        {
            DhDegree[2] = -(DhDegree[2] + 360);
        }
    }

    e = cos(DhDegree[2] * ANGLE_TO_RADIAN);
    f = sin(DhDegree[2] * ANGLE_TO_RADIAN);
    h = (e * A3_LENH + A2_LENH ) * (z - D1_LENH) - a * f * A3_LENH;
    j = a * (A3_LENH * e + A2_LENH) + A3_LENH * f * (z - D1_LENH);

    DhDegree[1] = (atan2(h, j) ) * 180 / 3.141593;
}
/*********************************************************************/
/************************计算补偿修正后的坐标值*************************/
static void ReviseCoordinate_Compute(uint8_t LegNum, double DeltaAlpha, double *x, double *y, double *z)
{
    static double           l = 0, r = 0;
    static double           c1 = 0, alfa1 = 0;
    static double           c2 = 0, alfa2 = 0;
    static double           c3 = 0, alfa3 = 0;
    static double           c4 = 0, alfa4 = 0;
    static double           c5 = 0, alfa5 = 0;
    static double           c6 = 0, alfa6 = 0;
    static double           gamma1 = 0, gamma2 = 0, gamma3 = 0;

    if(LegNum == 1)
    {
        r = ROBOT_LENH;
        l = STEP_LENH;

        gamma1 = BETA_INIT_1 - DeltaAlpha;
        gamma2 = BETA_INIT_2 - DeltaAlpha;
        gamma3 = BETA_INIT_3 - DeltaAlpha;

        c1 = sqrt( r * r - 2 * l * r * cos(gamma1 * ANGLE_TO_RADIAN) + l * l);
        c2 = sqrt( r * r - 2 * l * r * cos(gamma2 * ANGLE_TO_RADIAN) + l * l);
        c3 = sqrt( r * r - 2 * l * r * cos(gamma3 * ANGLE_TO_RADIAN) + l * l);
        c4 = sqrt( r * r - 2 * l * r * cos( (180 - gamma1) * ANGLE_TO_RADIAN ) + l * l);
        c5 = sqrt( r * r - 2 * l * r * cos( (180 - gamma2) * ANGLE_TO_RADIAN ) + l * l);
        c6 = sqrt( r * r - 2 * l * r * cos( (180 - gamma3) * ANGLE_TO_RADIAN ) + l * l);

        alfa1 = acos( (c1 * c1 + r * r - l * l) / (2 * c1 * r));
        alfa2 = acos( (c2 * c2 + r * r - l * l) / (2 * c2 * r));
        alfa3 = acos( (c3 * c3 + r * r - l * l) / (2 * c3 * r));
        alfa4 = acos( (c4 * c4 + r * r - l * l) / (2 * c4 * r));
        alfa5 = acos( (c5 * c5 + r * r - l * l) / (2 * c5 * r));
        alfa6 = acos( (c6 * c6 + r * r - l * l) / (2 * c6 * r));
    }

    if(LegNum == 1)
    {
        *x = c1 * cos(THET_INIT_1 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN + alfa1);
        *y = c1 * sin(THET_INIT_1 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN + alfa1);
    }
    else if(LegNum == 2)
    {
        *x = c2 * cos(THET_INIT_2 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN + alfa2);
        *y = c2 * sin(THET_INIT_2 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN + alfa2);
    }
    else if(LegNum == 3)
    {
        *x = c3 * cos(THET_INIT_3 * ANGLE_TO_RADIAN - DeltaAlpha * ANGLE_TO_RADIAN - alfa3);
        *y = c3 * sin(THET_INIT_3 * ANGLE_TO_RADIAN - DeltaAlpha * ANGLE_TO_RADIAN - alfa3);
    }
    else if(LegNum == 4)
    {
        *x = c4 * cos(THET_INIT_1 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN - alfa4);
        *y = c4 * sin(THET_INIT_1 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN - alfa4);
    }
    else if(LegNum == 5)
    {
        *x = c5 * cos(THET_INIT_2 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN - alfa5);
        *y = c5 * sin(THET_INIT_2 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN - alfa5);
    }
    else if(LegNum == 6)
    {
        *x = c6 * cos(THET_INIT_3 * ANGLE_TO_RADIAN - DeltaAlpha * ANGLE_TO_RADIAN + alfa6);
        *y = c6 * sin(THET_INIT_3 * ANGLE_TO_RADIAN - DeltaAlpha * ANGLE_TO_RADIAN + alfa6);
    }

    *z = -ROBOT_HIGH;
}
/**********************************************************************/
/***************************开环运动控制********************************/
static void MoveCoordinate_Calculate(double FStepLength, double SStepLength)
{
    double                  x = 0, y = 0, z = 0;

    /***********the coordinate datas of the first group legs ****************/
    x = FONT_X_INIT;
    y = FONT_Y_INIT + FStepLength;
    z = FONT_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    SServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


    x = MIDE_X_INIT;
    y = MIDE_Y_INIT + FStepLength;
    z = MIDE_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    SServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - MIDE_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);

    x = BACK_X_INIT;
    y = BACK_Y_INIT - FStepLength;
    z = BACK_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    SServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    /************************************************************************/

    /***********the coordinate datas of the second group legs ***************/
    x = FONT_X_INIT;
    y = FONT_Y_INIT + SStepLength;
    z = FONT_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    SServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);

    x = MIDE_X_INIT;
    y = MIDE_Y_INIT + SStepLength;
    z = MIDE_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    SServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - MIDE_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);

    x = BACK_X_INIT;
    y = BACK_Y_INIT - SStepLength;
    z = BACK_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    SServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    SServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    /***********************************************************************/
}
/**********************************************************************/
/***********************转圈运动计算函数********************************/
static void CircleCoordinate_Calculate(uint8_t DeltaAngle)
{
    double          x = 0, y = 0, z = 0;

    /***********the shifting datas of the first group legs ******************/
    x = ROBOT_LENH * cos( (FONT_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (FONT_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    z = FONT_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


    x = ROBOT_LENH * cos( (MIDE_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (MIDE_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    z = MIDE_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - MIDE_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


    x = ROBOT_LENH * cos( (BACK_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (BACK_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    z = BACK_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    /************************************************************************/

    /***********the shifting datas of the second group legs *****************/
    x = ROBOT_LENH * cos( (FONT_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (FONT_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    z = FONT_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


    x = ROBOT_LENH * cos( (MIDE_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (MIDE_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    z = MIDE_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - MIDE_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);

    x = ROBOT_LENH * cos( (BACK_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (BACK_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    z = BACK_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
}
/**********************************************************************/
/*************************步长修正计算函数******************************/
static void AdjustStepLen_Calculate(double ExpectedAngle, uint8_t direction)
{
    double      CurrentAngle = 0;
    double      x = 0, y = 0, z = 0;

    CurrentAngle = yaw;
/********************************前进功能***************************************************************/
    if(direction == 0)
    {
        FAngleError = (CurrentAngle - ExpectedAngle) * ROBOT_PID_CONST_P;
        SAngleError = (ExpectedAngle - CurrentAngle) * ROBOT_PID_CONST_P + BASIC_FSTEP_ERROR;

        if(FAngleError >= 15)
        {
            FAngleError = 15;
        }
        else
        {
            if(FAngleError < -15)
            {
                FAngleError = -15;
            }
        }

        if(SAngleError >= 15)
        {
            SAngleError = 15;
        }
        else
        {
            if(SAngleError < -15)
            {
                SAngleError = -15;
            }
        }
        /*****************************************第一组腿***********************************************/
        ReviseCoordinate_Compute(1, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(3, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(1, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(3, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);

        /*****************************************第一组腿************操作与前一组操作相同***************/
        ReviseCoordinate_Compute(4, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(6, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(4, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(6, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    }
/********************************后退功能*****************后退操作与前进操作相同************************/
    if(direction == 1)
    {
        FAngleError = (CurrentAngle - ExpectedAngle) * ROBOT_PID_CONST_P;
        SAngleError = (ExpectedAngle - CurrentAngle) * ROBOT_PID_CONST_P + BASIC_FSTEP_ERROR;

        if(FAngleError >= 15)
        {
            FAngleError = 15;
        }
        else
        {
            if(FAngleError < -15)
            {
                FAngleError = -15;
            }
        }
        
        if(SAngleError >= 15)
        {
            SAngleError = 15;
        }
        else
        {
            if(SAngleError < -15)
            {
                SAngleError = -15;
            }
        }
        /*****************************************第一组腿***********************************************/
        ReviseCoordinate_Compute(3, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(1, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(3, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);

        ReviseCoordinate_Compute(5, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(1, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);

        /*****************************************第一组腿************操作与前一组操作相同***************/
        ReviseCoordinate_Compute(6, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(4, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(6, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(4, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    }
}
/**********************************************************************/
/************************右步长调整函数*********************************/
static void AdjustRStepLen_Calculate(double ExpectedAngle, uint8_t direction)
{
    double      CurrentAngle = 0;
    double      x = 0, y = 0, z = 0;

    CurrentAngle = yaw;
/********************************前进功能**************************************************************/
    if(direction == 0)
    {
        FAngleError = (CurrentAngle - ExpectedAngle) * ROBOT_PID_CONST_P;
        SAngleError = (ExpectedAngle - CurrentAngle) * ROBOT_PID_CONST_P + BASIC_LFSTEP_ERROR;

        if(FAngleError >= 15)
        {
            FAngleError = 15;
        }
        else
        {
            if(FAngleError < -15)
            {
                FAngleError = -15;
            }
        }

        if(SAngleError >= 15)
        {
            SAngleError = 15;
        }
        else
        {
            if(SAngleError < -15)
            {
                SAngleError = -15;
            }
        }
        /*****************************************第一组腿***********************************************/
        ReviseCoordinate_Compute(1, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - FONT_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(3, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(1, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(3, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (BACK_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);

/*****************************************第一组腿**********************操作与前一组操作相同*****/
        ReviseCoordinate_Compute(4, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - FONT_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(6, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(4, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(6, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LFServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (BACK_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LFServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    }
/********************************后退功能*****************后退操作与前进操作相同**************************/
    if(direction == 1)
    {
        FAngleError = (CurrentAngle - ExpectedAngle) * ROBOT_PID_CONST_P;
        SAngleError = (ExpectedAngle - CurrentAngle) * ROBOT_PID_CONST_P + BASIC_LBSTEP_ERROR;

        if(FAngleError >= 15)
        {
            FAngleError = 15;
        }
        else
        {
            if(FAngleError < -15)
            {
                FAngleError = -15;
            }
        }

        if(SAngleError >= 20)
        {
            SAngleError = 20;
        }
        else
        {
            if(SAngleError < -20)
            {
                SAngleError = -20;
            }
        }
        /*****************************************第一组腿***********************************************/
        ReviseCoordinate_Compute(3, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - FONT_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(1, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(3, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(1, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (BACK_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);

/*****************************************第一组腿**********************操作与前一组操作相同*****/
        ReviseCoordinate_Compute(6, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - FONT_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(4, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(6, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(4, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        LBServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (BACK_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        LBServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    }
}
/**********************************************************************/
/***********************左步长调整函数**********************************/
static void AdjustLStepLen_Calculate(double ExpectedAngle, uint8_t direction)
{
    double      CurrentAngle = 0;
    double      x = 0, y = 0, z = 0;

    CurrentAngle = yaw;
/********************************前进功能***************************************************************/
    if(direction == 0)
    {
        FAngleError = (CurrentAngle - ExpectedAngle) * ROBOT_PID_CONST_P;
        SAngleError = (ExpectedAngle - CurrentAngle) * ROBOT_PID_CONST_P + BASIC_RFSTEP_ERROR;

        if(FAngleError >= 15)
        {
            FAngleError = 15;
        }
        else
        {
            if(FAngleError < -15)
            {
                FAngleError = -15;
            }
        }

        if(SAngleError >= 15)
        {
            SAngleError = 15;
        }
        else
        {
            if(SAngleError < -15)
            {
                SAngleError = -15;
            }
        }
        /*****************************************第一组腿***********************************************/
        ReviseCoordinate_Compute(1, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(3, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (BACK_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(1, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - FONT_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(3, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第一组腿**********************操作与前一组操作相同*****/
        ReviseCoordinate_Compute(4, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(6, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (BACK_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(4, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - FONT_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(6, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RFServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RFServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    }
/********************************后退功能*****************后退操作与前进操作相同************************/
    if(direction == 1)
    {
        FAngleError = (CurrentAngle - ExpectedAngle) * ROBOT_PID_CONST_P + BASIC_RBSTEP_ERROR;
        SAngleError = (ExpectedAngle - CurrentAngle) * ROBOT_PID_CONST_P;

        if(FAngleError >= 15)
        {
            FAngleError = 15;
        }
        else
        {
            if(FAngleError < -15)
            {
                FAngleError = -15;
            }
        }

        if(SAngleError >= 15)
        {
            SAngleError = 15;
        }
        else
        {
            if(SAngleError < -15)
            {
                SAngleError = -15;
            }
        }
        /*****************************************第一组腿***********************************************/
        ReviseCoordinate_Compute(3, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(1, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (BACK_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(3, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - FONT_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(1, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第一组腿**********************操作与前一组操作相同*****/
        ReviseCoordinate_Compute(6, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(4, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (BACK_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(6, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - FONT_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER + (FONT_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER + (FONT_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - DhDegree[0]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(4, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        RBServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (DhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER + (BACK_LEG_INIT_ANGLE + DhDegree[1]) * SERVO_ANGLE_TO_PWM);
        RBServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER + (BACK_FET_INIT_ANGLE - DhDegree[2]) * SERVO_ANGLE_TO_PWM);
    }
}
/**********************************************************************/
/************************机器人第一条腿控制函数**************************/
static void FirstLeg_Control(uint16_t FirstParm, uint16_t SecndParm, uint16_t ThirdParm)
{
    ServoPwmData[0] = FirstParm;
    ServoPwmData[1] = SecndParm;
    ServoPwmData[2] = ThirdParm;
}
/**********************************************************************/
/************************机器人第二条腿控制函数**************************/
static void SecndLeg_Control(uint16_t FirstParm, uint16_t SecndParm, uint16_t ThirdParm)
{
    ServoPwmData[3] = FirstParm;
    ServoPwmData[4] = SecndParm;
    ServoPwmData[5] = ThirdParm;
}
/**********************************************************************/
/************************机器人第三条腿控制函数**************************/
static void ThirdLeg_Control(uint16_t FirstParm, uint16_t SecndParm, uint16_t ThirdParm)
{
    ServoPwmData[6] = FirstParm;
    ServoPwmData[7] = SecndParm;
    ServoPwmData[8] = ThirdParm;
}
/**********************************************************************/
/************************机器人第四条腿控制函数**************************/
static void ForthLeg_Control(uint16_t FirstParm, uint16_t SecndParm, uint16_t ThirdParm)
{
    ServoPwmData[9] = FirstParm;
    ServoPwmData[10] = SecndParm;
    ServoPwmData[11] = ThirdParm;
}
/**********************************************************************/
/************************机器人第五条腿控制函数**************************/
static void FifthLeg_Control(uint16_t FirstParm, uint16_t SecndParm, uint16_t ThirdParm)
{
    ServoPwmData[12] = FirstParm;
    ServoPwmData[13] = SecndParm;
    ServoPwmData[14] = ThirdParm;
}
/**********************************************************************/
/************************机器人第六条腿控制函数**************************/
static void SixthLeg_Control(uint16_t FirstParm, uint16_t SecndParm, uint16_t ThirdParm)
{
    ServoPwmData[15] = FirstParm;
    ServoPwmData[16] = SecndParm;
    ServoPwmData[17] = ThirdParm;
}
/**********************************************************************/
/************************机器人第一条手臂控制函数************************/
static void FirstHand_Control(uint16_t FirstParm, uint16_t SecndParm)
{
    ServoPwmData[18] = FirstParm;
    ServoPwmData[19] = SecndParm;
}
/**********************************************************************/
/************************机器人第二条手臂控制函数************************/
static void SecndHand_Control(uint16_t FirstParm, uint16_t SecndParm)
{
    ServoPwmData[20] = FirstParm;
    ServoPwmData[21] = SecndParm;
}
/**********************************************************************/
/************************机器人第三条手臂控制函数************************/
static void ThirdHand_Control(uint16_t FirstParm, uint16_t SecndParm)
{
    ServoPwmData[22] = FirstParm;
    ServoPwmData[23] = SecndParm;
}
/**********************************************************************/
/************************机器人第四条手臂控制函数************************/
static void ForthHand_Control(uint16_t FirstParm, uint16_t SecndParm)
{
    ServoPwmData[24] = FirstParm;
    ServoPwmData[25] = SecndParm;
}
/**********************************************************************/
/************************机器人第五条手臂控制函数************************/
static void FifthHand_Control(uint16_t FirstParm, uint16_t SecndParm)
{
    ServoPwmData[26] = FirstParm;
    ServoPwmData[27] = SecndParm;
}
/**********************************************************************/
/************************机器人第六条手臂控制函数************************/
static void SixthHand_Control(uint16_t FirstParm, uint16_t SecndParm)
{
    ServoPwmData[28] = FirstParm;
    ServoPwmData[29] = SecndParm;
}
/**********************************************************************/
/********************机器人上半球展开函数*******************************/
static void RobotUpBox_Open(void)
{
    FirstHand_Control(UP_BOX_LEG_OPEN, UP_BOX_FET_OPEN);
    SecndHand_Control(UP_BOX_LEG_OPEN, UP_BOX_FET_OPEN);
    ThirdHand_Control(UP_BOX_LEG_OPEN, UP_BOX_FET_OPEN);
    ForthHand_Control(UP_BOX_LEG_OPEN, UP_BOX_FET_OPEN);
    FifthHand_Control(UP_BOX_LEG_OPEN, UP_BOX_FET_OPEN);
    SixthHand_Control(UP_BOX_LEG_OPEN, UP_BOX_FET_OPEN);
}
/*********************************************************************/
/********************机器人上半球闭合函数******************************/
static void RobotUpBox_Close(void)
{
    FirstHand_Control(UP_BOX_LEG_CLOS, UP_BOX_FET_CLOS);
    SecndHand_Control(UP_BOX_LEG_CLOS, UP_BOX_FET_CLOS);
    ThirdHand_Control(UP_BOX_LEG_CLOS, UP_BOX_FET_CLOS);
    ForthHand_Control(UP_BOX_LEG_CLOS, UP_BOX_FET_CLOS);
    FifthHand_Control(UP_BOX_LEG_CLOS, UP_BOX_FET_CLOS);
    SixthHand_Control(UP_BOX_LEG_CLOS, UP_BOX_FET_CLOS);
}
/*********************************************************************/
///********************机器人完整球形闭合函数****************************/
//static void RobotBoxShape_Finsh(void)
//{
//    FirstLeg_Control(BOX_CENTER_1, BOX_CENTER_2, BOX_CENTER_3);
//    SecndLeg_Control(BOX_CENTER_4, BOX_CENTER_5, BOX_CENTER_6);
//    ThirdLeg_Control(BOX_CENTER_7, BOX_CENTER_8, BOX_CENTER_9);
//    ForthLeg_Control(BOX_CENTER_10, BOX_CENTER_11, BOX_CENTER_12);
//    FifthLeg_Control(BOX_CENTER_13, BOX_CENTER_14, BOX_CENTER_15);
//    SixthLeg_Control(BOX_CENTER_16, BOX_CENTER_17, BOX_CENTER_18);

//    FirstHand_Control(BOX_CENTER_19, BOX_CENTER_20);
//    SecndHand_Control(BOX_CENTER_21, BOX_CENTER_22);
//    ThirdHand_Control(BOX_CENTER_23, BOX_CENTER_24);
//    ForthHand_Control(BOX_CENTER_25, BOX_CENTER_26);
//    FifthHand_Control(BOX_CENTER_27, BOX_CENTER_28);
//    SixthHand_Control(BOX_CENTER_29, BOX_CENTER_30);
//}
///*********************************************************************/
/***********************机器人站立函数*********************************/
void RobotHexShape_Stand(void)
{
    static uint8_t      CalcuFlag = 0;

    StartMoveFlag = 0;

    if(!CalcuFlag)
    {
        CalcuFlag = 1;
        MoveCoordinate_Calculate(0, 0);
    }

    FirstLeg_Control(SServoPwmData[0], SServoPwmData[1], SServoPwmData[2]);                                                        //将所有舵机赋值为初始值，机器人站立待命
    SecndLeg_Control(SServoPwmData[3], SServoPwmData[4], SServoPwmData[5]);
    ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7], SServoPwmData[8]);
    ForthLeg_Control(SServoPwmData[9], SServoPwmData[10], SServoPwmData[11]);
    FifthLeg_Control(SServoPwmData[12], SServoPwmData[13], SServoPwmData[14]);
    SixthLeg_Control(SServoPwmData[15], SServoPwmData[16], SServoPwmData[17]);

    RobotUpBox_Open();
}
/*********************************************************************/
/*******************机器人变形成球形形态函数****************************/
static void RobotBoxShape_Transform(uint8_t Flag)
{
    switch(Flag)
    {
        case 1: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1] + ROBOT_LEG_LIFT * 2, SServoPwmData[2] + ROBOT_FET_LIFT * 3);
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13] + ROBOT_LEG_LIFT * 2, SServoPwmData[14] + ROBOT_FET_LIFT * 3);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7] + ROBOT_LEG_LIFT * 2, SServoPwmData[8] + ROBOT_FET_LIFT * 3);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10], SServoPwmData[11]);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4], SServoPwmData[5]);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16], SServoPwmData[17]);
                break;

        case 2: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1] - ROBOT_LEG_LIFT * 1, SServoPwmData[2] + ROBOT_FET_LIFT * 1);
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13] - ROBOT_LEG_LIFT * 1, SServoPwmData[14] + ROBOT_FET_LIFT * 1);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7] - ROBOT_LEG_LIFT * 1, SServoPwmData[8] + ROBOT_FET_LIFT * 1);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10], SServoPwmData[11]);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4], SServoPwmData[5]);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16], SServoPwmData[17]);
                break;

        case 3: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1] - ROBOT_LEG_LIFT * 1, SServoPwmData[2] + ROBOT_FET_LIFT * 1);
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13] - ROBOT_LEG_LIFT * 1, SServoPwmData[14] + ROBOT_FET_LIFT * 1);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7] - ROBOT_LEG_LIFT * 1, SServoPwmData[8] + ROBOT_FET_LIFT * 1);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10] + ROBOT_LEG_LIFT * 2, SServoPwmData[11] + ROBOT_FET_LIFT * 3);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4] + ROBOT_LEG_LIFT * 2, SServoPwmData[5] + ROBOT_FET_LIFT * 3);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16] + ROBOT_LEG_LIFT * 2, SServoPwmData[17] + ROBOT_FET_LIFT * 3);
                break;

        case 4: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1] - ROBOT_LEG_LIFT * 1, SServoPwmData[2] + ROBOT_FET_LIFT * 1);
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13] - ROBOT_LEG_LIFT * 1, SServoPwmData[14] + ROBOT_FET_LIFT * 1);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7] - ROBOT_LEG_LIFT * 1, SServoPwmData[8] + ROBOT_FET_LIFT * 1);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10] - ROBOT_LEG_LIFT * 1, SServoPwmData[11] + ROBOT_FET_LIFT * 1);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4] - ROBOT_LEG_LIFT * 1, SServoPwmData[5] + ROBOT_FET_LIFT * 1);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16] - ROBOT_LEG_LIFT * 1, SServoPwmData[17] + ROBOT_FET_LIFT * 1);
                break;

        case 5: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1] - ROBOT_LEG_LIFT * 0, SServoPwmData[2] + ROBOT_FET_LIFT * 3);
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13] - ROBOT_LEG_LIFT * 0, SServoPwmData[14] + ROBOT_FET_LIFT * 3);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7] - ROBOT_LEG_LIFT * 0, SServoPwmData[8] + ROBOT_FET_LIFT * 3);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10] - ROBOT_LEG_LIFT * 1, SServoPwmData[11] + ROBOT_FET_LIFT * 1);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4] - ROBOT_LEG_LIFT * 1, SServoPwmData[5] + ROBOT_FET_LIFT * 1);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16] - ROBOT_LEG_LIFT * 1, SServoPwmData[17] + ROBOT_FET_LIFT * 1);
                break;

        case 6: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1] - ROBOT_LEG_LIFT * 2, SServoPwmData[2] + ROBOT_FET_LIFT * 1);
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13] - ROBOT_LEG_LIFT * 2, SServoPwmData[14] + ROBOT_FET_LIFT * 1);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7] - ROBOT_LEG_LIFT * 2, SServoPwmData[8] + ROBOT_FET_LIFT * 1);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10] - ROBOT_LEG_LIFT * 1, SServoPwmData[11] + ROBOT_FET_LIFT * 1);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4] - ROBOT_LEG_LIFT * 1, SServoPwmData[5] + ROBOT_FET_LIFT * 1);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16] - ROBOT_LEG_LIFT * 1, SServoPwmData[17] + ROBOT_FET_LIFT * 1);
                break;

        case 7: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1] - ROBOT_LEG_LIFT * 2, SServoPwmData[2] + ROBOT_FET_LIFT * 1);
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13] - ROBOT_LEG_LIFT * 2, SServoPwmData[14] + ROBOT_FET_LIFT * 1);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7] - ROBOT_LEG_LIFT * 2, SServoPwmData[8] + ROBOT_FET_LIFT * 1);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10] - ROBOT_LEG_LIFT * 0, SServoPwmData[11] + ROBOT_FET_LIFT * 3);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4] - ROBOT_LEG_LIFT * 0, SServoPwmData[5] + ROBOT_FET_LIFT * 3);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16] - ROBOT_LEG_LIFT * 0, SServoPwmData[17] + ROBOT_FET_LIFT * 3);
                break;

        case 8: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1] - ROBOT_LEG_LIFT * 2, SServoPwmData[2] + ROBOT_FET_LIFT * 1);
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13] - ROBOT_LEG_LIFT * 2, SServoPwmData[14] + ROBOT_FET_LIFT * 1);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7] - ROBOT_LEG_LIFT * 2, SServoPwmData[8] + ROBOT_FET_LIFT * 1);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10] - ROBOT_LEG_LIFT * 2, SServoPwmData[11] + ROBOT_FET_LIFT * 1);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4] - ROBOT_LEG_LIFT * 2, SServoPwmData[5] + ROBOT_FET_LIFT * 1);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16] - ROBOT_LEG_LIFT * 2, SServoPwmData[17] + ROBOT_FET_LIFT * 1);
                break;

        case 9: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1] - ROBOT_LEG_LIFT * 4, SServoPwmData[2] - ROBOT_FET_LIFT * 0);
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13] - ROBOT_LEG_LIFT * 4, SServoPwmData[14] - ROBOT_FET_LIFT * 0);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7] - ROBOT_LEG_LIFT * 4, SServoPwmData[8] - ROBOT_FET_LIFT * 0);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10] - ROBOT_LEG_LIFT * 2, SServoPwmData[11] + ROBOT_FET_LIFT * 1);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4] - ROBOT_LEG_LIFT * 2, SServoPwmData[5] + ROBOT_FET_LIFT * 1);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16] - ROBOT_LEG_LIFT * 2, SServoPwmData[17] + ROBOT_FET_LIFT * 1);
                break;

        case 10:FirstLeg_Control(BOX_CENTER_1, BOX_CENTER_2, BOX_CENTER_3);
                FifthLeg_Control(BOX_CENTER_13, BOX_CENTER_14, BOX_CENTER_15);
                ThirdLeg_Control(BOX_CENTER_7, BOX_CENTER_8, BOX_CENTER_9);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10] - ROBOT_LEG_LIFT * 2, SServoPwmData[11] + ROBOT_FET_LIFT * 1);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4] - ROBOT_LEG_LIFT * 2, SServoPwmData[5] + ROBOT_FET_LIFT * 1);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16] - ROBOT_LEG_LIFT * 2, SServoPwmData[17] + ROBOT_FET_LIFT * 1);
                break;

        case 11:FirstLeg_Control(BOX_CENTER_1, BOX_CENTER_2, BOX_CENTER_3);
                FifthLeg_Control(BOX_CENTER_13, BOX_CENTER_14, BOX_CENTER_15);
                ThirdLeg_Control(BOX_CENTER_7, BOX_CENTER_8, BOX_CENTER_9);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10] - ROBOT_LEG_LIFT * 4, SServoPwmData[11] - ROBOT_FET_LIFT * 0);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4] - ROBOT_LEG_LIFT * 4, SServoPwmData[5] - ROBOT_FET_LIFT * 0);
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16] - ROBOT_LEG_LIFT * 4, SServoPwmData[17] - ROBOT_FET_LIFT * 0);
                break;

        case 12:FirstLeg_Control(BOX_CENTER_1, BOX_CENTER_2, BOX_CENTER_3);
                FifthLeg_Control(BOX_CENTER_13, BOX_CENTER_14, BOX_CENTER_15);
                ThirdLeg_Control(BOX_CENTER_7, BOX_CENTER_8, BOX_CENTER_9);

                ForthLeg_Control(BOX_CENTER_10, BOX_CENTER_11, BOX_CENTER_12);
                SecndLeg_Control(BOX_CENTER_4, BOX_CENTER_5, BOX_CENTER_6);
                SixthLeg_Control(BOX_CENTER_16, BOX_CENTER_17, BOX_CENTER_18);
                break;

        default:break;
    }

    switch(Flag)
    {
        case 1: RobotUpBox_Close();
                break;

        case 5: FirstHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 1, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 1);
                SecndHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 1, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 1);
                ThirdHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 1, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 1);
                ForthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 1, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 1);
                FifthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 1, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 1);
                SixthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 1, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 1);
                break;

        case 7: FirstHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 2, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 2);
                SecndHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 2, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 2);
                ThirdHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 2, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 2);
                ForthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 2, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 2);
                FifthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 2, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 2);
                SixthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 2, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 2);
                break;

        case 9: FirstHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 3, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 3);
                SecndHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 3, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 3);
                ThirdHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 3, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 3);
                ForthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 3, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 3);
                FifthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 3, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 3);
                SixthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 3, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 3);
                break;

        case 10:FirstHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 4, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 4);
                SecndHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 4, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 4);
                ThirdHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 4, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 4);
                ForthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 4, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 4);
                FifthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 4, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 4);
                SixthHand_Control(UP_BOX_LEG_CLOS - ROBOT_LEG_LIFT * 4, UP_BOX_FET_CLOS - ROBOT_FET_LIFT * 4);
                break;

        case 12:FirstHand_Control(BOX_CENTER_19, BOX_CENTER_20);
                SecndHand_Control(BOX_CENTER_21, BOX_CENTER_22);
                ThirdHand_Control(BOX_CENTER_23, BOX_CENTER_24);
                ForthHand_Control(BOX_CENTER_25, BOX_CENTER_26);
                FifthHand_Control(BOX_CENTER_27, BOX_CENTER_28);
                SixthHand_Control(BOX_CENTER_29, BOX_CENTER_30);
                break;

        default:break;
    }
}
/*********************************************************************/
/*******************机器人变形成六足形态函数****************************/
static void RobotHexShape_Transform(uint8_t Flag)
{
    static uint8_t          CountFlag = 0;

    CountFlag++;

    if(RollFinshFlag)
    {
        switch(CountFlag)
        {

        }
    }
    else
    {
        switch(Flag)
        {

        }
        
        switch(Flag)
        {

        }
    }
}
/*********************************************************************/
/************************机器人前进函数********************************/
static void RobotHexShape_Forward(void)
{
    static uint8_t      Flag = 0;                                                                                                   //Flag标记用于控制机器人动作组的执行顺序

    Flag++;

    if(Flag > 4)
    {
        Flag = 1;
    }

    if(!StartMoveFlag)
    {
        StartMoveFlag = 1;
        ExpectedAngle = yaw;

        RobotUpBox_Open();
    }

    switch(Flag)
    {
        case 1: AdjustStepLen_Calculate(ExpectedAngle, 0);

                FirstLeg_Control(FServoPwmData[0], SServoPwmData[1] + ROBOT_LEG_LIFT, SServoPwmData[2] + ROBOT_FET_LIFT);          //第一步：第一组腿抬脚前移
                FifthLeg_Control(FServoPwmData[12], SServoPwmData[13] + ROBOT_LEG_LIFT, SServoPwmData[14] + ROBOT_FET_LIFT);
                ThirdLeg_Control(FServoPwmData[6], SServoPwmData[7] + ROBOT_LEG_LIFT, SServoPwmData[8] + ROBOT_FET_LIFT);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10], SServoPwmData[11]);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4], SServoPwmData[5]);                                            //同时，第二组腿向后滑动
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16], SServoPwmData[17]);
                break;

        case 2: FirstLeg_Control(FServoPwmData[0], FServoPwmData[1], FServoPwmData[2]);                                             //第二步：第一组腿落地
                FifthLeg_Control(FServoPwmData[12], FServoPwmData[13], FServoPwmData[14]);
                ThirdLeg_Control(FServoPwmData[6], FServoPwmData[7], FServoPwmData[8]);

                ForthLeg_Control(FServoPwmData[27], FServoPwmData[28], FServoPwmData[29]);
                SecndLeg_Control(FServoPwmData[21], FServoPwmData[22], FServoPwmData[23]);                                         //同时，第二组腿继续向后滑动
                SixthLeg_Control(FServoPwmData[33], FServoPwmData[34], FServoPwmData[35]);
                break;

        case 3: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1], SServoPwmData[2]);                                             //第三步：第一组腿向后滑动
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13], SServoPwmData[14]);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7], SServoPwmData[8]);
        
                ForthLeg_Control(FServoPwmData[9], SServoPwmData[10] + ROBOT_LEG_LIFT, SServoPwmData[11] + ROBOT_FET_LIFT);
                SecndLeg_Control(FServoPwmData[3], SServoPwmData[4] + ROBOT_LEG_LIFT, SServoPwmData[5] + ROBOT_FET_LIFT);           //同时，第二组腿抬腿
                SixthLeg_Control(FServoPwmData[15], SServoPwmData[16] + ROBOT_LEG_LIFT, SServoPwmData[17] + ROBOT_FET_LIFT);
                break; 

        case 4: FirstLeg_Control(FServoPwmData[18], FServoPwmData[19], FServoPwmData[20]);                                          //第四步：第一组腿继续向后滑动
                FifthLeg_Control(FServoPwmData[30], FServoPwmData[31], FServoPwmData[32]);
                ThirdLeg_Control(FServoPwmData[24], FServoPwmData[25], FServoPwmData[26]);
                
                ForthLeg_Control(FServoPwmData[9], FServoPwmData[10], FServoPwmData[11]);
                SecndLeg_Control(FServoPwmData[3], FServoPwmData[4], FServoPwmData[5]);                                            //同时，第二组腿落地
                SixthLeg_Control(FServoPwmData[15], FServoPwmData[16], FServoPwmData[17]);
                break;

        default:break;
    }
}
/*********************************************************************/
/*************************机器人后退函数*******************************/
static void RobotHexShape_Backwrd(void)
{
    static uint8_t          Flag = 0;                                                                                                        //Flag标记用于控制机器人动作组的执行顺序

    Flag++;

    if(Flag > 4)
    {
        Flag = 1;
    }

    if(! StartMoveFlag)
    {
        StartMoveFlag = 1;
        ExpectedAngle = yaw;
        
        RobotUpBox_Open();
    }

    switch(Flag)
    {
        case 1: AdjustStepLen_Calculate(ExpectedAngle, 1);
        
                FirstLeg_Control(BServoPwmData[0], SServoPwmData[1] + ROBOT_LEG_LIFT, SServoPwmData[2] + ROBOT_FET_LIFT);           //第一步：第一组腿抬脚前移
                FifthLeg_Control(BServoPwmData[12], SServoPwmData[13] + ROBOT_LEG_LIFT, SServoPwmData[14] + ROBOT_FET_LIFT);
                ThirdLeg_Control(BServoPwmData[6], SServoPwmData[7] + ROBOT_LEG_LIFT, SServoPwmData[8] + ROBOT_FET_LIFT);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10], SServoPwmData[11]);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4], SServoPwmData[5]);                                             //同时，第二组腿向后滑动
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16], SServoPwmData[17]);
                break;

        case 2: FirstLeg_Control(BServoPwmData[0], BServoPwmData[1], BServoPwmData[2]);                                             //第二步：第一组腿落地
                FifthLeg_Control(BServoPwmData[12], BServoPwmData[13], BServoPwmData[14]);
                ThirdLeg_Control(BServoPwmData[6], BServoPwmData[7], BServoPwmData[8]);

                ForthLeg_Control(BServoPwmData[27], BServoPwmData[28], BServoPwmData[29]);
                SecndLeg_Control(BServoPwmData[21], BServoPwmData[22], BServoPwmData[23]);                                          //同时，第二组腿继续向后滑动
                SixthLeg_Control(BServoPwmData[33], BServoPwmData[34], BServoPwmData[35]);
                break;

        case 3: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1], SServoPwmData[2]);                                             //第三步：第一组腿向后滑动
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13], SServoPwmData[14]);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7], SServoPwmData[8]);
        
                ForthLeg_Control(BServoPwmData[9], SServoPwmData[10] + ROBOT_LEG_LIFT, SServoPwmData[11] + ROBOT_FET_LIFT);
                SecndLeg_Control(BServoPwmData[3], SServoPwmData[4] + ROBOT_LEG_LIFT, SServoPwmData[5] + ROBOT_FET_LIFT);           //同时，第二组腿抬腿
                SixthLeg_Control(BServoPwmData[15], SServoPwmData[16] + ROBOT_LEG_LIFT, SServoPwmData[17] + ROBOT_FET_LIFT);
                break; 

        case 4: FirstLeg_Control(BServoPwmData[18], BServoPwmData[19], BServoPwmData[20]);                                          //第四步：第一组腿继续向后滑动
                FifthLeg_Control(BServoPwmData[30], BServoPwmData[31], BServoPwmData[32]);
                ThirdLeg_Control(BServoPwmData[24], BServoPwmData[25], BServoPwmData[26]);
                
                ForthLeg_Control(BServoPwmData[9], BServoPwmData[10], BServoPwmData[11]);                                           //同时，第二组腿落地
                SecndLeg_Control(BServoPwmData[3], BServoPwmData[4], BServoPwmData[5]);
                SixthLeg_Control(BServoPwmData[15], BServoPwmData[16], BServoPwmData[17]);
                break;

        default:break;
    }
}
/*********************************************************************/
/************************机器人逆时针转圈函数**************************/
static void RobotHexShape_AtiClck(void)
{
    static uint8_t          Flag = 0;

    Flag++;

    StartMoveFlag = 0;

    if(Flag > 4)
    {
        Flag = 1;
        
        RobotUpBox_Open();
    }

    switch(Flag)
    {
        case 1: CircleCoordinate_Calculate(10);

                FirstLeg_Control(CServoPwmData[18], SServoPwmData[1] + ROBOT_LEG_LIFT, SServoPwmData[2] + ROBOT_FET_LIFT);         //第一步：第一组腿抬脚前移
                FifthLeg_Control(CServoPwmData[30], SServoPwmData[13] + ROBOT_LEG_LIFT, SServoPwmData[14] + ROBOT_FET_LIFT);
                ThirdLeg_Control(CServoPwmData[24], SServoPwmData[7] + ROBOT_LEG_LIFT, SServoPwmData[8] + ROBOT_FET_LIFT);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10], SServoPwmData[11]);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4], SServoPwmData[5]);                                            //同时，第二组腿向后滑动
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16], SServoPwmData[17]);
                break;

        case 2: FirstLeg_Control(CServoPwmData[18], CServoPwmData[19], CServoPwmData[20]);                                          //第二步：第一组腿落地
                FifthLeg_Control(CServoPwmData[30], CServoPwmData[31], CServoPwmData[32]);
                ThirdLeg_Control(CServoPwmData[24], CServoPwmData[25], CServoPwmData[26]);

                ForthLeg_Control(CServoPwmData[9], CServoPwmData[10], CServoPwmData[11]);
                SecndLeg_Control(CServoPwmData[3], CServoPwmData[4], CServoPwmData[5]);                                            //同时，第二组腿落地
                SixthLeg_Control(CServoPwmData[15], CServoPwmData[16], CServoPwmData[17]);
                break;

        case 3: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1], SServoPwmData[2]);                                             //第三步：第一组腿向后滑动
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13], SServoPwmData[14]);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7], SServoPwmData[8]);

                ForthLeg_Control(CServoPwmData[27], SServoPwmData[10] + ROBOT_LEG_LIFT, SServoPwmData[11] + ROBOT_FET_LIFT);
                SecndLeg_Control(CServoPwmData[21], SServoPwmData[4] + ROBOT_LEG_LIFT, SServoPwmData[5] + ROBOT_FET_LIFT);        //同时，第二组腿抬腿
                SixthLeg_Control(CServoPwmData[33], SServoPwmData[16] + ROBOT_LEG_LIFT, SServoPwmData[17] + ROBOT_FET_LIFT);
                break; 

        case 4: FirstLeg_Control(CServoPwmData[0], CServoPwmData[1], CServoPwmData[2]);                                             //第四步：第一组腿继续向后滑动
                FifthLeg_Control(CServoPwmData[12], CServoPwmData[13], CServoPwmData[14]);
                ThirdLeg_Control(CServoPwmData[6], CServoPwmData[7], CServoPwmData[8]);

                ForthLeg_Control(CServoPwmData[27], CServoPwmData[28], CServoPwmData[29]);
                SecndLeg_Control(CServoPwmData[21], CServoPwmData[22], CServoPwmData[23]);                                         //同时，第二组腿继续向后滑动
                SixthLeg_Control(CServoPwmData[33], CServoPwmData[34], CServoPwmData[35]);
                break;

        default:break;
    }
}
/*********************************************************************/
/************************机器人顺时针转圈函数**************************/
static void RobotHexShape_ClckWis(void)
{
    static uint8_t          Flag = 0;                                                                                                        //Flag标记用于控制机器人动作组的执行顺序

    Flag++;

    StartMoveFlag = 0;

    if(Flag > 4)
    {
        Flag = 1;
        
        RobotUpBox_Open();
    }

    switch(Flag)
    {
        case 1: CircleCoordinate_Calculate(10);

                FirstLeg_Control(CServoPwmData[0], SServoPwmData[1] + ROBOT_LEG_LIFT, SServoPwmData[2] + ROBOT_FET_LIFT);           //第一步：第一组腿抬脚前移
                FifthLeg_Control(CServoPwmData[12], SServoPwmData[13] + ROBOT_LEG_LIFT, SServoPwmData[14] + ROBOT_FET_LIFT);
                ThirdLeg_Control(CServoPwmData[6], SServoPwmData[7] + ROBOT_LEG_LIFT, SServoPwmData[8] + ROBOT_FET_LIFT);

                ForthLeg_Control(SServoPwmData[9], SServoPwmData[10], SServoPwmData[11]);
                SecndLeg_Control(SServoPwmData[3], SServoPwmData[4], SServoPwmData[5]);                                             //同时，第二组腿向后滑动
                SixthLeg_Control(SServoPwmData[15], SServoPwmData[16], SServoPwmData[17]);
                break;

        case 2: FirstLeg_Control(CServoPwmData[0], CServoPwmData[1], CServoPwmData[2]);                                             //第二步：第一组腿落地
                FifthLeg_Control(CServoPwmData[12], CServoPwmData[13], CServoPwmData[14]);
                ThirdLeg_Control(CServoPwmData[6], CServoPwmData[7], CServoPwmData[8]);

                ForthLeg_Control(CServoPwmData[27], CServoPwmData[28], CServoPwmData[29]);
                SecndLeg_Control(CServoPwmData[21], CServoPwmData[22], CServoPwmData[23]);                                          //同时，第二组腿继续向后滑动
                SixthLeg_Control(CServoPwmData[33], CServoPwmData[34], CServoPwmData[35]);
                break;

        case 3: FirstLeg_Control(SServoPwmData[0], SServoPwmData[1], SServoPwmData[2]);                                             //第三步：第一组腿向后滑动
                FifthLeg_Control(SServoPwmData[12], SServoPwmData[13], SServoPwmData[14]);
                ThirdLeg_Control(SServoPwmData[6], SServoPwmData[7], SServoPwmData[8]);

                ForthLeg_Control(CServoPwmData[9], SServoPwmData[10] + ROBOT_LEG_LIFT, SServoPwmData[11] + ROBOT_FET_LIFT);
                SecndLeg_Control(CServoPwmData[3], SServoPwmData[4] + ROBOT_LEG_LIFT, SServoPwmData[5] + ROBOT_FET_LIFT);           //同时，第二组腿抬腿
                SixthLeg_Control(CServoPwmData[15], SServoPwmData[16] + ROBOT_LEG_LIFT, SServoPwmData[17] + ROBOT_FET_LIFT);
                break; 

        case 4: FirstLeg_Control(CServoPwmData[18], CServoPwmData[19], CServoPwmData[20]);                                          //第四步：第一组腿继续向后滑动
                FifthLeg_Control(CServoPwmData[30], CServoPwmData[31], CServoPwmData[32]);
                ThirdLeg_Control(CServoPwmData[24], CServoPwmData[25], CServoPwmData[26]);

                ForthLeg_Control(CServoPwmData[9], CServoPwmData[10], CServoPwmData[11]);
                SecndLeg_Control(CServoPwmData[3], CServoPwmData[4], CServoPwmData[5]);                                             //同时，第二组腿落地
                SixthLeg_Control(CServoPwmData[15], CServoPwmData[16], CServoPwmData[17]);
                break;

        default:break;
    }
}
/********************************************************************/
/************************机器人左前进函数*****************************/
static void RobotHexShape_LForward(void)
{
    static uint8_t          Flag = 0;

    Flag++;

    if(Flag > 4)
    {
        Flag = 1;
    }

    if(!StartMoveFlag)
    {
        StartMoveFlag = 1;
        ExpectedAngle = yaw;

        RobotUpBox_Open();
    }

    switch(Flag)
    {
        case 1: AdjustLStepLen_Calculate(ExpectedAngle, 0);

                ForthLeg_Control(LFServoPwmData[9], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                SixthLeg_Control(LFServoPwmData[15], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                SecndLeg_Control(LFServoPwmData[3], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);

                FifthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                FirstLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                ThirdLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                break;

        case 2: ForthLeg_Control(LFServoPwmData[9], LFServoPwmData[10], LFServoPwmData[11]);
                SixthLeg_Control(LFServoPwmData[15], LFServoPwmData[16], LFServoPwmData[17]);
                SecndLeg_Control(LFServoPwmData[3], LFServoPwmData[4], LFServoPwmData[5]);

                FifthLeg_Control(LFServoPwmData[30], LFServoPwmData[31], LFServoPwmData[32]);
                FirstLeg_Control(LFServoPwmData[18], LFServoPwmData[19], LFServoPwmData[20]);
                ThirdLeg_Control(LFServoPwmData[24], LFServoPwmData[25], LFServoPwmData[26]);
                break;

        case 3: ForthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                SixthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                SecndLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);

                FifthLeg_Control(LFServoPwmData[12], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                FirstLeg_Control(LFServoPwmData[0], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                ThirdLeg_Control(LFServoPwmData[6], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                break; 

        case 4: ForthLeg_Control(LFServoPwmData[27], LFServoPwmData[28], LFServoPwmData[29]);
                SixthLeg_Control(LFServoPwmData[33], LFServoPwmData[34], LFServoPwmData[35]);
                SecndLeg_Control(LFServoPwmData[21], LFServoPwmData[22], LFServoPwmData[23]);

                FifthLeg_Control(LFServoPwmData[12], LFServoPwmData[13], LFServoPwmData[14]);
                FirstLeg_Control(LFServoPwmData[0], LFServoPwmData[1], LFServoPwmData[2]);
                ThirdLeg_Control(LFServoPwmData[6], LFServoPwmData[7], LFServoPwmData[8]);
                break;

        default:break;
    }
}
/********************************************************************/
/************************机器人左后退函数*****************************/
static void RobotHexShape_LBackward(void)
{
    static uint8_t          Flag = 0;

    Flag++;

    if(Flag > 4)
    {
        Flag = 1;
    }

    if(!StartMoveFlag)
    {
        StartMoveFlag = 1;
        ExpectedAngle = yaw;

        RobotUpBox_Open();
    }

    switch(Flag)
    {
        case 1: AdjustRStepLen_Calculate(ExpectedAngle, 1);

                SecndLeg_Control(RBServoPwmData[3], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                ForthLeg_Control(RBServoPwmData[9], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                SixthLeg_Control(RBServoPwmData[15], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);

                FirstLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                ThirdLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                FifthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                break;

        case 2: SecndLeg_Control(RBServoPwmData[3], RBServoPwmData[4], RBServoPwmData[5]);
                ForthLeg_Control(RBServoPwmData[9], RBServoPwmData[10], RBServoPwmData[11]);
                SixthLeg_Control(RBServoPwmData[15], RBServoPwmData[16], RBServoPwmData[17]);

                FirstLeg_Control(RBServoPwmData[18], RBServoPwmData[19], RBServoPwmData[20]);
                ThirdLeg_Control(RBServoPwmData[24], RBServoPwmData[25], RBServoPwmData[26]);
                FifthLeg_Control(RBServoPwmData[30], RBServoPwmData[31], RBServoPwmData[32]);
                break;

        case 3: SecndLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                ForthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                SixthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);

                FirstLeg_Control(RBServoPwmData[0], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                ThirdLeg_Control(RBServoPwmData[6], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                FifthLeg_Control(RBServoPwmData[12], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                break; 

        case 4: SecndLeg_Control(RBServoPwmData[21], RBServoPwmData[22], RBServoPwmData[23]);
                ForthLeg_Control(RBServoPwmData[27], RBServoPwmData[28], RBServoPwmData[29]);
                SixthLeg_Control(RBServoPwmData[33], RBServoPwmData[34], RBServoPwmData[35]);

                FirstLeg_Control(RBServoPwmData[0], RBServoPwmData[1], RBServoPwmData[2]);
                ThirdLeg_Control(RBServoPwmData[6], RBServoPwmData[7], RBServoPwmData[8]);
                FifthLeg_Control(RBServoPwmData[12], RBServoPwmData[13], RBServoPwmData[14]);
                break;

        default:break;
    }
}
/********************************************************************/
/************************机器人右前进函数*****************************/
static void RobotHexShape_RForward(void)
{
    static uint8_t          Flag = 0;

    Flag++;

    if(Flag > 4)
    {
        Flag = 1;
    }

    if(!StartMoveFlag)
    {
        StartMoveFlag = 1;
        ExpectedAngle = yaw;

        RobotUpBox_Open();
    }

    switch(Flag)
    {
        case 1: AdjustRStepLen_Calculate(ExpectedAngle, 0);

                SecndLeg_Control(RFServoPwmData[3], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                ForthLeg_Control(RFServoPwmData[9], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                SixthLeg_Control(RFServoPwmData[15], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);

                FirstLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                ThirdLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                FifthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                break;

        case 2: SecndLeg_Control(RFServoPwmData[3], RFServoPwmData[4], RFServoPwmData[5]);
                ForthLeg_Control(RFServoPwmData[9], RFServoPwmData[10], RFServoPwmData[11]);
                SixthLeg_Control(RFServoPwmData[15], RFServoPwmData[16], RFServoPwmData[17]);

                FirstLeg_Control(RFServoPwmData[18], RFServoPwmData[19], RFServoPwmData[20]);
                ThirdLeg_Control(RFServoPwmData[24], RFServoPwmData[25], RFServoPwmData[26]);
                FifthLeg_Control(RFServoPwmData[30], RFServoPwmData[31], RFServoPwmData[32]);
                break;

        case 3: SecndLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                ForthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                SixthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);

                FirstLeg_Control(RFServoPwmData[0], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                ThirdLeg_Control(RFServoPwmData[6], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                FifthLeg_Control(RFServoPwmData[12], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                break; 

        case 4: SecndLeg_Control(RFServoPwmData[21], RFServoPwmData[22], RFServoPwmData[23]);
                ForthLeg_Control(RFServoPwmData[27], RFServoPwmData[28], RFServoPwmData[29]);
                SixthLeg_Control(RFServoPwmData[33], RFServoPwmData[34], RFServoPwmData[35]);

                FirstLeg_Control(RFServoPwmData[0], RFServoPwmData[1], RFServoPwmData[2]);
                ThirdLeg_Control(RFServoPwmData[6], RFServoPwmData[7], RFServoPwmData[8]);
                FifthLeg_Control(RFServoPwmData[12], RFServoPwmData[13], RFServoPwmData[14]);
                break;

        default:break;
    }
}
/********************************************************************/
/************************机器人右后退函数*****************************/
static void RobotHexShape_RBackward(void)
{
    static uint8_t          Flag = 0;

    Flag++;

    if(Flag > 4)
    {
        Flag = 1;
    }

    if(!StartMoveFlag)
    {
        StartMoveFlag = 1;
        ExpectedAngle = yaw;

        RobotUpBox_Open();
    }

    switch(Flag)
    {
        case 1: AdjustLStepLen_Calculate(ExpectedAngle, 1);

                ForthLeg_Control(LBServoPwmData[9], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                SixthLeg_Control(LBServoPwmData[15], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                SecndLeg_Control(LBServoPwmData[3], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);

                FifthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                FirstLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                ThirdLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                break;

        case 2: ForthLeg_Control(LBServoPwmData[9], LBServoPwmData[10], LBServoPwmData[11]);
                SixthLeg_Control(LBServoPwmData[15], LBServoPwmData[16], LBServoPwmData[17]);
                SecndLeg_Control(LBServoPwmData[3], LBServoPwmData[4], LBServoPwmData[5]);

                FifthLeg_Control(LBServoPwmData[30], LBServoPwmData[31], LBServoPwmData[32]);
                FirstLeg_Control(LBServoPwmData[18], LBServoPwmData[19], LBServoPwmData[20]);
                ThirdLeg_Control(LBServoPwmData[24], LBServoPwmData[25], LBServoPwmData[26]);
                break;

        case 3: ForthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                SixthLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);
                SecndLeg_Control(SERVO_PWM_CENTER, SERVO_PWM_CENTER, SERVO_PWM_CENTER);

                FifthLeg_Control(LBServoPwmData[12], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                FirstLeg_Control(LBServoPwmData[0], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                ThirdLeg_Control(LBServoPwmData[6], SERVO_PWM_CENTER + ROBOT_LEG_LIFT, SERVO_PWM_CENTER + ROBOT_FET_LIFT);
                break; 

        case 4: ForthLeg_Control(LBServoPwmData[27], LBServoPwmData[28], LBServoPwmData[29]);
                SixthLeg_Control(LBServoPwmData[33], LBServoPwmData[34], LBServoPwmData[35]);
                SecndLeg_Control(LBServoPwmData[21], LBServoPwmData[22], LBServoPwmData[23]);

                FifthLeg_Control(LBServoPwmData[12], LBServoPwmData[13], LBServoPwmData[14]);
                FirstLeg_Control(LBServoPwmData[0], LBServoPwmData[1], LBServoPwmData[2]);
                ThirdLeg_Control(LBServoPwmData[6], LBServoPwmData[7], LBServoPwmData[8]);
                break;

        default:break;
    }
}
/********************************************************************/
/************************机器人前滚函数*******************************/
static void RobotBoxShape_ForwdRoll(void)
{
    static uint8_t          CountFlag = 0;                                                                          //Flag标记用于控制机器人动作组的执行顺序

    CountFlag++;

    RollFinshFlag = 1;

    if(!RollReadyFlag)
    {
        switch(CountFlag)
        {
            case 1: FirstLeg_Control(BOX_CENTER_1, BOX_CENTER_2 + ROBOT_LEG_LIFT * 1, BOX_CENTER_3 + ROBOT_FET_LIFT * 1);
                    SecndLeg_Control(BOX_CENTER_4, BOX_CENTER_5 + ROBOT_LEG_LIFT * 1, BOX_CENTER_6 + ROBOT_FET_LIFT * 1);
                    ThirdLeg_Control(BOX_CENTER_7, BOX_CENTER_8 + ROBOT_LEG_LIFT * 1, BOX_CENTER_9 + ROBOT_FET_LIFT * 1);
                    ForthLeg_Control(BOX_CENTER_10, BOX_CENTER_11 + ROBOT_LEG_LIFT * 1, BOX_CENTER_12 + ROBOT_FET_LIFT * 1);
                    FifthLeg_Control(BOX_CENTER_13, BOX_CENTER_14 + ROBOT_LEG_LIFT * 1, BOX_CENTER_15 + ROBOT_FET_LIFT * 1);
                    SixthLeg_Control(BOX_CENTER_16, BOX_CENTER_17 + ROBOT_LEG_LIFT * 1, BOX_CENTER_18 + ROBOT_FET_LIFT * 1);

                    FirstHand_Control(BOX_CENTER_19 + ROBOT_LEG_LIFT * 1, BOX_CENTER_20 + ROBOT_FET_LIFT * 1);
                    SecndHand_Control(BOX_CENTER_21 + ROBOT_LEG_LIFT * 1, BOX_CENTER_22 + ROBOT_FET_LIFT * 1);
                    ThirdHand_Control(BOX_CENTER_23 + ROBOT_LEG_LIFT * 1, BOX_CENTER_24 + ROBOT_FET_LIFT * 1);
                    ForthHand_Control(BOX_CENTER_25 + ROBOT_LEG_LIFT * 1, BOX_CENTER_26 + ROBOT_FET_LIFT * 1);
                    FifthHand_Control(BOX_CENTER_27 + ROBOT_LEG_LIFT * 1, BOX_CENTER_28 + ROBOT_FET_LIFT * 1);
                    SixthHand_Control(BOX_CENTER_29 + ROBOT_LEG_LIFT * 1, BOX_CENTER_30 + ROBOT_FET_LIFT * 1);
                    break;

            case 2: FirstLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    SecndLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    ThirdLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    ForthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    FifthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    SixthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);

                    FirstHand_Control(BOX_CENTER_19 + ROBOT_LEG_LIFT * 2, BOX_CENTER_20 + ROBOT_FET_LIFT * 2);
                    SecndHand_Control(BOX_CENTER_21 + ROBOT_LEG_LIFT * 2, BOX_CENTER_22 + ROBOT_FET_LIFT * 2);
                    ThirdHand_Control(BOX_CENTER_23 + ROBOT_LEG_LIFT * 2, BOX_CENTER_24 + ROBOT_FET_LIFT * 2);
                    ForthHand_Control(BOX_CENTER_25 + ROBOT_LEG_LIFT * 2, BOX_CENTER_26 + ROBOT_FET_LIFT * 2);
                    FifthHand_Control(BOX_CENTER_27 + ROBOT_LEG_LIFT * 2, BOX_CENTER_28 + ROBOT_FET_LIFT * 2);
                    SixthHand_Control(BOX_CENTER_29 + ROBOT_LEG_LIFT * 2, BOX_CENTER_30 + ROBOT_FET_LIFT * 2);
                    break;

            case 3: FirstHand_Control(BOX_CENTER_19 + ROBOT_LEG_LIFT * 3, BOX_CENTER_20 + ROBOT_FET_LIFT * 4);
                    SecndHand_Control(BOX_CENTER_21 + ROBOT_LEG_LIFT * 3, BOX_CENTER_22 + ROBOT_FET_LIFT * 4);
                    ThirdHand_Control(BOX_CENTER_23 + ROBOT_LEG_LIFT * 3, BOX_CENTER_24 + ROBOT_FET_LIFT * 4);
                    ForthHand_Control(BOX_CENTER_25 + ROBOT_LEG_LIFT * 3, BOX_CENTER_26 + ROBOT_FET_LIFT * 4);
                    FifthHand_Control(BOX_CENTER_27 + ROBOT_LEG_LIFT * 3, BOX_CENTER_28 + ROBOT_FET_LIFT * 4);
                    SixthHand_Control(BOX_CENTER_29 + ROBOT_LEG_LIFT * 3, BOX_CENTER_30 + ROBOT_FET_LIFT * 4);
                    break;

            case 4: FirstHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    SecndHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    ThirdHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    ForthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    FifthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    SixthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    break;

            case 5: FirstHand_Control(BOX_CENTER_19 + ROBOT_LEG_LIFT * 3, BOX_CENTER_20 + ROBOT_FET_LIFT * 4);
                    SecndHand_Control(BOX_CENTER_21 + ROBOT_LEG_LIFT * 3, BOX_CENTER_22 + ROBOT_FET_LIFT * 4);
                    ThirdHand_Control(BOX_CENTER_23 + ROBOT_LEG_LIFT * 3, BOX_CENTER_24 + ROBOT_FET_LIFT * 4);
                    ForthHand_Control(BOX_CENTER_25 + ROBOT_LEG_LIFT * 3, BOX_CENTER_26 + ROBOT_FET_LIFT * 4);
                    FifthHand_Control(BOX_CENTER_27 + ROBOT_LEG_LIFT * 3, BOX_CENTER_28 + ROBOT_FET_LIFT * 4);
                    SixthHand_Control(BOX_CENTER_29 + ROBOT_LEG_LIFT * 3, BOX_CENTER_30 + ROBOT_FET_LIFT * 4);
                    break;

            case 6: FirstHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    SecndHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    ThirdHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    ForthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    FifthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    SixthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);

                    CountFlag = 0;
                    RollReadyFlag = 1;

                    ServoRunTime_Set(RobotRollSpeed);
                    break;
        }
    }
    else
    {
        switch(CountFlag)
        {
            case 1: FirstLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    SecndLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    ThirdLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    ForthLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    FifthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    SixthLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);

                    FirstHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    SecndHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    ThirdHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    ForthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    FifthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    SixthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    break;


            case 2: FirstLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    SecndLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    ThirdLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    ForthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    FifthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    SixthLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);

                    FirstHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    SecndHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    ThirdHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    ForthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    FifthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    SixthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    break;

            case 3: FirstLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    SecndLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    ThirdLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    ForthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    FifthLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    SixthLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);

                    FirstHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    SecndHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    ThirdHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    ForthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    FifthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    SixthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    break;

            case 4: FirstLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    SecndLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    ThirdLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    ForthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    FifthLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    SixthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);

                    FirstHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    SecndHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    ThirdHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    ForthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    FifthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    SixthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    break;

            case 5: FirstLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    SecndLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    ThirdLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    ForthLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    FifthLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    SixthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);

                    FirstHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    SecndHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    ThirdHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    ForthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    FifthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    SixthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    break;

            case 6: FirstLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    SecndLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    ThirdLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    ForthLeg_Control(DN_BOX_ARM_ROLL, DN_BOX_LEG_ROLL, DN_BOX_FET_ROLL);
                    FifthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);
                    SixthLeg_Control(DN_BOX_ARM_REDY, DN_BOX_LEG_REDY, DN_BOX_FET_REDY);

                    FirstHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    SecndHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    ThirdHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    ForthHand_Control(UP_BOX_LEG_ROLL, UP_BOX_FET_ROLL);
                    FifthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);
                    SixthHand_Control(UP_BOX_LEG_REDY, UP_BOX_FET_REDY);

                    CountFlag = 0;
                    break;
        }
    }
}
/********************************************************************/
/************************机器人遥控函数*******************************/
void RobotInstruct_Control(void)
{
    static uint8_t          TransfrmFlag = 1;
    static uint8_t          RollLockFlag = 0;
    static uint8_t          TransformHexFinsh = 1;
    static uint8_t          TransformBoxFinsh = 0;
    static uint8_t          TransformBoxOrder = 0;
    static uint8_t          TransformHexOrder = 0;
    static uint8_t          ForwardSpeechFlag = 0;
    static uint8_t          BackwrdSpeechFlag = 0;
    static uint8_t          ClckWisSpeechFlag = 0;
    static uint8_t          AticlckSpeechFlag = 0;
    static uint8_t          TrasfrmSpeechFlag = 0;
    static uint8_t          FrdRollSpeechFlag = 0;
    static uint8_t          BakRollSpeechFlag = 0;


    switch(SpeechReturnData)
    {
        case 2:     if(TransfrmFlag)
                    {
                        RobotHexShape_Forward();
                    }
                    break;

        case 3:     if(TransfrmFlag)
                    {
                        RobotHexShape_Backwrd();
                    }
                    break;

        case 5:     if(TransfrmFlag)
                    {
                        RobotHexShape_AtiClck();
                    }
                    break;

        case 6:     if(TransfrmFlag)
                    {
                        RobotHexShape_ClckWis();
                    }
                    break;

        default:    break;
    }

    switch(RxBuffer[0])
    {
        case 'W':   if(TransfrmFlag)
                    {
                        if(!ForwardSpeechFlag)
                        {
                            ForwardSpeechFlag = 1;
                            ForwardSpeech_Play();

                            ServoRunTime_Set(RobotCrepSpeed);
                        }

                        RobotHexShape_Forward();
                    }
                    break;

        case 'S':   if(TransfrmFlag)
                    {
                        if(!BackwrdSpeechFlag)
                        {
                            BackwrdSpeechFlag = 1;
                            BackwrdSpeech_Play();
                            ServoRunTime_Set(RobotCrepSpeed);
                        }

                        RobotHexShape_Backwrd();
                    }
                    break;

        case 'A':   if(TransfrmFlag)
                    {
                        if(!AticlckSpeechFlag)
                        {
                            AticlckSpeechFlag = 1;
                            AtiClckSpeech_Play();

                            ServoRunTime_Set(RobotCrepSpeed);
                        }

                        RobotHexShape_AtiClck();
                    }
                    break;

        case 'D':   if(TransfrmFlag)
                    {
                        if(!ClckWisSpeechFlag)
                        {
                            ClckWisSpeechFlag = 1;
                            ClckWisSpeech_Play();

                            ServoRunTime_Set(RobotCrepSpeed);
                        }

                        RobotHexShape_ClckWis();
                    }
                    break;

        case 'Y':   if(TransformHexFinsh)
                    {
                        TransfrmFlag = 0;
                        RollReadyFlag = 0;

                        if(!TrasfrmSpeechFlag)
                        {
                            TrasfrmSpeechFlag = 1;
                            TrasfrmSpeech_Play();

                            ServoRunTime_Set(RobotTsfmSpeed);
                        }

                        if(TransformBoxOrder < TRANSFORM_STEP)
                        {
                            RollLockFlag = 0;
                            TransformBoxOrder++;
                            TransformBoxFinsh = 0;
                        }
                        else
                        {
                            RollLockFlag = 1;
                            TransformBoxFinsh = 1;
                            TransformHexOrder = 0;
                        }

                        RobotBoxShape_Transform(TransformBoxOrder);
                    }
                    break;

        case 'G':   if(TransformBoxFinsh)
                    {
                        RollLockFlag = 0; 
                        
                        if(!RollFinshFlag)
                        {
                            if(TransformHexOrder < TRANSFORM_STEP)
                            {
                                TransformHexOrder++;
                                TransformHexFinsh = 0;
                            }
                            else
                            {
                                TransfrmFlag = 1;
                                TransformBoxOrder = 0;
                                TransformHexFinsh = 1;
                            }
                            RobotHexShape_Transform(TransformHexOrder);
                        }
                        else
                        {
                            RobotHexShape_Transform(0);
                        }
                    }
                    break;

        case 'B':   if(RollLockFlag)
                    {
                        if(!FrdRollSpeechFlag)
                        {
                            FrdRollSpeechFlag = 1;
                            FrdRollSpeech_Play();
                        }

                        RobotBoxShape_ForwdRoll();
                    }
                    break;

        case 'R':   if(RollLockFlag)
                    {
                        if(!BakRollSpeechFlag)
                        {
                            BakRollSpeechFlag = 1;
                            BakRollSpeech_Play();
                        }

                        //Backward_Rolling();
                    }
                    break;

        case 'C':   if(TransfrmFlag)
                    {
                        ForwardSpeechFlag = 0;
                        BackwrdSpeechFlag = 0;
                        ClckWisSpeechFlag = 0;
                        AticlckSpeechFlag = 0;
                        TrasfrmSpeechFlag = 0;
                        FrdRollSpeechFlag = 0;
                        BakRollSpeechFlag = 0;

                        HandGestureFlag = 0;                                     //清空标记，关闭综合功能

                        RobotHexShape_Stand();
                    }
                    break;

        case 'E':   if(TransfrmFlag)
                    {

                    }
                    break;

        default:    break;
    }

    switch(RemoteControlData)
    {
        case 5:     if(TransfrmFlag)
                    {
                        UPBoxLockFlag = 0;

                        RobotHexShape_Forward();
                    }
                    break;

        case 7:     if(TransfrmFlag)
                    {
                        UPBoxLockFlag = 0;

                        RobotHexShape_Backwrd();
                    }
                    break;

        case 8:     if(TransfrmFlag)
                    {
                        UPBoxLockFlag = 0;

                        RobotHexShape_AtiClck();
                    }
                    break;

        case 6:     if(TransfrmFlag)
                    {
                        UPBoxLockFlag = 0;

                        RobotHexShape_ClckWis();
                    }
                    break;

        case 13:    if(TransformHexFinsh)
                    {
                        TransfrmFlag = 0;
                        RollReadyFlag = 0;
                        RollFinshFlag = 0;

                        if(TransformBoxOrder < TRANSFORM_STEP)
                        {
                            RollLockFlag = 0;
                            TransformBoxOrder++;
                            TransformBoxFinsh = 0;
                        }
                        else
                        {
                            RollLockFlag = 1;
                            TransformBoxFinsh = 1;
                            TransformHexOrder = 0;
                        }

                        RobotBoxShape_Transform(TransformBoxOrder);
                    }
                    break;

        case 15:    if(TransformBoxFinsh)
                    {
                        RollLockFlag = 0;
                        
                        if(TransformHexOrder < TRANSFORM_STEP)
                        {
                            TransformHexOrder++;
                            TransformHexFinsh = 0;
                        }
                        else
                        {
                            TransfrmFlag = 1;
                            TransformBoxOrder = 0;
                            TransformHexFinsh = 1;
                        }

                        RobotHexShape_Transform(TransformHexOrder);
                    }
                    break;

        case 16:    if(RollLockFlag)
                    {
                        RobotBoxShape_ForwdRoll();
                    }
                    break;

        case 14:    if(RollLockFlag)
                    {
                        //Backward_Rolling();
                    }
                    break;

        case 1:     if(TransfrmFlag)
                    {
                        HandGestureFlag = 1;
                    }
                    break;
        
        case 2:     if(TransfrmFlag)
                    {
                        HandGestureFlag = 0;

                        RobotHexShape_Stand();
                    }
                    break;

        case 3:     ServoTorsion_Disable(30);
                    break;

        case 4:     if(TransfrmFlag)
                    {

                    }
                    break;

        case 11:    if(TransfrmFlag)
                    {
                        UPBoxLockFlag = 0;

                        RobotHexShape_LForward();
                    }
                    break;

        case 12:    if(TransfrmFlag)
                    {
                        UPBoxLockFlag = 0;

                        RobotHexShape_RForward();
                    }
                    break;

        case 9:     if(TransfrmFlag)
                    {
                        UPBoxLockFlag = 0;

                        RobotHexShape_LBackward();
                    }
                    break;

        case 10:    if(TransfrmFlag)
                    {
                        UPBoxLockFlag = 0;

                        RobotHexShape_RBackward();
                    }
                    break;

        default:    break;
    }
}
/********************************************************************/

