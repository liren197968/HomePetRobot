#ifndef __ROBOTCONTROL_H
#define __ROBOTCONTROL_H

#include <math.h>
#include "common.h"

/***************************数学计算常量********************************/
#define D1_LENH                             0
#define A1_LENH                             38
#define A2_LENH                             61.3
#define A3_LENH                             71.57
#define A4_LENH                             19

#define LEG_LENH                            118.3
#define LEG_HIGH                            69

#define ANGLE_TO_RADIAN                     3.141593 / 180
#define ROBOT_DOWN_ANGL                     15

#define ROBOT_LENH                          (A1_LENH + A2_LENH * cos(ROBOT_DOWN_ANGL * ANGLE_TO_RADIAN) + A4_LENH)
#define ROBOT_HIGH                          (A2_LENH * sin(ROBOT_DOWN_ANGL * ANGLE_TO_RADIAN) + LEG_HIGH)

#define THET_INIT_1                         60
#define THET_INIT_2                         0
#define THET_INIT_3                         60
#define BETA_INIT_1                         150
#define BETA_INIT_2                         90
#define BETA_INIT_3                         30
/**********************************************************************/
#define SERVO_ANGLE_TO_PWM                  1024 / 220.0                        //舵机角度与PWM数值转换比例长度
#define SERVO_PWM_CENTER                    512
#define SERVO_PWM_CENTER_1                  652
#define SERVO_PWM_CENTER_2                  512

#define FONT_ARM_INIT_ANGLE                 60
#define FONT_LEG_INIT_ANGLE                 0
#define FONT_FET_INIT_ANGLE                 -74.60
#define MIDE_ARM_INIT_ANGLE                 0
#define MIDE_LEG_INIT_ANGLE                 0
#define MIDE_FET_INIT_ANGLE                 -74.60
#define BACK_ARM_INIT_ANGLE                 60
#define BACK_LEG_INIT_ANGLE                 0
#define BACK_FET_INIT_ANGLE                 -74.60

#define FONT_X_INIT                         ROBOT_LENH / 2
#define FONT_Y_INIT                         ROBOT_LENH * 0.866
#define FONT_Z_INIT                         -ROBOT_HIGH
#define MIDE_X_INIT                         ROBOT_LENH
#define MIDE_Y_INIT                         0
#define MIDE_Z_INIT                         -ROBOT_HIGH
#define BACK_X_INIT                         ROBOT_LENH / 2
#define BACK_Y_INIT                         ROBOT_LENH * 0.866
#define BACK_Z_INIT                         -ROBOT_HIGH
/**************************************************************************************/
/********************************UP BOX************************************************/
#define UP_BOX_LEG_CLOS                     700
#define UP_BOX_FET_CLOS                     650

#define UP_BOX_LEG_OPEN                     680
#define UP_BOX_FET_OPEN                     770
/**************************************************************************************/
/******************************ROBOT BOX***********************************************/
#define BOX_CENTER_1                        509
#define BOX_CENTER_2                        150
#define BOX_CENTER_3                        303

#define BOX_CENTER_4                        524
#define BOX_CENTER_5                        122
#define BOX_CENTER_6                        275

#define BOX_CENTER_7                        505
#define BOX_CENTER_8                        128
#define BOX_CENTER_9                        296

#define BOX_CENTER_10                       513
#define BOX_CENTER_11                       140
#define BOX_CENTER_12                       302

#define BOX_CENTER_13                       496
#define BOX_CENTER_14                       144
#define BOX_CENTER_15                       299

#define BOX_CENTER_16                       514
#define BOX_CENTER_17                       124
#define BOX_CENTER_18                       284

#define BOX_CENTER_19                       223
#define BOX_CENTER_20                       175

#define BOX_CENTER_21                       216
#define BOX_CENTER_22                       179

#define BOX_CENTER_23                       210
#define BOX_CENTER_24                       179

#define BOX_CENTER_25                       220
#define BOX_CENTER_26                       178

#define BOX_CENTER_27                       209
#define BOX_CENTER_28                       158

#define BOX_CENTER_29                       232
#define BOX_CENTER_30                       188
/**************************************************************************************/
/************************************ROLLING*******************************************/
#define DN_BOX_ARM_REDY                     512
#define DN_BOX_LEG_REDY                     160
#define DN_BOX_FET_REDY                     320

#define UP_BOX_LEG_REDY                     640
#define UP_BOX_FET_REDY                     600


#define DN_BOX_ARM_ROLL                     512
#define DN_BOX_LEG_ROLL                     380
#define DN_BOX_FET_ROLL                     280

#define UP_BOX_LEG_ROLL                     500
#define UP_BOX_FET_ROLL                     750
/**************************************************************************************/
#define TRANSFORM_STEP                      12

#define ROBOT_LEG_LIFT                      90                                //机器人抬腿时腿部、脚部抬升增量
#define ROBOT_FET_LIFT                      90

#define STEP_LENH                           20

#define BASIC_FSTEP_ERROR                   0
#define BASIC_BSTEP_ERROR                   4
#define BASIC_LFSTEP_ERROR                  -2
#define BASIC_LBSTEP_ERROR                  0
#define BASIC_RFSTEP_ERROR                  -2
#define BASIC_RBSTEP_ERROR                  0
#define ROBOT_PID_CONST_P                   1
/**************************************************************************************/

void RobotHexShape_Stand(void);
void RobotInstruct_Control(void);

extern uint16_t             ServoPwmData[32];

#endif
