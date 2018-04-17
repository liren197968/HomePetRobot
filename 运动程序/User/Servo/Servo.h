#ifndef __SERVO_H
#define __SERVO_H

#include "common.h"

#define MAX_SERVO_NUM                       32
#define SERVO_RUN_TIME_DEFAULT              300                                                 //舵机初始转动速度为每300ms一次
#define SEND_ANGLE_STR_MAX_SIZE             250                                                 //字符串最大长度为250
#define RECEIVE_ANGLE_STR_MAX_SIZE          12                                                  //读取角度字符串最大长度为12


void Servo_Init(void);
void Servo_Run(void);
void Servo_Stop(void);
void ServoRunTime_Set(uint16_t RunTime);
int8_t ServoTorsion_Enable(uint8_t ServoNum);
int8_t ServoTorsion_Disable(uint8_t ServoNum);
int8_t ServoAngleData_Read(uint8_t ServoNum);
void ServoAngleRead_Disp(uint8_t ServoNum);

#endif
