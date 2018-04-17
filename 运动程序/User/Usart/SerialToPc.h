#ifndef __SERVO_TO_PC_H
#define __SERVO_TO_PC_H

#include "common.h"
#include "Servo.h"

#define MAX_NUM_LEN     12

int DoubleToString(double SendData, unsigned char *pString);
void StringData_Send(unsigned char *pString);
int SerialDoubleData_Send(double DoubleData);

#endif

