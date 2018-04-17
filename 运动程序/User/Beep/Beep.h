#ifndef __BEEP_H
#define __BEEP_H

#include "common.h"

#define BEEP            PAout(15)

void Beep_Init(void);
void Beep_On(void);
void Beep_Off(void);
void Beep_BliBli(void);

#endif
