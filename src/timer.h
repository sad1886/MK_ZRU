#ifndef __TIMER_H
#define __TIMER_H

#include "MDR32F9x.h"

void Timer_Init(void);
void SysTickInit(uint32_t delay);

#endif	//__TIMER_H