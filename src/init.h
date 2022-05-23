// Инициализация всей переферии
#include <stdint.h>

#ifndef INIT_H
#define INIT_H

void Clock_Init(void);								// Инициализация тактового генератора
void WWDT_Init (void);								// Инициализация оконного следящего таймера
void Ports_Init(void);								// Инициализация всех портов 
//void Ports_Init_Tst (void);
void ADC_Init (void);									// Инициализация АЦП и включение его прерываний
void UART1_Init (void); 							// Инициализация UART 
void UART2_Init (void); 							// Инициализация UART 
//void UART_Reset (void);
void SysTickInit(uint32_t nMks);			// Значение в микросекундах

#endif
