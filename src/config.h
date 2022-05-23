#ifndef __CONFIG_H
#define __CONFIG_H

#include "stdbool.h"
#include "MDR32F9x.h"

// enum {MCU1 = 0x01, ZRU1 = 0x07, ZRU2 = 0x09, ZRU3 = 0x0B, ZRU4 = 0x0D, RT1 = 0x03} ;

#define MCU1 0x01
#define ZRU1 0x07
#define ZRU2 0x09
#define ZRU3 0x0B
#define ZRU4 0x0D
#define RT1  0x03

#define ZRU
#define ADR ZRU3

#if ADR == ZRU1
	#define REG_EN ((1<<5)|(0<<7))	//на какие регистры DD5,DD6 должно приходить разрешение
	#define DT0 1487*4095/3300
#elif ADR == ZRU2
	#define REG_EN ((0<<5)|(1<<7))	//на какие регистры DD5,DD6 должно приходить разрешение
	#define DT0 1484*4095/3300
#elif ADR == ZRU3
	#define REG_EN ((1<<5)|(0<<7))	//на какие регистры DD5,DD6 должно приходить разрешение
	#define DT0 1483*4095/3300
#elif ADR == ZRU4
	#define REG_EN ((0<<5)|(1<<7))	//на какие регистры DD5,DD6 должно приходить разрешение
	#define DT0 1484*4095/3300
#endif

#define ADC0 Shim
#define ADC1 
#define ADC2
#define ADC3 Ush
#define ADC4
#define ADC5 Ish
#define ADC6
#define ADC7

extern uint32_t Ush;	
extern uint32_t Ish;	
extern uint32_t Uab;
extern uint32_t Izru;
extern bool zru_razr;
extern bool zru_zar;
extern bool zru_otkl;

extern int32_t Shim;

#endif	//__CONFIG_H
