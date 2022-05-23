/*	
*	init_BKP.h
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INIT_BKP_H
#define INIT_BKP_H

#include <stdint.h>
/* Date Structure definition */
typedef struct
{
  uint8_t day;
  uint8_t month;
  uint16_t year;
}tDate;

/* Time Structure definition */
typedef struct
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
}tTime;

static tDate  sDate;
//tTime	sTime;

//void ClockConfigure(void);
//void Calendar_Init(void);
//void BKP_RTC_SetCounter(uint32_t CounterValue);
//uint32_t BKP_RTC_GetCounter(void);
void Date_Update(void);

#endif /* INIT_BKP_H */
