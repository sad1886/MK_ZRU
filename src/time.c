/**
  ******************************************************************************
  * @file    Menu_time.c
  * @author  Phyton Application Team
  * @version V1.0.0
  * @date    01.12.2011
  * @brief   This file contains all the "TIME" menu handlers.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_config.h"
//#include <MDR32F9Qx_port.h>
#include "MDR32F9x.h"
#include "MDR32F9Qx_bkp.h"
#include "MDR32F9Qx_rst_clk.h"
//#include "systick.h"
//#include "Menu.h"
//#include "Menu_items.h"
//#include "leds.h"
//#include "lcd.h"
//#include "text.h"
//#include "joystick.h"
#include "time.h"
//#include "MDR32F9Qx_it.h"

/* Time Structure definition */
typedef struct
{
  uint8_t sec_l;
  uint8_t sec_h;
  uint8_t min_l;
  uint8_t min_h;
  uint8_t hour_l;
  uint8_t hour_h;
}tTime;

/* Alarm Structure definition */
typedef struct
{
  uint8_t sec_l;
  uint8_t sec_h;
  uint8_t min_l;
  uint8_t min_h;
  uint8_t hour_l;
  uint8_t hour_h;
}tAlarm;

/* Date Structure definition */
typedef struct
{
  uint8_t day;
  uint8_t month;
  uint16_t year;
}tDate;

//static tTime  sTime;
//static tAlarm sAlarm;
static tDate  sDate;

/*******************************************************************************
* Function Name  : RTC_Configuration
* Description    : Configures the RTC.
*******************************************************************************/
void RTC_Configuration(void)
{
  /* Configure LSE as RTC clock source */
  RST_CLK_LSEconfig(RST_CLK_LSE_ON);
  /* Wait till LSE is ready */
  while (RST_CLK_LSEstatus() != SUCCESS)  {  }

  /* Select the RTC Clock Source */
  BKP_RTCclkSource(BKP_RTC_LSEclk);
  /* Wait until last write operation on RTC registers has finished */
  BKP_RTC_WaitForUpdate();

  /* Sets the RTC prescaler */
  BKP_RTC_SetPrescaler(RTC_PRESCALER_VALUE);
  /* Wait until last write operation on RTC registers has finished */
  BKP_RTC_WaitForUpdate();

  /* Sets the RTC calibrator */
  BKP_RTC_Calibration(RTC_CalibratorValue);
  /* Wait until last write operation on RTC registers has finished */
  BKP_RTC_WaitForUpdate();

  /* Enable the RTC Clock */
  BKP_RTC_Enable(ENABLE);

  /* Enable the Second interrupt */
  BKP_RTC_ITConfig(BKP_RTC_IT_SECF, ENABLE);
  NVIC_EnableIRQ(BACKUP_IRQn);
}

/*******************************************************************************
* Function Name  : IsLeapYear
* Description    : Check whether the passed year is Leap or not.
* Input          : nYear - Year value	* Return: 1: leap year; 0: not leap year
*******************************************************************************/
static uint32_t IsLeapYear(uint32_t nYear)
{
  if(nYear % 4 != 0) return 0;
  if(nYear % 100 != 0) return 1;
  return (uint8_t)(nYear % 400 == 0);
}

/*******************************************************************************
* Function Name  : Date_Update
* Description    : Updates date when time is 23:59:59.
*******************************************************************************/
void Date_Update(void)
{
  if(sDate.month == 1 || sDate.month == 3 || sDate.month == 5 || sDate.month == 7 ||
     sDate.month == 8 || sDate.month == 10 || sDate.month == 12)
  {
    if(sDate.day < 31)
    {
      sDate.day++;
    }
    /* Date structure member: sDate.day = 31 */
    else
    {
      if(sDate.month != 12)
      {
        sDate.month++;
        sDate.day = 1;
      }
      /* Date structure member: sDate.day = 31 & sDate.month =12 */
      else
      {
        sDate.month = 1;
        sDate.day = 1;
        sDate.year++;
      }
    }
  }
  else if(sDate.month == 4 || sDate.month == 6 || sDate.month == 9 ||
          sDate.month == 11)
  {
    if(sDate.day < 30)
    {
      sDate.day++;
    }
    /* Date structure member: sDate.day = 30 */
    else
    {
      sDate.month++;
      sDate.day = 1;
    }
  }
  else if(sDate.month == 2)
  {
    if(sDate.day < 28)
    {
      sDate.day++;
    }
    else if(sDate.day == 28)
    {
      /* Leap year */
      if(IsLeapYear(sDate.year))
      {
        sDate.day++;
      }
      else
      {
        sDate.month++;
        sDate.day = 1;
      }
    }
    else if(sDate.day == 29)
    {
      sDate.month++;
      sDate.day = 1;
    }
  }

  MDR_BKP->REG_01 = sDate.year + (sDate.month << 16) + (sDate.day << 24);
  /* Wait until last write operation on RTC registers has finished */
  BKP_RTC_WaitForUpdate();
}

/*******************************************************************************
* Function Name  : IsValidDate
* Description    : Checks if the given date valid.
* Input          : Day   - Day value; Month - Month value; Year  - Year value
* Return         : SUCCESS if the date is valid, ERROR otherwise.
*******************************************************************************/
ErrorStatus IsValidDate(uint32_t Day, uint32_t Month, uint32_t Year)
{
  if(Day == 0 || Month == 0 || Month > 12 || Year == 0)
  {
    return ERROR;
  }
  if((Month == 1 || Month == 3 || Month == 5 || Month == 7 ||
     Month == 8 || Month == 10 || Month == 12) && (Day > 31))
  {
    return ERROR;
  }
  if((Month == 4 || Month == 6 || Month == 9 || Month == 11) && (Day > 30))
  {
    return ERROR;
  }
  if(Month == 2)
  {
    if (IsLeapYear(Year))
    {
      if (Day > 29)
      return ERROR;
    }
    else
    {
      if (Day > 28)
      return ERROR;
    }
  }
  return SUCCESS;
}

/*******************************************************************************
* Function Name  : Calendar_Init
* Description    : Allows the user to initialize time and date values.
*******************************************************************************/
void Calendar_Init(void)
{
  /* Enables the HSE clock for BKP control */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_BKP,ENABLE);

  RTC_Configuration();
}

/* END OF FILE Menu_time.c */

