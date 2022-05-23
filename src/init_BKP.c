/*
*
*/
#include <stdint.h>
#include "MDR32F9x.h"
#include "init_BKP.h"

#define RST_CLK_FLAG_LSERDY     ((uint32_t)(0x00 | 13))
#define LSEonTimeOut  				  ((uint16_t)0x0600)

tTime	sTime, sTimeCurnt;
int NewDay;

//...................................................................
//void ClockConfigure(void)
//{
//  uint32_t temp;
//  /*** Configure CPU_PLL clock ***/
//  //begin...
//	/* Select CPUPLL source */
//  temp = MDR_RST_CLK->CPU_CLOCK;		  
//  temp &= (~(uint32_t)0x03);					/* Clear CPU_C1_SEL bits */
//  temp |= 0;												  /* Set the CPU_C1_SEL bits */
//  MDR_RST_CLK->CPU_CLOCK = temp; 			/* Store the new value = 0*/
//  /* Set CPUPLL multiplier */
//  temp = MDR_RST_CLK->PLL_CONTROL;
//  temp &= (~(uint32_t)(0x0F << 8));		/* Clear PLLMUL[3:0] bits */
//  temp |= (0<<8);										  /* Set the PLLMUL[3:0] bits */
//  MDR_RST_CLK->PLL_CONTROL = temp; 		/* Store the new value = 0*/

//	//...end

//  /*** Enables the RTCHSE clock on all ports ***/
//  //RST_CLK_PCLKcmd(ALL_PORTS_CLK, ENABLE);
//	MDR_RST_CLK->PER_CLOCK |= (1<<21)|(1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<29);// 0x2BE00010  A, B, C
//};

//...................................................................
/**
  * @brief  RST_CLK_LSEstatus - LSE clock status
  * @param  None
  * @retval enum ErrorStatus - SUCCESS if LSE clock is ready, else ERROR
  */
//ErrorStatus RST_CLK_LSEstatus(void)
//{ uint32_t flag, statusreg, startCounter = 0;
//  ErrorStatus state;

//	do  { /* Wait until LSE is ready or time out is occure */
//		statusreg = MDR_BKP->REG_0F;
//		flag = statusreg & 0x2000;											// (1<<RST_CLK_FLAG_LSERDY);
//    startCounter++;
//  } while ((startCounter < LSEonTimeOut) && (!flag));
//  if (flag) { state = SUCCESS; }
//  else			{ state = ERROR;   }
//  return state;
//}

//...................................................................
//void RTC_Configuration(void)
//{  uint32_t tmpreg;
//	
//  /* Configure LSE as RTC clock source */
//  //RST_CLK_LSEconfig(((uint32_t)0x00000001));				//((uint32_t)0x00000001)
//	MDR_BKP->REG_0F &= ~((uint32_t)(1 | 2));					/* Reset LSEON and LSEBYP bits before configuring the LSE */
//  MDR_BKP->REG_0F |= ((uint32_t)0x00000001);				/* Set LSEON bit */
//  while (RST_CLK_LSEstatus() != SUCCESS)  {  }		  /* Wait till LSE is ready */

//  /* Select the RTC Clock Source */
//  //BKP_RTCclkSource(((uint32_t)0x0004));
//	tmpreg  = MDR_BKP -> REG_0F & (~((uint32_t)0x0000000C));	/* Clear BKP_REG0F[3:2] bits */
//	tmpreg |= (uint32_t)0x00000004;														/* Set BKP_REG0F[3:2] bits according to RTC clock source*/
//	MDR_BKP -> REG_0F = tmpreg;
//  /* Wait until last write operation on RTC registers has finished */
//	while (MDR_BKP -> RTC_CS & (uint32_t)0x00000040)	{	}  			/* Loop until WEC flag is set */

//  /* Sets the RTC prescaler */
//	MDR_BKP -> RTC_PRL = 32768;
//  /* Wait until last write operation on RTC registers has finished */
//	while (MDR_BKP -> RTC_CS & (uint32_t)0x00000040)	{	}  			/* Loop until WEC flag is set */

//  /* Sets the RTC calibrator */
//  //BKP_RTC_Calibration(0);
//	tmpreg  = MDR_BKP -> REG_0F & (~((uint32_t)0x00001FE0));  /* Clear BKP_REG0F[12:5] bits */
//	tmpreg |= ((uint32_t)0x00001FE0) & (0 << 5);  /* Set BKP_REG0F[12:5] bits according to RTC clock source*/
//	MDR_BKP -> REG_0F = tmpreg;
//  /* Wait until last write operation on RTC registers has finished */
//	while (MDR_BKP -> RTC_CS & (uint32_t)0x00000040)	{	}  			/* Loop until WEC flag is set */

//  /* Enable the RTC Clock */
//  //BKP_RTC_Enable(1);
//	MDR_BKP -> REG_0F |= (uint32_t)0x00000010;						// *(__IO uint32_t *) RTC_ENABLE_BB = (uint32_t)1;

//  /* Enable the Second interrupt */
//  //BKP_RTC_ITConfig(((uint32_t)0x00000010), ENABLE);
//  MDR_BKP->RTC_CS |= ((uint32_t)0x00000010);
//  NVIC_EnableIRQ(BACKUP_IRQn);
//}

//...................................................................
//void Calendar_Init(void)
//{
//  /* Enables the HSE clock for BKP control */
//  //RST_CLK_PCLKcmd(RST_CLK_PCLK_BKP,ENABLE);
//  MDR_RST_CLK->PER_CLOCK |= (1<<27);					// PCLK[27] – BKP

//  RTC_Configuration();

//	sTime.sec = 0;	sTime.min = 0;	sTime.hour = 0;
//  /* Initialize Date structure with default values */
//  sDate.day = 16;  sDate.month = 10;  sDate.year  = 2014;
//  MDR_BKP->REG_01 = sDate.year + (sDate.month << 16) + (sDate.day << 24);
//  /* Wait until last write operation on RTC registers has finished */
//	while (MDR_BKP -> RTC_CS & (uint32_t)0x00000040)	{	}  			/* Loop until WEC flag is set */
//  /* Initialize Alarm backup register */
//  MDR_BKP->REG_02 = 0;
//  /* Wait until last write operation on RTC registers has finished */
//	while (MDR_BKP -> RTC_CS & (uint32_t)0x00000040)	{	}  			/* Loop until WEC flag is set */
//  MDR_BKP->RTC_CNT = 0;
//  /* Wait until last write operation on RTC registers has finished */
//	while (MDR_BKP -> RTC_CS & (uint32_t)0x00000040)	{	}  			/* Loop until WEC flag is set */
//}

//...................................................................
//void BKP_RTC_SetCounter(uint32_t CounterValue)
//{
//  MDR_BKP -> RTC_CNT = CounterValue;
//}

////...................................................................
//uint32_t BKP_RTC_GetCounter(void)
//{
//  return MDR_BKP -> RTC_CNT;
//}

//...................................................................
/*******************************************************************************
* Function Name  : IsLeapYear
* Description    : Check whether the passed year is Leap or not.
* Input          : nYear - Year value
* Output         : None
* Return         : 1: leap year
*                : 0: not leap year
*******************************************************************************/
static uint32_t IsLeapYear(uint32_t nYear)
{
  if(nYear % 4 != 0) return 0;
  if(nYear % 100 != 0) return 1;
  return (uint8_t)(nYear % 400 == 0);
}

//...................................................................
/*******************************************************************************
* Function Name  : Date_Update
* Description    : Updates date when time is 23:59:59.
*******************************************************************************/
void Date_Update(void)
{
  if(sDate.month == 1 || sDate.month == 3 || sDate.month == 5 || sDate.month == 7 ||		/* */
     sDate.month == 8 || sDate.month == 10 || sDate.month == 12)
  {
    if(sDate.day < 31)	 {  sDate.day++;  }
    else   							 {    /* sDate.day = 31 */
      if(sDate.month != 12)  {  sDate.month++;   sDate.day = 1;  }
      else     							 {  sDate.month = 1; sDate.day = 1; sDate.year++;  }  /* sDate.day = 31 & sDate.month =12 */
    }
  }
  else
		if(sDate.month == 4 || sDate.month == 6 || sDate.month == 9 || sDate.month == 11)
		{
			if(sDate.day < 30) {  sDate.day++;  }
			else   						 {  sDate.month++;   sDate.day = 1;  }    /* sDate.day = 30 */
		}
		else	{				/* sDate.month = 2 */
			if(sDate.month == 2)  {
				if(sDate.day < 28) 		   {  sDate.day++;  }
				else
					if(sDate.day == 28)		 {
						if(IsLeapYear(sDate.year))   {  sDate.day++;      }	/* Leap year */
						else    										 {  sDate.month++;   sDate.day = 1;  }
					}
					else if(sDate.day == 29)  		 {  sDate.month++;   sDate.day = 1;  }
			}
		}
//  MDR_BKP->REG_01 = sDate.year + (sDate.month << 16) + (sDate.day << 24);
//  /* Wait until last write operation on RTC registers has finished */
//	while (MDR_BKP -> RTC_CS & (uint32_t)0x00000040)	{	}  			/* Loop until WEC flag is set */
}

// end of init_.c
