#include "init.h"
#include "MDR32F9x.h"

// Инициализация тактового генератора
//MDR_RST_CLK - Контроллер тактовой частоты, базов адр 0x4002_0000
void Clock_Init (void)
{	
	MDR_RST_CLK->HS_CONTROL = RST_CLK_HS_CONTROL_HSE_ON;															// 0x00000001 - Тактирование от внешнего кварца
	while ((MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_HSE_RDY)==0)			{	}			// 0x00000004 - Ожидание стабилиазации кварца

//	MDR_RST_CLK->PLL_CONTROL = (9<<RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos) | 						// Коэффициент умножения для ядра равен x+1=10, т.е. 10*8=80МГц
//															RST_CLK_PLL_CONTROL_PLL_CPU_ON;												// 0<<8|4. Бит включения PLL
	
	MDR_RST_CLK->PLL_CONTROL = (2<<RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos) | 						// Коэффициент умножения для ядра равен x+1=10, т.е. 3*8=24МГц
															RST_CLK_PLL_CONTROL_PLL_CPU_ON;												// 0<<8|4. Бит включения PLL
	
	while ((MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY)==0)	{	}			// Ожидание стабилиазации умножителя

	MDR_RST_CLK->CPU_CLOCK = ((RST_CLK_CPU_CLOCK_CPU_C1_SEL_Msk & 0x02) | 						// 0x3&0x2=HSE Выбор внешнего кварца в качестве общего источника тактирования,
														 RST_CLK_CPU_CLOCK_CPU_C2_SEL |													// 0x4
														(RST_CLK_CPU_CLOCK_CPU_C3_SEL_Msk & 0x0) |							// использование умноженной частоты кварца
														(1<<RST_CLK_CPU_CLOCK_HCLK_SEL_Pos));										// 1<<8 Биты выбора источника для HCLK: 01 – CPU_C3

	// Подача тактирования на:
	// 						 						 UART1, UART2, ADC, порты A, 		B, 			C, 			D, 			E,			F
	MDR_RST_CLK->PER_CLOCK |= (1<<6)|(1<<7)|(1<<17)|(1<<21)|(1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<29);					// Подача тактирования на порты A, B, C
//	MDR_RST_CLK->PER_CLOCK |= (0<<6)|(0<<7)|(1<<17)|(1<<21)|(1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<29);					// Подача тактирования на порты A, B, C
	// 						 						порты A, 		B, 			C, 			D, 			E,			F
//	MDR_RST_CLK->PER_CLOCK |= (1<<21)|(1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<29);					// Подача тактирования на порты A, B, C

}

//............................................................................................
/*******************************************************************************
* Function Name  : ClockConfig
* Description    : Configures the CPU_PLL and RTCHSE clock.
*******************************************************************************/
// void ClockConfig(void)
// {
//   /* Configure CPU_PLL clock */
//   //RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv1,0);		//RST_CLK_CPU_PLLsrcHSIdiv1 = 0
//   MDR_RST_CLK->PLL_CONTROL = 0x4;
//   MDR_RST_CLK->HS_CONTROL = 0x1;
//   MDR_RST_CLK->CPU_CLOCK = 0x106;
// //  MDR_RST_CLK->CPU_CLOCK = 0x104;
//   MDR_RST_CLK->ADC_MCO_CLOCK = 0x2F20;
//   MDR_RST_CLK->PER_CLOCK = 0x2BE200D2;
//   MDR_RST_CLK->CAN_CLOCK = 0x2000000;
//   MDR_RST_CLK->UART_CLOCK = 0x2000000;

//   /* Enables the RTCHSE clock on all ports */
//   //RST_CLK_PCLKcmd(ALL_PORTS_CLK, ENABLE);
// }

// //............................................................................................
// void RTC_Configuration(void)
// {  uint32_t tmpreg;

//   /* Configure LSE as RTC clock source */
//   //RST_CLK_LSEconfig(RST_CLK_LSE_ON);
//   /* Set LSEON bit */
//    MDR_BKP->REG_0F |= RST_CLK_LSE_ON;
//   /* Wait till LSE is ready */
//   while (RST_CLK_LSEstatus() != SUCCESS)	{	}

//   /* Select the RTC Clock Source */
//   //BKP_RTCclkSource(BKP_RTC_LSEclk);
//   /* Clear BKP_REG0F[3:2] bits */
//   tmpreg  = MDR_BKP -> REG_0F & (uint32_t) (~BKP_REG_0F_RTC_SEL_Msk);
//   /* Set BKP_REG0F[3:2] bits according to RTC clock source*/
//   tmpreg |= BKP_REG_0F_RTC_SEL_Msk & RTC_CLK;
//   MDR_BKP -> REG_0F = tmpreg;

//   /* Wait until last write operation on RTC registers has finished */
//   BKP_RTC_WaitForUpdate();

//   /* Sets the RTC prescaler */
//   BKP_RTC_SetPrescaler(RTC_PRESCALER_VALUE);
//   MDR_BKP -> RTC_PRL = PrescalerValue;

//   /* Wait until last write operation on RTC registers has finished */
//   BKP_RTC_WaitForUpdate();

//   /* Sets the RTC calibrator */
//   BKP_RTC_Calibration(RTC_CalibratorValue);
//   tmpreg  = MDR_BKP -> REG_0F & (uint32_t) (~BKP_REG_0F_CAL_Msk);
//   /* Set BKP_REG0F[12:5] bits according to RTC clock source*/
//   tmpreg |= BKP_REG_0F_CAL_Msk & (RTC_Calibration << BKP_REG_0F_CAL_Pos);
//   MDR_BKP -> REG_0F = tmpreg;
// 	
//   /* Wait until last write operation on RTC registers has finished */
//   BKP_RTC_WaitForUpdate();

//   /* Enable the RTC Clock */
//   BKP_RTC_Enable(ENABLE);
//   *(__IO uint32_t *) RTC_ENABLE_BB = (uint32_t)NewState;

//   /* Enable the Second interrupt */
//   BKP_RTC_ITConfig(BKP_RTC_IT_SECF, ENABLE);
//   if (NewState != DISABLE)
//   {
//     MDR_BKP->RTC_CS |= BKP_RTC_IT;
//   }
//   else
//   {
//     MDR_BKP->RTC_CS &= (uint32_t)~BKP_RTC_IT;
//   }
// 	
//   NVIC_EnableIRQ(BACKUP_IRQn);
// }

// //............................................................................................
// void Calendar_Init(void)
// {
// #define RST_CLK_PCLK_BKP            PCLK_BIT(MDR_BKP_BASE)
//   uint32_t tmp, i;
//   /* Enables the HSE clock for BKP control */
//   //RST_CLK_PCLKcmd(RST_CLK_PCLK_BKP,ENABLE);	//ENABLE=1
//   if (ENABLE)  {
//     MDR_RST_CLK->PER_CLOCK |= RST_CLK_PCLK_BKP;  }
//   else  {
//     MDR_RST_CLK->PER_CLOCK &= ~RST_CLK_PCLK_BKP;  }
// 	
//   RTC_Configuration();
// 	
//   if(MDR_BKP->REG_00 != 0x1234)  {  }
//   else  {
//     /* Initialize Date structure */
//     tmp = MDR_BKP->REG_01;
// 		//    sDate.day   = tmp >> 24;//    sDate.month = (tmp >> 16) & 0xFF;//    sDate.year  = tmp & 0xFFFF;
//     tmp = BKP_RTC_GetCounter();
//     if(tmp / 86399 != 0)    {
//       for(i = 0; i < (tmp / 86399); i++)   {  Date_Update();   }
//       BKP_RTC_SetCounter(tmp % 86399);    }
//   }
// }

//---------------------------------------------------------------------------------------------------------------------------------
// Инициализация системного таймера
void SysTickInit(uint32_t nMks)															// Значение в микросекундах
{
	//SysTick->LOAD = nMks*80;																	// HCLK = 80MHz
	SysTick->LOAD = nMks*24;																	// HCLK = 24MHz
	SysTick->CTRL = 1<<2|1<<1|1;															// Источник синхросигнала HCLK, разрешение прерывания, работа таймера разрешена
}

//---------------------------------------------------------------------------------------------------------------------------------
// Инициализация оконного сторожевого таймера в классическом режиме
// При частоте ядра 8 МГц период срабатывания составляет около 4 мс
void WWDT_Init (void)
{//част таймера (PCLK/4096)/8	110000000			1111111
	MDR_WWDG->CFR |= (WWDG_CFR_WGTB_Msk | WWDG_CFR_W_Msk);			// Максимальный делитель частоты, максимальный размер окна, прерывания выключены
}

//---------------------------------------------------------------------------------------------------------------------------------
/*/ Инициализация всех портов только для отладочной платы
void Ports_Init_Tst (void)
{
	//...................................................................................
	//	 						 						 ADC, порты A, 		B, 			D, 			E			F.
	MDR_RST_CLK->PER_CLOCK |=  (1<<17)|(1<<21)|(1<<22)|(1<<24)|(1<<25)|(1<<29);	
	
	//...................................................................................
	// Инициализация порта A										 PA4, PA3, PA2, PA1, PA0
	MDR_PORTA->OE = 0x0000001F;								// 4-0 выход IQ11..IQ7
	MDR_PORTA->FUNC = 0x00000000;							// Функция порта для всех выводов
	MDR_PORTA->ANALOG = 0x0000001f;						// Цифровой режим для всех задейсвованных выводов
	MDR_PORTA->PWR = 0x000002aa;							// Быстрый фронт
	
	// Инициализация порта D										 PD7, PD6, PD5, PD4, PD3, PD2, PD1, PD0
	MDR_PORTD->OE = 0x00000000;								// Все выводы настроены на вход
	MDR_PORTD->FUNC = 0x00000000;							// Функция порта, хотя для аналогового режима и не важно
	MDR_PORTD->ANALOG = 0x00000000;						// Аналоговый режим
	MDR_PORTD->PWR = 0x00000000;							// Отключаем передатчик порта
	
	//Uart2	....................................................................................
	MDR_PORTF->OE			|= (0<<0) | (1<<1);                     
	MDR_PORTF->FUNC   |= (3<<0) | (3<<2); 		// 3 - переопред 2 - альтернат 1 - основ 0 - порт
	MDR_PORTF->ANALOG |= (1<<0) | (1<<1);            
	MDR_PORTF->PWR		|= (2<<0) | (2<<2);			// 2 - быстрый порт

	//CAN2	....................................................................................
	MDR_PORTF->OE			|= (0<<2) | (1<<3);                     
	MDR_PORTF->FUNC   |= (3<<4) | (3<<6); 		// 3 - переопред 2 - альтернат 1 - основ 0 - порт
	MDR_PORTF->ANALOG |= (1<<2) | (1<<3);                 
	MDR_PORTF->PWR		|= (2<<4) | (2<<6);			// 2 - быстрый порт

}
*/

//---------------------------------------------------------------------------------------------------------------------------------
// Инициализация всех портов 
void Ports_Init (void)
{
	// Инициализация порта A										 PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0
	MDR_PORTA->OE = 0x000000A3;								// Выводы 7-6 - UART1; 5-0 выход IQ11..IQ7
	MDR_PORTA->FUNC = 0x0000F000;							// Функция порта для всех выводов, кроме 6 и 7 - UART1
	MDR_PORTA->ANALOG = 0x000000ff;						// Цифровой режим для всех задейсвованных выводов
	//MDR_PORTA->PWR = 0x0000AAAA;							// Быстрый фронт для всех
	MDR_PORTA->PWR = 0x0000f555;							// Максимально быстрые фронты Uart. Медленный фронт для всех
	MDR_PORTA->GFEN = 0x000000ff;							// Фильтр включен 
	
 	// Инициализация порта B										 PB0 - IQ12, PB3..10 - IQ13..20
 	MDR_PORTB->OE = 0x00000400;								// Выводы настроены на выход
 	MDR_PORTB->FUNC = 0x00000000;							// Функция порта
 	MDR_PORTB->ANALOG = 0x00000581;						// Цифровой режим для всех выводов
 	MDR_PORTB->PWR = 0x00228002;							// Быстрый фронт
 	
	// Инициализация порта C										 PC2, PC1, PC0 - IQ3..1
	MDR_PORTC->OE = 0x00000007;								// Все выводы настроены на вход
	MDR_PORTC->FUNC = 0x00000000;							// Функция порта
	MDR_PORTC->ANALOG = 0x00000007;						// Цифровой режим
	MDR_PORTC->PWR = 0x0000003f;							// Самый быстрый фронт

	MDR_PORTC->RXTX &= 0xfffc;								// Очистить биты
	
	// Инициализация порта D										 PD7, PD6, PD5, PD4, PD3, PD2, PD1, PD0
	MDR_PORTD->OE = 0x00000000;								// Все выводы настроены на вход
	MDR_PORTD->FUNC = 0x00000000;							// Функция порта, хотя для аналогового режима и не важно
	MDR_PORTD->ANALOG = 0x00000000;						// Аналоговый режим
	MDR_PORTD->PWR = 0x00000000;							// Отключаем передатчик порта
	
	// Инициализация порта E										 PE7, PE6, PE3, PE2, PE1, PE0
 	MDR_PORTE->OE = 0x0000008E;								// Вывод 7,6 - CAN2; 3,2 - порт; 1,0 - CAN1, 
 	MDR_PORTE->FUNC = 0x0000A00F;							// 			1010		0000		0000		1111
 	MDR_PORTE->ANALOG = 0x000000CF;						// Цифровой режим для выводов 7, 6, 3, 2, 1, 0.
 	MDR_PORTE->PWR = 0x0000F0FF;							// Самые быстрые фронты		1111		0000		1111		1111

	// Инициализация порта F									 	 PF3, PF2, PF1, PF0
	MDR_PORTF->OE |= 0x0000000E;							// Вывод 0 - вход, выводы 1 и 2 - выходы
	MDR_PORTF->FUNC = 0x0000000F;							// Вывод 2 - порт, выводы 0 и 1 - UART2
	MDR_PORTF->ANALOG |= 0x0000000F;					// Цифровой режим
	//MDR_PORTF->PWR |= 0x000000AA;							// Быстрые фронты
	MDR_PORTF->PWR |= 0x0000005f;							// Максимально быстрые фронты Uart
	MDR_PORTF->GFEN = 0x0000000f;							// Фильтр включен 
}

//---------------------------------------------------------------------------------------------------------------------------------
// Инициализация АЦП
void ADC_Init (void)
{
	// Ининиализация АЦП1
	MDR_ADC->ADC1_CFG = ADC1_CFG_REG_ADON |																		// Включение АЦП
											(3 << ADC1_CFG_REG_DIVCLK_Pos) |											// Выбор коэффициента деления частоты процессора 0011 – CPU_CLK = HCLK/8
											ADC1_CFG_TS_EN | 																			// Включения датчика температуры и источника опорного напряжения
											ADC1_CFG_REG_RNGC;																		// Разрешение автоматического контроля уровней
	MDR_ADC->ADC1_H_LEVEL = 0xFFF;
}

//---------------------------------------------------------------------------------------------------------------------------------
// Инициализация UART1 
void UART1_Init (void)
{	
	MDR_RST_CLK->PER_CLOCK |= (1<<6);																					// Подача тактирования на Uart1
	
 	// Деинициализация UART1
 	MDR_UART1->CR = 0;					MDR_UART1->DMACR = 0;				MDR_UART1->FBRD = 0;
 	MDR_UART1->FR = 0;					MDR_UART1->IBRD = 0;				MDR_UART1->ICR = 0;
 	MDR_UART1->IFLS = 0;				MDR_UART1->IMSC = 0;				MDR_UART1->LCR_H = 0;
 	MDR_UART1->RSR_ECR = 0;
 	
 	// Инициализация UART1
 	MDR_RST_CLK->UART_CLOCK |= (0<<RST_CLK_UART_CLOCK_UART1_BRG_Pos);					// Подача частоты HCLK без деления (8 МГц) на UART1
 	MDR_RST_CLK->UART_CLOCK |=  RST_CLK_UART_CLOCK_UART1_CLK_EN;							// Включение тактирования UART1
	
 	MDR_UART1->IBRD = 13;								                         							// 24MHz. целая часть BAUDDIV = 13 для скорости 115200 бит/сек
 	MDR_UART1->FBRD = 1;                 								 									 		// 24MHz. дробная часть BAUDDIV = 1
	
	MDR_UART1->LCR_H |= UART_LCR_H_PEN |																			// Разрешение проверки четности
											UART_LCR_H_EPS |																			// Бит четности дополняет количество единиц до четного числа
											UART_LCR_H_WLEN_Msk; 																	// Длина слова 8 бит 
	
	MDR_UART1->CR |= UART_CR_TXE;																							// Включение передатчика
 	MDR_UART1->CR |= UART_CR_RXE;																							// Включение приёмника
 	MDR_UART1->CR |= UART_CR_UARTEN;																					// Включить UART1	
	
 	MDR_UART1->IMSC |= UART_IMSC_RXIM;																				// Разрешение прерываний от приёмника UART1
	MDR_UART1->IMSC |= UART_IMSC_TXIM;																				// Разрешение прерываний от передатчика UART1
	//MDR_UART1->IMSC |= UART_IMSC_OEIM;
		
	//NVIC_EnableIRQ(UART1_IRQn);																								// Разрешение прерываний от UART1
	NVIC_ClearPendingIRQ(UART1_IRQn);
}

//---------------------------------------------------------------------------------------------------------------------------------
// Инициализация UART2 
void UART2_Init (void)
{
	MDR_RST_CLK->PER_CLOCK |= (1<<7);																					// Подача тактирования на Uart2
	
	// Деинициализация UART2
	MDR_UART2->CR = 0;					MDR_UART2->DMACR = 0;				MDR_UART2->FBRD = 0;
	MDR_UART2->FR = 0;					MDR_UART2->IBRD = 0;				MDR_UART2->ICR = 0;
	MDR_UART2->IFLS = 0;				MDR_UART2->IMSC = 0;				MDR_UART2->LCR_H = 0;
	MDR_UART2->RSR_ECR = 0;
	
	// Инициализация UART2
	MDR_RST_CLK->UART_CLOCK |= (0<<RST_CLK_UART_CLOCK_UART2_BRG_Pos);					// Подача частоты HCLK без деления (8 МГц) на UART2
	MDR_RST_CLK->UART_CLOCK |=  RST_CLK_UART_CLOCK_UART2_CLK_EN;							// Включение тактирования UART2

// 	MDR_UART2->IBRD = 43;								                         						 	// 80MHz. целая часть BAUDDIV = 43 для скорости 115200 бит/сек
// 	MDR_UART2->FBRD = 26;                 								 									 	// 80MHz. дробная часть BAUDDIV = 26
	
 	MDR_UART2->IBRD = 13;								                         						 	// 24MHz. целая часть BAUDDIV = 13 для скорости 115200 бит/сек
 	MDR_UART2->FBRD = 1;                 								 									 		// 24MHz. дробная часть BAUDDIV = 1
	
	MDR_UART2->LCR_H |= UART_LCR_H_PEN |																			// Разрешение проверки четности
											UART_LCR_H_EPS |																			// Бит четности дополняет количество единиц до четного числа
											UART_LCR_H_WLEN_Msk; 																	// Длина слова 8 бит 

	MDR_UART2->CR |= UART_CR_TXE;																							// Включение передатчика
	MDR_UART2->CR |= UART_CR_RXE;																							// Включение приёмника
	MDR_UART2->CR |= UART_CR_UARTEN;																					// Включить UART2

	MDR_UART2->IMSC |= UART_IMSC_RXIM;																				// Разрешение прерываний от приёмника UART2
	MDR_UART2->IMSC |= UART_IMSC_TXIM;																				// Разрешение прерываний от передатчика UART2
	//MDR_UART2->IMSC |= UART_IMSC_OEIM;
	
	//NVIC_EnableIRQ(UART2_IRQn);																								// Разрешение прерываний от UART2
	NVIC_ClearPendingIRQ(UART2_IRQn);
}

//---------------------------------------------------------------------------------------------------------------------------------
/*
− остановите работу приемопередатчика;
− дождитесь окончания приема и/или передачи текущего символа данных;
− сбросьте буфер передатчика путем установки бита FEN регистра UARTLCR_H в 0;
− измените настройки регистра UARTCR;
− возобновите работу приемопередатчика.
*/
// Сброс FIFO UART 
//void UART_Reset (void)
//{
//	NVIC_DisableIRQ(UART2_IRQn);																							// Запрет прерываний от UART2
//	MDR_UART2->CR = 0;																												// Работа запрещена UART2
//	MDR_UART2->LCR_H |= (UART_LCR_H_WLEN_Msk);																// Длина слова 8 бит, FIFO нет

//	MDR_UART2->LCR_H |= (UART_LCR_H_WLEN_Msk) | UART_LCR_H_FEN;								// Длина слова 8 бит FIFO есть
//	MDR_UART2->CR |= UART_CR_TXE;																							// Включение передатчика
//	MDR_UART2->CR |= UART_CR_RXE;																							// Включение приёмника
//	MDR_UART2->CR |= UART_CR_UARTEN;																					// Включить UART2
//	NVIC_EnableIRQ(UART2_IRQn);																								// Разрешение прерываний от UART2
//}	
