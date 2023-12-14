	/****************************************************************************
  * @file    Work.c, модуль функций программы управления ЗРУ									*
  * @author  ЗАО Орбита Карташов Ю.Д.																					*
  * @version V2.0.0																														*
  * @date    20.07.2015																												*
	****************************************************************************/

#include <stdlib.h>	
#include "MDR32F9x.h"
#include "parameters.h"
#include "init.h"											// Файл с описанием процедур инициализации и аппаратных настроек микроконтроллера
#include "Can.h"
#include "init_bkp.h"									// Файл с определениями структур времени
#include "Work.h"
#include "Uart.h"

//--------------------------- Общие ---------------------------------------------------------------------------------------
extern int iMUK_ZRU;

extern volatile unsigned char nBadAk[5];											// Номера отказавших аккумуляторов 
extern unsigned char bReqBCU[2];															// Флаг: поступил запрос (команда) от БЦУ
extern volatile unsigned char mode;														// Текущий режим работы контроллера, последний режим работы
//extern volatile unsigned int Errors;													// Слово состояния ошибок аппаратуры
extern volatile unsigned char iUst;														// Индекс текущей уставки 0..nUst-1

extern unsigned char stat1[3], stat2[3], stat3[3], stat4[3];

extern volatile float	curW_zar,																// Текущ заряд НВАБ
											W_raz,																	// Энергоёмк
											C_raz;																	// Ёмкость

extern int add_nbuf;
extern uint32_t Result;

extern volatile unsigned char StepAlgortmRazr, StepAlgortmZar;

uint32_t bitNotZar, bitNotRaz;																// Состояние проводных линий
volatile unsigned char bNoWrkCAN;															// 0 - CAN работает, 1 - CAN не работает
volatile unsigned char bRestData, vRestData;									// 1 - восстановить данные
volatile unsigned char bRestData_indiv;												// запрос на восстановление данных, индивидуальный
volatile unsigned char bSendStatus;														// послать байт состояния МУК ЗРУ

//--------------------------- ADC переменные ------------------------------------------------------------------------------
extern float U_ADC[nParams];
extern float Vals_ZRU[nParams];																// Массив реальных значений измеренных параметров ЗРУ
extern float Vals_ZRUold[nParams];														// Массив реальных значений измеренных параметров ЗРУ

float aI_razr, aI_razrOld, aI_zar;
float vU_zru, vU_zru_Old;																			// Uаб измеренное самим ЗРУ, а также предыдущее значение, необходимое для расчета W
union uBytesFloat16 aIrazr, aIzar;														// Для передачи по CAN в БЭ

volatile unsigned char OkDataADC;															// Значение канала АЦП готово
volatile int iadc;																						// Указатель на текущий канал измерения АЦП
volatile int iReadAdc;																				// Счётчик измерений текущего канала АЦП

volatile uint32_t ADC1_REG_GO=2;															// #define ADC1_CFG_REG_GO  ((uint32_t)0x00000002)
float Uadc;																										// Значение, прочитанное АЦП

volatile uint32_t cntReadADC;																	// Счётчик циклов чтения АЦП (для пропуска первых 3-х)

const int nCorrect = 2;																				// Числ пропусков ложного значения
int cCorrect[nParams];																				// Счётчик пропусков ложного значения

//--------------------------- RS485 переменные ----------------------------------------------------------------------------
//UART1 . . . . . . . . . . . . . . . . . . . . . . . . . . .
extern unsigned char pack1[Npack_Cmd];												// Буфер приёма пакетов
extern volatile int ind_pack1;																// Индекс приёмного буфера
extern volatile int lngPack1;																	// Длина принятого пакета

extern volatile int BatchSize1;																// Размер передаваемого пакета в байтах
extern volatile int ind_mas_trans1;														// Индекс передаваемого пакета
extern volatile unsigned char * p_ParRs1;
extern volatile unsigned short checksumCalc1, checksumIn1;

//UART2 . . . . . . . . . . . . . . . . . . . . . . . . . . .
extern unsigned char pack2[Npack_Cmd];												// Буфер приёма пакетов
extern volatile int ind_pack2;																// Индекс приёмного буфера
extern volatile int lngPack2;																	// Длина принятого пакета

extern volatile int BatchSize2;																// Размер пакета в байтах
extern volatile int ind_mas_trans2;
extern volatile unsigned char * p_ParRs2;
extern volatile unsigned short checksumCalc2, checksumIn2;

//.......................................................................................................................
extern unsigned char PackRs1[lngPackRs1];											// Пакет данных реальных значений измеренных параметров ЗРУ для RS485
extern int indsData_p[nParams];																// Номера индексов в блоке данных пакета телеметрии
extern float z_p[nParams];																		// z – цена (вес) младшего разряда;
extern float x0_p[nParams];																		// x0 – сдвиг нуля


//--------------------------- Преобразованные данные из БЭ ----------------------------------------------------------------
union uBytesFloat16 fVdatch[nfVdatch];
int 								iVdatch[niVdatch];

union uBytesFloat16 fV_AB[nfV_AB];
int 								iV_AB[niV_AB];

volatile unsigned char err_BE;

//--------------------------- CAN переменные ------------------------------------------------------------------------------
volatile unsigned char bRunCmdCAN, CurrentCmd, CurrentDlc;		// Флаг отправки команды по CAN
uint32_t ResultCAN;																						// Результат выполнения команд Вкл_РС, Откл_РС

volatile union uBytes64 Reciev_CanErrors;											// Телеметрия отказов БЭ
volatile union uBytes64 Reciev_CanDatch[nFrameDatchCAN];			// Телеметрия датчиков
volatile union uBytes64 Reciev_CanAB[nFrameABCAN];						// Телеметрия АБ

//--------------------------- переменные времени ------------------------------------------------------------------------------
extern tTime	sTime;
extern int NewDay;
extern volatile unsigned char mCount5Main, mCountSecMain;			// Счётчик 5 мин для измерения температуры АБ
extern volatile unsigned char mCount5, mCountSec, bPauza5m;		// Счётчик 5 мин для паузы, флаг начала счёта
extern volatile unsigned char sCount20, bPauza20;							// Счётчик 20 сек для задержки повтора 3 раза алгоритма заряда, флаг начала счёта
//extern volatile unsigned char bOneSec;												// sCount5,  bPauza5, Счётчик 5 сек, флаг начала счёта, флаг 1 сек
extern volatile unsigned char bTimeOutCmd;										// Флаг Время ожидания ответа результата от БЭ команды 

extern unsigned char set100ms, yes100ms;

//--------------------------- для алгоритма переменные ------------------------------------------------------------------------------
extern unsigned char mode_Razryad; //если не 0, значит мы находимся в режиме разряда (ток разряда больше нуля)
extern unsigned char mode_Zaryad; //если не 0, значит мы находимся в режиме заряда (ток заряда больше нуля) 

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Задержка
void Wait_ (int wait)	{	int i=0;	while (i < wait) i++;	}


//-------------------------------------------------------------------------------------------------------------------------
float abs_f (float vf)
{	if (vf>0)	return vf;	else 	return -vf;
}	

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& Команды управления ЗРУ &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// 0 ----------------------------------------------------------------------------------------------------------------------------------------------
//void pVkl_AB_ZRU (void)																				// "Подключить ЗРУ к АБ" 
//{	
//	MDR_PORTF->RXTX &= ~0x8;																		// Подключить ЗРУ к АБ		22/PF3
//}

// 0 ----------------------------------------------------------------------------------------------------------------------------------------------
//void pOtkl_AB_ZRU (void)																			// "Отключить ЗРУ от АБ" 
//{
//	MDR_PORTF->RXTX |= 0x8;																			// Отключить ЗРУ от АБ		22/PF3
//}

// 1 ----------------------------------------------------------------------------------------------------------------------------------------------
void pVkl_Shim_ZRU (int bWait)																// "ВКЛ ЗРУ" 
{	
	MDR_PORTF->RXTX |= 0x8;		Wait_(tWaitCmd);									// Подключить ЗРУ к АБ		22/PF3
	MDR_PORTA->RXTX |= 0x1;		Wait_(tWaitCmd);									// Подключить ЗРУ к СЭС 	25/PA0
	MDR_PORTC->RXTX |= 0x4;		Wait_(tWaitCmd);									// Откл КОМП
	MDR_PORTA->RXTX |= 0x2;		Wait_(tWaitCmd);									// ШИМ ЗРУ 								26/PA1
	stat1[iMUK_ZRU]	|= pwrZRU;
	stat3[iMUK_ZRU] |= vklZRU;																	// ЗРУ включено
	//if (bWait) Wait_(tWaitCmd);																	// Ожидание завершения коммутации силовый цепей
}

// 1 ----------------------------------------------------------------------------------------------------------------------------------------------
void pOtkl_Shim_ZRU (int bWait)																// "ОТКЛ ЗРУ" 
{
	MDR_PORTA->RXTX &= ~0x2;	Wait_(tWaitCmd);									// Отключить ШИМ ЗРУ 			26/PA1
	MDR_PORTA->RXTX &= ~0x1;	Wait_(tWaitCmd);									// Отключить ЗРУ от СЭС 	25/PA0
	MDR_PORTF->RXTX &= ~0x8;	Wait_(tWaitCmd);									// Отключить ЗРУ от АБ		22/PF3
	stat1[iMUK_ZRU]	&= ~pwrZRU;
	stat3[iMUK_ZRU] &= ~vklZRU;																	// ЗРУ отключено
	set100ms = 1;
	
	MDR_PORTA->RXTX |= (1<<2); 
	Wait_(2*tWaitCmd);
	MDR_PORTA->RXTX &= ~(1<<2);
	//if (bWait) Wait_(tWaitCmd);																	// Ожидание завершения коммутации силовый цепей
}

// 2 ----------------------------------------------------------------------------------------------------------------------------------------------
void pVkl_KOMP (void)																					//
{
//	MDR_PORTC->RXTX |= 0x4;																			// "ВКЛ КОМП" 	Включение подзаряда (3А)
	MDR_PORTC->RXTX &= ~0x4;																		//
	stat3[iMUK_ZRU] |= bZaprZar;
}

// 2 ----------------------------------------------------------------------------------------------------------------------------------------------
void pOtkl_KOMP (void)																				// "Выкл_КОМП"	
{
	MDR_PORTC->RXTX |= 0x4;																			// "ВКЛ КОМП" 	Включение подзаряда (3А)
//	MDR_PORTC->RXTX &= ~0x4;																		//
	stat3[iMUK_ZRU] &= ~bZaprZar;
}

// 3 ---------------------------------------------------------------------------------------------------------------------------------------------
void pVkl_Zapr_Zarayd (void)																	//
{/*
	Сброс значений Wраз и Сраз в ноль при расчете разрядной энергии АБ производится при переходе параметра «Запрет ЗАРЯДА в «1»*/	
	MDR_PORTC->RXTX |= 0x2;																			// Включить запред заряда АБ 20/PC1
	stat1[iMUK_ZRU]	&= ~bZaryad;
	stat3[iMUK_ZRU] |= bZaprZar;
}
// 3 ----------------------------------------------------------------------------------------------------------------------------------------------
void pOtkl_Zapr_Zarayd (void)																	//
{
	MDR_PORTC->RXTX &= ~0x2;																		// Отключить запред заряда АБ 20/PC1
	stat1[iMUK_ZRU]	|= bZaryad;
	stat3[iMUK_ZRU] &= ~bZaprZar;
}

// 4 ----------------------------------------------------------------------------------------------------------------------------------------------
void pVkl_Zapr_Razrayd (void)																	// 
{
	MDR_PORTC->RXTX |= 0x1;																			// "Вкл_Запр_РАЗР " 19/PC0
	stat1[iMUK_ZRU]	&= ~bRazryad;		
	stat3[iMUK_ZRU] |= bZaprRazr;
}

// 4 ----------------------------------------------------------------------------------------------------------------------------------------------
void pOtkl_Zapr_Razrayd (void)																//
{
	MDR_PORTC->RXTX &= ~0x1;																		// "Откл_Запр_РАЗР " 19/PC0
	stat1[iMUK_ZRU]	|= bRazryad;		
	stat3[iMUK_ZRU] &= ~bZaprRazr;
}

// 5 ----------------------------------------------------------------------------------------------------------------------------------------------
void pVkl_Test_Zarayd (void)																	// "Вкл_ТЕСТ ЗАР"
{
//	MDR_PORTC->RXTX |= 0x4;																		// "ВКЛ ЗАРЯД" 	Разрешение заряда АБ (PC2/21). ZAR1=1;	 OKR1=0;
//	MDR_PORTC->RXTX &= ~0x2;																	// Отключить запред заряда АБ 20/PC1
//	MDR_PORTC->RXTX |= 0x1;																		// "Вкл_Запр_РАЗР " 19/PC0
//	MDR_PORTE->RXTX &= ~0x8;																	// "ОТКЛ ТЕСТ РАЗРЯД" Запрет принудительного разряда АБ на шины СЭС в ТВЦ
	MDR_PORTE->RXTX &= ~0x4;																		// Разрешение тестового заряда АБ 23/PE2
}	
// 5 ----------------------------------------------------------------------------------------------------------------------------------------------
void pOtkl_Test_Zarayd (void)																	// "Откл_ТЕСТ ЗАР" 
{	
//	MDR_PORTC->RXTX |= 0x2;																		// Включить запред заряда АБ 20/PC1
//	MDR_PORTE->RXTX &= ~0x8;																	// "ОТКЛ ТЕСТ РАЗРЯД" Запрет принудительного разряда АБ на шины СЭС в ТВЦ
//	MDR_PORTC->RXTX &= ~0x1;																	// "Откл_Запр_РАЗР " 19/PC0
	MDR_PORTE->RXTX |= 0x4;																			// Отключение тестового заряда АБ 23/PE2
}
// 6 ----------------------------------------------------------------------------------------------------------------------------------------------
void pVkl_Test_Razrayd (void)																	// 24/PE3
{
	MDR_PORTE->RXTX &= ~0x8;																		// "ВКЛ ТЕСТ РАЗРЯД" Принудительный разряд АБ на шины СЭС в ТВЦ
}

// 6 ----------------------------------------------------------------------------------------------------------------------------------------------
void pOtkl_Test_Razrayd (void)																// 24/PE3
{
	MDR_PORTE->RXTX |= 0x8;																			// "ОТКЛ ТЕСТ РАЗРЯД" Запрет принудительного разряда АБ на шины СЭС в ТВЦ
//	MDR_PORTC->RXTX |= 0x2;																		// Включить запред заряда АБ 20/PC1
}

// 7 ----------------------------------------------------------------------------------------------------------------------------------------------
void pVkl_RS (int bWait)
{
	CurrentDlc = 0;		CurrentCmd = CAN_Vkl_RS;
	add_nbuf=0;
	CAN_SendCmd(AdrMUK_ZRU, CurrentDlc, CAN_Vkl_RS);
	bRunCmdCAN = 1;		bTimeOutCmd = 1;
}
// 7 ----------------------------------------------------------------------------------------------------------------------------------------------
void pOtkl_RS (int bWait)
{
	CurrentDlc = 0;		CurrentCmd = CAN_Otkl_RS;
	add_nbuf=0;
	CAN_SendCmd(AdrMUK_ZRU, CurrentDlc, CAN_Otkl_RS);
	bRunCmdCAN = 1;		bTimeOutCmd = 1;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------
void pNotCan (void)																						// При отказе 2-х CAN от БЭ управление проводными сигналами
{	//uint32_t bitNotZar, bitNotRaz;
	
//	bitNotRaz = 0x10 & MDR_PORTA->RXTX;													// Запр разряд 29/РА4
//	bitNotZar = 0x01 & MDR_PORTB->RXTX;													// Запр заряда 30/РВ0
//
//	if (bitNotRaz)	MDR_PORTC->RXTX |= (bitNotRaz>>4);					// Установить текущее состояние Вкл_Запр_РАЗР 19/PC0
//	else						MDR_PORTC->RXTX &= 0xfffe;									//~(bitNotRaz>>4);
//	if (bitNotZar)	MDR_PORTC->RXTX |= (bitNotZar<<1);					// Установить текущее состояние Вкл_Запр_ЗАР АБ 20/PC1
//	else						MDR_PORTC->RXTX &= 0xfffd;									//~(bitNotZar<<1);					

// 	
	bitNotZar = 0x080 & MDR_PORTB->RXTX;												// Запр заряда 44/РВ7
	bitNotRaz = 0x100 & MDR_PORTB->RXTX;												// Запр разряд 45/РB8

//	if (bitNotRaz)	MDR_PORTC->RXTX |= (bitNotRaz>>8);					// Установить текущее состояние Вкл_Запр_РАЗР 19/PC0
//	else						MDR_PORTC->RXTX &= 0xfffe;									//
//	if (bitNotZar)	MDR_PORTC->RXTX |= (bitNotZar>>6);					// Установить текущее состояние Вкл_Запр_ЗАР АБ 20/PC1
//	else						MDR_PORTC->RXTX &= 0xfffd;									//
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Функция отправки пакета в БЭ
void CAN_SendCmd(unsigned char adr_MUK_Z, unsigned char dlc, unsigned char cmd)
{	int lbuf_TX = nbuf_TX + add_nbuf;
	// Очистим буфер
	MDR_CAN1->CAN_BUF[lbuf_TX].ID 				=0;
	MDR_CAN1->CAN_BUF[lbuf_TX].DATAL 			=0;		MDR_CAN1->CAN_BUF[lbuf_TX].DATAH 				 =0;
	MDR_CAN1->CAN_BUF_FILTER[lbuf_TX].MASK=0;		MDR_CAN1->CAN_BUF_FILTER[lbuf_TX].FILTER =0;
	// заполним буфер
 	MDR_CAN1->CAN_BUF[lbuf_TX].ID = nPrior_ZRU<< 24 | adr_MUK_Z<< 20 | nMUKs_BE<< 16 | cmd<< 12 | 1<< 6 | 1;
	MDR_CAN1->CAN_BUF[lbuf_TX].DLC = (1 << CAN_IDE) | (1<< CAN_SSR) | (1<< CAN_R1) | dlc;								// Длина передаваемых данных в пакете (в байтах)
	if (cmd==CAN_NumBadAk)	{
		MDR_CAN1->CAN_BUF[lbuf_TX].DATAL = (nBadAk[3]<<24)|(nBadAk[2]<<16)|(nBadAk[1]<<8)|nBadAk[0];			// Четвёртый..первый байт в пакете
		MDR_CAN1->CAN_BUF[lbuf_TX].DATAH =  nBadAk[4];																										// Восьмой..пятый байт в пакете
	}		

	// отправка пакета
	MDR_CAN1->BUF_CON[lbuf_TX]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);																	// Запрос на отправку сообщения, установить бит TX_REQ

	//.................................................................................................
	// Очистим буфер
	MDR_CAN2->CAN_BUF[lbuf_TX].ID 				=0;
	MDR_CAN2->CAN_BUF[lbuf_TX].DATAL 			=0;		MDR_CAN2->CAN_BUF[lbuf_TX].DATAH 				 =0;
	MDR_CAN2->CAN_BUF_FILTER[lbuf_TX].MASK=0;		MDR_CAN2->CAN_BUF_FILTER[lbuf_TX].FILTER =0;
	// заполним буфер
 	MDR_CAN2->CAN_BUF[lbuf_TX].ID = nPrior_ZRU<< 24 | (adr_MUK_Z+1)<< 20 | nMUKs_BE<< 16 | cmd<< 12 | 1<< 6 | 1;
	MDR_CAN2->CAN_BUF[lbuf_TX].DLC = (1 << CAN_IDE) | (1<< CAN_SSR) | (1<< CAN_R1) | dlc;								// Длина передаваемых данных в пакете (в байтах)
	if (cmd==CAN_NumBadAk)	{
		MDR_CAN2->CAN_BUF[lbuf_TX].DATAL = (nBadAk[3]<<24)|(nBadAk[2]<<16)|(nBadAk[1]<<8)|nBadAk[0];			// Четвёртый..первый байт в пакете
		MDR_CAN2->CAN_BUF[lbuf_TX].DATAH =  nBadAk[4];																										// Восьмой..пятый байт в пакете
	}		

	// отправка пакета
	MDR_CAN2->BUF_CON[lbuf_TX]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);																	// Запрос на отправку сообщения, установить бит TX_REQ
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Функция отправки пакета состояния текущего МУКа ЗРУ в другие МУКи ЗРУ. Для мажоритирования битовых параметров
void CAN_SendStatusZRU(void)
//-------------------------------------------------------------------------------------------------------------------------------------------------
{	int lbuf_TX = nbuf_TX;
	
	if(bRestData_indiv)
		stat3[iMUK_ZRU] |= RestData; // Флаг на восстановление данных, обмениваемся индивидуальными значениями между МУКами
	else
		stat3[iMUK_ZRU] &= ~RestData; // Флаг на восстановление данных, обмениваемся индивидуальными значениями между МУКами
	
	
	// Очистим буфер
	MDR_CAN1->CAN_BUF[lbuf_TX].ID 				=0;
	MDR_CAN1->CAN_BUF[lbuf_TX].DATAL 			=0;		MDR_CAN1->CAN_BUF[lbuf_TX].DATAH 				 =0;
	MDR_CAN1->CAN_BUF_FILTER[lbuf_TX].MASK=0;		MDR_CAN1->CAN_BUF_FILTER[lbuf_TX].FILTER =0;
	
	// заполним буфер
 	MDR_CAN1->CAN_BUF[lbuf_TX].ID = nPrior_ZRU<< 24 | AdrCAN1_ZRU<< 20 | nMUKs_BE<< 16 | CAN_CopyCmd<< 12 | 1<< 6 | 1;
	MDR_CAN1->CAN_BUF[lbuf_TX].DLC = (1 << CAN_IDE) | (1<< CAN_SSR) | (1<< CAN_R1) | 4;								// Длина передаваемых данных в пакете (в байтах)

	MDR_CAN1->CAN_BUF[nbuf_TX].DATAL = (stat4[iMUK_ZRU]<<24)|(stat3[iMUK_ZRU]<<16)|(stat2[iMUK_ZRU]<<8)|stat1[iMUK_ZRU];		// Четвёртый..первый байт в пакете
	MDR_CAN1->CAN_BUF[lbuf_TX].DATAH = 0;
	// отправка пакета
	MDR_CAN1->BUF_CON[lbuf_TX]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);																// Запрос на отправку сообщения, установить бит TX_REQ
	
	//.................................................................................................
	lbuf_TX = nbuf_TX+1;
	// Очистим буфер
	MDR_CAN2->CAN_BUF[lbuf_TX].ID 				=0;
	MDR_CAN2->CAN_BUF[lbuf_TX].DATAL 			=0;		MDR_CAN2->CAN_BUF[lbuf_TX].DATAH 				 =0;
	MDR_CAN2->CAN_BUF_FILTER[lbuf_TX].MASK=0;		MDR_CAN2->CAN_BUF_FILTER[lbuf_TX].FILTER =0;
	// заполним буфер
 	MDR_CAN2->CAN_BUF[lbuf_TX].ID = nPrior_ZRU<< 24 | (AdrCAN2_ZRU)<< 20 | nMUKs_BE<< 16 | CAN_CopyCmd<< 12 | 1<< 6 | 1;
	MDR_CAN2->CAN_BUF[lbuf_TX].DLC = (1 << CAN_IDE) | (1<< CAN_SSR) | (1<< CAN_R1) | 4;								// Длина передаваемых данных в пакете (в байтах)

//	MDR_CAN2->CAN_BUF[lbuf_TX].DATAL =  (stat3[iMUK_ZRU]<<16)|(stat2[iMUK_ZRU]<<8)|stat1[iMUK_ZRU];		// Четвёртый..первый байт в пакете;																													// Код подтверждаемой команды
	MDR_CAN2->CAN_BUF[lbuf_TX].DATAL =  (stat4[iMUK_ZRU]<<24)|(stat3[iMUK_ZRU]<<16)|(stat2[iMUK_ZRU]<<8)|stat1[iMUK_ZRU];		// Четвёртый..первый байт в пакете;																													// Код подтверждаемой команды
	MDR_CAN2->CAN_BUF[lbuf_TX].DATAH = 0;
	// отправка пакета
	MDR_CAN2->BUF_CON[lbuf_TX]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);																// Запрос на отправку сообщения, установить бит TX_REQ
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Функция подтверждения пакета в БЭ CAN1
void CAN_SendConf_1 (unsigned char confcmd)
{	int lbuf_TX = nbuf_TX + add_nbuf;
	
	// Очистим буфер
	MDR_CAN1->CAN_BUF[lbuf_TX].ID 				=0;
	MDR_CAN1->CAN_BUF[lbuf_TX].DATAL 			=0;		MDR_CAN1->CAN_BUF[lbuf_TX].DATAH 				 =0;
	MDR_CAN1->CAN_BUF_FILTER[lbuf_TX].MASK=0;		MDR_CAN1->CAN_BUF_FILTER[lbuf_TX].FILTER =0;
	// заполним буфер
 	MDR_CAN1->CAN_BUF[lbuf_TX].ID = nPrior_ZRU<< 24 | AdrCAN1_ZRU<< 20 | nMUKs_BE<< 16 | CAN_MSG_Ok<< 12 | 1<< 6 | 1;
	MDR_CAN1->CAN_BUF[lbuf_TX].DLC = (1 << CAN_IDE) | (1<< CAN_SSR) | (1<< CAN_R1) | 1;								// Длина передаваемых данных в пакете (в байтах)

	MDR_CAN1->CAN_BUF[lbuf_TX].DATAL = confcmd | ((stat3[iMUK_ZRU] & vklZRU)<<3);																												// ((stat3[iMUK_ZRU] & vklZRU)<<3) = 0x10
//	MDR_CAN1->CAN_BUF[lbuf_TX].DATAL = (aIrazr.b[0] <<24)|(aIzar.b[1]<<16)|(aIzar.b[0] <<8)|confcmd;	// Четвёртый..первый байт в пакете
//	MDR_CAN1->CAN_BUF[lbuf_TX].DATAH =  aIrazr.b[1];																									// Восьмой..пятый байт в пакете

//	MDR_CAN1->CAN_BUF[lbuf_TX].DATAL = confcmd;																												// Код подтверждаемой команды
//	MDR_CAN1->CAN_BUF[lbuf_TX].DATAH = 0;																															// 

	// отправка пакета
	MDR_CAN1->BUF_CON[lbuf_TX]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);																// Запрос на отправку сообщения, установить бит TX_REQ
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Функция подтверждения пакета в БЭ CAN2
void CAN_SendConf_2(unsigned char confcmd)
{	int lbuf_TX = nbuf_TX + add_nbuf;
	
	// Очистим буфер
	MDR_CAN2->CAN_BUF[lbuf_TX].ID 				=0;
	MDR_CAN2->CAN_BUF[lbuf_TX].DATAL 			=0;		MDR_CAN2->CAN_BUF[lbuf_TX].DATAH 				 =0;
	MDR_CAN2->CAN_BUF_FILTER[lbuf_TX].MASK=0;		MDR_CAN2->CAN_BUF_FILTER[lbuf_TX].FILTER =0;
	// заполним буфер
 	MDR_CAN2->CAN_BUF[lbuf_TX].ID = nPrior_ZRU<< 24 | (AdrCAN2_ZRU)<< 20 | nMUKs_BE<< 16 | CAN_MSG_Ok<< 12 | 1<< 6 | 1;
	MDR_CAN2->CAN_BUF[lbuf_TX].DLC = (1 << CAN_IDE) | (1<< CAN_SSR) | (1<< CAN_R1) | 1;								// Длина передаваемых данных в пакете (в байтах)

	MDR_CAN2->CAN_BUF[lbuf_TX].DATAL = confcmd | ((stat3[iMUK_ZRU] & vklZRU)<<3);												//
//	MDR_CAN2->CAN_BUF[lbuf_TX].DATAL = (aIrazr.b[0] <<24)|(aIzar.b[1]<<16)|(aIzar.b[0] <<8)|confcmd;	// Четвёртый..первый байт в пакете
//	MDR_CAN2->CAN_BUF[lbuf_TX].DATAH =  aIrazr.b[1];																									// Восьмой..пятый байт в пакете

//	MDR_CAN2->CAN_BUF[lbuf_TX].DATAL = confcmd;																												// Код подтверждаемой команды
//	MDR_CAN2->CAN_BUF[lbuf_TX].DATAH = 0;																															// 

	// отправка пакета
	MDR_CAN2->BUF_CON[lbuf_TX]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);																// Запрос на отправку сообщения, установить бит TX_REQ
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Функция отправки пакета в БЭ
void CAN_SendBadNumAk(unsigned char adr_MUK_Z, unsigned char n_MUK_BE, unsigned char cmd)
{	unsigned char dlc=5;
	// Очистим буфера
	MDR_CAN1->CAN_BUF[nbuf_TX].ID 				=0;
	MDR_CAN1->CAN_BUF[nbuf_TX].DATAL 			=0;		MDR_CAN1->CAN_BUF[nbuf_TX].DATAH 				 =0;
	MDR_CAN1->CAN_BUF_FILTER[nbuf_TX].MASK=0;		MDR_CAN1->CAN_BUF_FILTER[nbuf_TX].FILTER =0;
	// отправка пакета
 	MDR_CAN1->CAN_BUF[nbuf_TX].ID = nPrior_ZRU << 24 | (adr_MUK_Z) << 20 | n_MUK_BE << 16 | cmd	<< 12 | 1 << 6  | 1;
	MDR_CAN1->CAN_BUF[nbuf_TX].DLC	 = (1 << CAN_IDE) | (1 << CAN_SSR) | (1 << CAN_R1) | dlc;					// Длина передаваемых данных в пакете (в байтах)

	MDR_CAN1->CAN_BUF[nbuf_TX].DATAL = (nBadAk[3]<<24)|(nBadAk[2]<<16)|(nBadAk[1]<<8)|nBadAk[0];			// Четвёртый..первый байт в пакете
	MDR_CAN1->CAN_BUF[nbuf_TX].DATAH =  nBadAk[4];																										// Восьмой..пятый байт в пакете

	MDR_CAN1->BUF_CON[nbuf_TX]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);																// Запрос на отправку сообщения, установить бит TX_REQ

	// Очистим буфера
	MDR_CAN2->CAN_BUF[nbuf_TX].ID 				=0;
	MDR_CAN2->CAN_BUF[nbuf_TX].DATAL 			=0;		MDR_CAN2->CAN_BUF[nbuf_TX].DATAH 				 =0;
	MDR_CAN2->CAN_BUF_FILTER[nbuf_TX].MASK=0;		MDR_CAN2->CAN_BUF_FILTER[nbuf_TX].FILTER =0;
	// отправка пакета
 	MDR_CAN2->CAN_BUF[nbuf_TX].ID = nPrior_ZRU << 24 | (adr_MUK_Z+1)<< 20 | n_MUK_BE << 16 | cmd	<< 12 | 1 << 6  | 1;
		MDR_CAN2->CAN_BUF[nbuf_TX].DLC	 = (1 << CAN_IDE) | (1 << CAN_SSR) | (1 << CAN_R1) | dlc;				// Длина передаваемых данных в пакете (в байтах)

	MDR_CAN2->CAN_BUF[nbuf_TX].DATAL = (nBadAk[3]<<24)|(nBadAk[2]<<16)|(nBadAk[1]<<8)|nBadAk[0];			// Четвёртый..первый байт в пакете
	MDR_CAN2->CAN_BUF[nbuf_TX].DATAH =  nBadAk[4];																										// Восьмой..пятый байт в пакете

	MDR_CAN2->BUF_CON[nbuf_TX]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);																// Запрос на отправку сообщения, установить бит TX_REQ
}

//-------------------------------------------------------------------------------------------------------------------------
// Начальный запуск или переход в нормальный режим
void EnableIRQ_ADC_CAN_UART (void)
{
	NVIC_EnableIRQ(ADC_IRQn);																	// Разрешить прерывание от АЦП в NVIC
	ADC_GO(iadc);																							// Запуск преобразования. Читает 1-й канал АЦП
	
 	NVIC_EnableIRQ(UART1_IRQn);																// Разрешение прерываний от UART1
	NVIC_EnableIRQ(UART2_IRQn);																// Разрешение прерываний от UART2
	
	NVIC_EnableIRQ(CAN1_IRQn);																// Разрешение прерываний
 	NVIC_EnableIRQ(CAN2_IRQn);																// Разрешение прерываний
	
	RS_RECEIVE1;																							// Установка Uart1 на приём, Uart_OE=0
	RS_RECEIVE2;																							// Установка Uart2 на приём, Uart_OE=0
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Выбор канала и включение АЦП
void ADC_Start(int ch)
{
	MDR_ADC->ADC1_CFG &= (~ADC1_CFG_REG_CHS_Msk);										// Сброс номера канала
	MDR_ADC->ADC1_CFG |= ((ch)<<ADC1_CFG_REG_CHS_Pos);							// Установить номер канала
//	MDR_ADC->ADC1_CFG |= ((ch+add_chanl)<<ADC1_CFG_REG_CHS_Pos);		// Установить номер канала
	MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_ADON;													// АЦП включено
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Зауск процесса преобразования
void ADC_GO (int nadc)
{	
	MDR_ADC->ADC1_STATUS |= ADC_STATUS_ECOIF_IE;							// Разрешение прерываний от АЦП
	ADC_Start(nadc);																					// Опрос начинается с iadc-го канала
	MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO;											// Запуск преобразования
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Измерение напряжения АЙП (PD0, PD1, PD2, PD4, PD6, PD7), iadc=0,1,2,4,6,7
//........................... КОД c 14.07.20..........................................
void PutParamADC (void)																																				// С 14.07.20
{		int i;	float delta;
	
	/*.....................................................................................
	* ТАБ1-1	Измерение тока АБ1							13/PD1		0…-30/+35 А												*
	* ТАБ1-2	Измерение тока АБ1							14/PD2		0…-30/+35 А												*
	* НАБ1		Измерение напряжения шины АБ		15/PD4		0…122 		В			k = 18.33	C = 80	*
	* ТМП1		Измерение температуры корпуса 	17/PD6		0…+90 		°С		Т = 10 мВ*Uизм*Кт	*
	* ТМП2		Измерение температуры корпуса 	18/PD7		0…+90 		°С											* 
	......................................................................................*/	

	U_ADC[iadc] = Uadc;

	switch (iadc)	{
		case 1:
		case 2:
			if (Uadc < cUsm[iMUK_ZRU][iadc-1])	{																																// З А Р Я Д
				//Vals_ZRU[iadc-1] = Uadc;																																				// Напряжение АЦП
				Vals_ZRU[iadc-1] = KoefZar[iMUK_ZRU][iadc-1] * (cUsm[iMUK_ZRU][iadc-1]-Uadc);											// Реальные значения I заряда
				// Определение статуса ЗРУ
				if ((Vals_ZRU[0] > fNul)||(Vals_ZRU[1] > fNul))	{																									// fNul = 0.8
					if (mode_Razryad != 0) 
					{
						StepAlgortmZar = bInitZarayd;																																//начинаем алгоритм заново
						//в момент перехода из одного режима в другой обнуляем эти флаги
						//stat2[iMUK_ZRU] = 0;
						//stat3[iMUK_ZRU] &= ~(errNoOgrTokRazr|errNoOgrTokZar|errPrevDopustT);  											
						stat2[iMUK_ZRU] &= ~(errNoVklRazr|errNoOtklRazr);																						// Сброс аварийных сообщений разряда
						stat3[iMUK_ZRU] &= ~(errNoOgrTokRazr|errPrevDopustT);  											
					} //если был разряд, значит нужно начать заново алгоритм заряда
					mode_Razryad = 0; //больше нет разряда								
					mode_Zaryad = 1; //теперь у нас заряд
					C_raz = 0;	W_raz = 0;
				}	
				// Получение значений Зарядного и Разрядного токов
				if (Vals_ZRU[iadc-1]<0.5)	Vals_ZRU[iadc-1] = 0;
				aIzar.Fdata = Vals_ZRU[iadc-1];																																		// Для передачи по CAN в БЭ
				aI_zar = Vals_ZRU[iadc-1];
				aI_razr = 0;	Vals_ZRU[iadc-1+2] = aI_razr;																												// Для телеметрии
			}																																																		
			else	{																																															// Р А З Р Я Д
				//Vals_ZRU[iadc-1+2] = Uadc;																																			// Напряжение АЦП
				Vals_ZRU[iadc-1+2] = KoefRazr[iMUK_ZRU][iadc-1] * (Uadc-cUsm[iMUK_ZRU][iadc-1]);									// Реальные значения I разряда
				// Определение статуса ЗРУ
				if ((Vals_ZRU[2] > fNul)||(Vals_ZRU[3] > fNul))	{
					if (mode_Zaryad != 0) 
					{
						StepAlgortmRazr = bInitRazryda;																															//начинаем алгоритм заново
						//в момент перехода из одного режима в другой обнуляем эти флаги							
						//stat2[iMUK_ZRU] = 0;
						//stat3[iMUK_ZRU] &= ~(errNoOgrTokRazr|errNoOgrTokZar|errPrevDopustT);  											
						stat2[iMUK_ZRU] &= ~(errNoVklZar|errNoOtklZar);																							// Сброс аварийных сообщений заряда
						stat3[iMUK_ZRU] &= ~(errNoOgrTokZar|errPrevDopustT);  											
					} //если был заряд, значит нужно начать заново алгоритм разряда
					mode_Zaryad = 0; //больше нет заряда						
					mode_Razryad = 1; //теперь у нас разряд 
				}
				// Получение значений Зарядного и Разрядного токов
				if (Vals_ZRU[iadc-1+2]<0.5)		Vals_ZRU[iadc-1+2] = 0;
				aIrazr.Fdata = Vals_ZRU[iadc-1+2];
				aI_razr = Vals_ZRU[iadc-1+2];
				aI_zar = 0;		Vals_ZRU[iadc-1] = aI_zar;
			}																																																		
			if (iadc == 2)	iadc++;																																							// Пропуск канала измерения АЦП №3
			break;
		
		case 4:		i = iadc-4;		iadc++;																																				// Пропуск канала измерения АЦП №5
			//Uadc = 3.5;
			Vals_ZRU[iadc-1] = KoefUABT[iMUK_ZRU][i] * Uadc + dUABT[iMUK_ZRU][i];																// Реальные значения U
			if (mode_Zaryad)		Vals_ZRU[iadc-1] -= aI_zar * KoefIzarABT[iMUK_ZRU];								// Корекция значения U при заряде
			if (mode_Razryad)		Vals_ZRU[iadc-1] += aI_razr* KoefIrazABT[iMUK_ZRU];								// Корекция значения U при разряде
			vU_zru = Vals_ZRU[iadc-1];																															// Uаб измеренное самим ЗРУ
			break;
			
		case 6:		i = iadc-5;		
			Vals_ZRU[iadc+1] = KoefUABT[iMUK_ZRU][i] * Uadc + dUABT[iMUK_ZRU][i];																// Реальные значения T1
			if (mode_Zaryad)		Vals_ZRU[iadc-1] -= aI_zar * KoefIzarT[iMUK_ZRU][iadc-6];					// Корекция Т1
			if (mode_Razryad)		Vals_ZRU[iadc-1] += aI_razr* KoefIrazT[iMUK_ZRU][iadc-6];					// Корекция Т1
		break;
		case 7:		i = iadc-5;		
			Vals_ZRU[iadc-1] = KoefUABT[iMUK_ZRU][i] * Uadc + dUABT[iMUK_ZRU][i];																// Реальные значения T2
			if (mode_Zaryad)		Vals_ZRU[iadc-1] -= aI_zar * KoefIzarT[iMUK_ZRU][iadc-6];					// Корекция Т2
			if (mode_Razryad)		Vals_ZRU[iadc-1] += aI_razr* KoefIrazT[iMUK_ZRU][iadc-6];					// Корекция Т2
		break;
	}			
	
	//.....................................................................................
	OkDataADC=0;	iReadAdc=0;									
	if (iadc < nLastChannel)	{	iadc++;	}																																		// Номер последнего канала чтения АЦП в ЗРУ		nDatch = 7
	else											{	iadc=0;
		if ((!(MDR_PORTA->RXTX & 0x20))&&(!(MDR_PORTF->RXTX & 0x04)))	{																				// Нет передачи по RS485
			
			for (i=0; i < nParams; i++)	 {																																			// Коррекция значений
				delta = Vals_ZRU[i] - Vals_ZRUold[i];
				if (abs_f(delta) >= 0.5)	{
					if (cCorrect[i] <= nCorrect)	{	Vals_ZRU[i] = Vals_ZRUold[i];	cCorrect[i]++;	}
					else													{	
					Vals_ZRUold[i] = Vals_ZRU[i];	cCorrect[i]=0;	}																									//akkCnCopy[i].Fdata = Vals_ZRU[i];	
				}
				else	{	
					if (abs_f(delta) > 0.1)	{	
						if (delta > 0)	{		Vals_ZRUold[i] += 0.1;	}	
						else						{		Vals_ZRUold[i] -= 0.1;	}	
						Vals_ZRU[i] = Vals_ZRUold[i];
					}
					else	{	
						if (abs_f(delta) > 0.01)	{	
							if (delta > 0)	{		Vals_ZRUold[i] += 0.01;	}	
							else						{		Vals_ZRUold[i] -= 0.01;	}	
							Vals_ZRU[i] = Vals_ZRUold[i];
						}
					}
					Vals_ZRUold[i] = Vals_ZRU[i];	cCorrect[i]=0;
				}
			}
			MakePack1();
		}
	}																																																				// Переход к каналу измерения Uref
	ADC_GO(iadc);																																														// Запуск преобразования
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Отладочная процедура для проверки преобразований реальных значений в код и обратно
//void Test (void)
//{	/*const	int	nArr=35;
//	float fznach=2.4, a=-1, b=2.4;
//	float fr[nArr], fr2[nArr];
//	unsigned char kod[nArr];
//	unsigned short kod2[nArr];
//	int i;
//	
//	for (i=0; i < nArr; i++)	 {
//		kod[i]=255*((fznach-a)/(b-a));
//		kod2[i]=0xffff*((fznach-a)/(b-a));
////		fr[i]=(a+b*kod[i])/255;
//		fr[i]=((b-a)*kod[i])/255+a;
//		fr2[i]=((b-a)*kod2[i])/0xffff+a;
//		fznach-=0.1;
//	}	*/
//}


// End of Work
