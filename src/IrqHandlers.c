/********************************************************************************************************
*	 IrqHandlers.c от 30.05.2014																																					*
********************************************************************************************************/

//#include <stdlib.h>	
#include "MDR32F9x.h"
#include "init.h"																													// Файл с описанием процедур инициализации и аппаратных настроек
#include "Can.h"
#include "init_bkp.h"																											// Файл инициализации BACKUP
#include "Work.h"
#include "Uart.h"

//--------------------------- ADC переменные ------------------------------------------------------------------------------
extern volatile int iadc;																									// Указатель на текущий канал измерения АЦП
extern float Uadc;																												// Значения, прочитанные из АЦП
extern const float UBitADC;																								// (мВ)0,708 Для Uопор=3В один разряд АЦП=3000/4096=0,732мВ. Бит = Uопорн/0xfff. 2900/4096=0.708

extern volatile int iReadAdc;																							// Счётчик измерений текущего канала АЦП
extern volatile unsigned char OkDataADC;																	// Значение канала АЦП готово
extern unsigned char OkResult;																						// Результат получен
extern volatile uint32_t ADC1_REG_GO;																			// #define ADC1_CFG_REG_GO  ((uint32_t)0x00000002)

uint32_t summa;																														// Сумма значений
volatile int NumReadAdc;																									// Число достоверных значений измерения АЦП текущего канала
uint32_t Result;																													// Результат чтения АЦП

extern float	AUcc[3];																										// (В) Опорное напряжение АЦП МУК1, МУК2, МУК3
extern const float	Uref[3];																							// (В) Прецизионное напряжение АЦП МУК1, МУК2, МУК3 на PD0


//--------------------------- CAN переменные ------------------------------------------------------------------------------
extern volatile unsigned char bRunCmdCAN, CurrentCmd;											// Флаг отправки команды по CAN
extern uint32_t ResultCAN;																								// Результат выполнения команд Вкл_РС, Откл_РС, Байт ошибок БЭ  по CAN

//extern volatile unsigned char bReciev_CanErrors;													// Флаги принятых фреймов телеметрии отказов БЭ
//extern volatile unsigned char bReciev_CanDatch[nFrameDatchCAN];						// Флаги принятых фреймов телеметрии датчиков
//extern volatile unsigned char bReciev_CanAB[nFrameABCAN];									// Флаги принятых фреймов телеметрии АБ

extern int nfRec_CanDatch1[numMUKs_BE],
					 nfRec_CanDatch2[numMUKs_BE];																		// Маска принятых фреймов телеметрии датчиков 0x3ff
extern int nfRec_CanAK1[numMUKs_BE],
					 nfRec_CanAK2[numMUKs_BE];																			// Маска принятых фреймов телеметрии АК 0xfffff

extern volatile union uBytes64 Reciev_CanErrors;													// Телеметрия отказов БЭ
extern volatile union uBytes64 Reciev_CanDatch[numMUKs_BE][nFrameDatchCAN];						// Телеметрия датчиков
extern volatile union uBytes64 Reciev_CanAB[numMUKs_BE][nFrameABCAN];									// Телеметрия АБ

extern volatile unsigned char bNoWrkCAN;																	// 1 - CAN не работает, 0 - CAN  работает

int cntSec_noCAN[6];																											// счётчики секунд неработоспособности CAN (1..6)

//int clrECAN[6]= {0x3e, 0x3d, 0x3b, 0x37, 0x2f, 0x1f};											// маски сброса битов неработоспособности соотв CAN

uint64_t mask;
uint32_t adrTx1, adrTx2;																									// 
	
//--------------------------- UART переменные ------------------------------------------------------------------------------
//UART1 . . . . . . . . . . . . . . . . . . . . . . . . . . .
extern unsigned char pack1[Npack_Cmd];																		// Буфер приёма пакетов
extern volatile int ind_pack1;																						// Индекс приёмного буфера
extern volatile int lngPack1;																							// Длина принятого пакета

extern volatile int BatchSize1;																						// Размер передаваемого пакета в байтах
extern volatile int ind_mas_trans1;																				// Индекс передаваемого пакета
extern volatile unsigned char * p_ParRs1;
extern volatile unsigned short checksumCalc1, checksumIn1;

//UART2 . . . . . . . . . . . . . . . . . . . . . . . . . . .
extern unsigned char pack2[Npack_Cmd];																		// Буфер приёма пакетов
extern volatile int ind_pack2;																						// Индекс приёмного буфера
extern volatile int lngPack2;																							// Длина принятого пакета

extern volatile int BatchSize2;																						// Размер пакета в байтах
extern volatile int ind_mas_trans2;
extern volatile unsigned char * p_ParRs2;
extern volatile unsigned short checksumCalc2, checksumIn2;

//.......................................................................................................................
extern unsigned char	secUart1, secUart2;																	// Счётчик секунд Uart1, флаг достижения двух секунд

extern unsigned char PackRs1[lngPackRs1];																	// Ответ на пакет 1
extern unsigned char PackRs2[lngPackRs2];																	// Ответ на пакет 2
extern unsigned char PackRs3[lngPackRs3];																	// Ответ на пакет 3
extern unsigned char PackRs4[lngPackRs4];																	// Ответ на пакет 4
extern unsigned char PackRs5[lngPackRs5];																	// Ответ на пакет 5
extern unsigned char PackRs6[lngPackRs6];																	// Ответ на пакет 6
extern unsigned char PackRs7[lngPackRs7];																	// Ответ на пакет 7

extern unsigned char bUstavkiBCU;
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

unsigned char bBadCmd2, bBadCmd1;
extern unsigned char bReqBCU[2];																					// Флаг: поступил запрос (команда) от БЦУ

//--------------------------- Time переменные ------------------------------------------------------------------------------
extern tTime	sTime;
extern int NewDay;
extern volatile unsigned char mCount5Main, mCountSecMain;									// Счётчик 5 мин для измерения температуры АБ
extern volatile unsigned char mCount5, mCountSec, bPauza5m;								// Счётчик 5 мин для паузы, флаг начала счёта
extern volatile unsigned char sCount20, bPauza20;													// Счётчик 20 сек для задержки повтора 3 раза алгоритма заряда, флаг начала счёта
extern volatile unsigned char sCount5, sWait5, bPauza5, bOneSec;					// Счётчик 5 сек, флаг начала счёта, флаг 1 сек

extern volatile unsigned char bTimeOutCmd;																// Флаг Время ожидания ответа результата от БЭ команды 
extern volatile float	time_Razr;

extern unsigned char bPauza, bPauza_R;																		// Флаг общей паузы
extern int sCount, sCount_R, sCount_2h;																		// Счётчик секунд общей паузы
extern int LimsCount, LimsCount_R;																				// Предельное (конечное) значение для счётчика секунд общей паузы

unsigned char	secST;																											// Счётчик секунд
unsigned char	secStat;																										// Счётчик секунд
extern unsigned char	secUart1, NoWrkUart1;																	// Счётчик секунд Uart1, флаг достижения двух секунд
extern unsigned char	secTimeOutCmd, AddSec;


//--------------------------- Общие переменные ------------------------------------------------------------------------------
extern int iMUK_ZRU;
extern volatile unsigned char mode;																				// Текущий режим работы контроллера, последний режим работы

extern unsigned char stat1[3],stat2[3],stat3[3],stat4[3];													// Сост ЗРУ14, Сост ЗРУ16, Сост ЗРУ16.																		14-15
extern volatile unsigned char bSendStatus;																// послать байт состояния МУК ЗРУ
extern volatile unsigned char bRestData, vRestData;												// 1 - восстановить данные

extern int tVkl_ZRU;
extern unsigned char set100ms, yes100ms;

//=================================================================================================================================================
void ADC_IRQHandler (void)																													// Обработчик прерывания от АЦП
//=================================================================================================================================================
{		
	if (!(MDR_ADC->ADC1_STATUS & ADC_STATUS_FLG_REG_OVERWRITE))	{											// Проверка на переписывание значения в регистре
		if ((MDR_ADC->ADC1_STATUS & ADC_STATUS_FLG_REG_EOCIF)==ADC_STATUS_FLG_REG_EOCIF)		// Проверка готовности данных в регистре
		{	
			Result = (MDR_ADC->ADC1_RESULT & ADC_RESULT_Msk);															// Чтение (nReadADC раз) результата преобразования во временный массив
			//Result = 4095;
			OkResult = 1;
		}
		else mode=ADC_ERR;																									  					// Перезапуск опроса текущей четвёрки
	}
	else	{
		mode=ADC_ERR;																																		// Перезапуск опроса текущей четвёрки
		MDR_ADC->ADC1_STATUS &=(!ADC_STATUS_FLG_REG_OVERWRITE);													// Сброс флага перезаписи
	}

	if (MDR_ADC->ADC1_STATUS & ADC_STATUS_FLG_REG_AWOIFEN)
	{
		MDR_ADC->ADC1_STATUS &= ~(ADC_STATUS_FLG_REG_AWOIFEN);
	}
	
}


//=================================================================================================================================================
void CAN1_IRQHandler()																														// Получает данные с 1-го МУКа. С 31.08.20
//=================================================================================================================================================
{	uint64_t dataH;			int i, bGet;
	uint32_t can1_status, RecievCanID, nom, cod, nFrame, DLC_last;									//     adrRx;, adrTx
	
	can1_status = MDR_CAN1->STATUS;

	// .......................................................................................................
	if(can1_status & (1<<CAN_RX_READY))		{																					 // Приём
			
			if(CAN1->CAN_BUF_CON[nbuf_RX] & (1<<CAN_RX_FULL))														 // 1 – принятое сообщение в буфере
			{	
				MDR_CAN1->BUF_CON[nbuf_RX] &= ~(1<<CAN_RX_FULL);
				RecievCanID = MDR_CAN1->CAN_BUF[nbuf_RX].ID;
			
				nom		 = 0x0000003f & RecievCanID;																					// номер фрейма
				nFrame = 0x0000003f & RecievCanID >> 6;																			// Число фреймов в пакете 
				cod		 = 0x0000000f & RecievCanID >> 12;																		// Код пакета
				adrTx1 = 0x0000000f & RecievCanID >> 20;																		// Адрес узла-передатчика пакета (МУК1..3 соотв 1,3,5)
	
				//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
				if (adrTx1 < 7)	{		// обслужить запросы только 1..6 адрес МУК1..3 БЭ

					cntSec_noCAN[adrTx1-1]=0;		bNoWrkCAN &= ~(1<<(adrTx1-1));
					
					bGet=0;
					if (nMUK_ZRU==nMUK3_ZRU)	{																								// Только для МУК3
						if ((adrTx1== AdrMUK3_BE)&&(cod!=2))	bGet=1;														// Принять пакет 1,3,4 от МУК3				 
						if ((adrTx1 < AdrMUK3_BE)&&((cod==1)||(cod==2)))	bGet=1;								// Принять пакет 1,2   от МУК1, МУК2	 
					}
					else	{
						if ((adrTx1+6)==AdrCAN1_ZRU)	bGet=1;																		// Для этого МУКа
					}

					// ...............................................................................................
					if (bGet)
					{
						if (nom==nFrame)	{
							DLC_last = 0x0000000f & MDR_CAN1->CAN_BUF[nbuf_RX].DLC;								// Последний фрейм
							mask = 0x0;		for(i=1;i<DLC_last+1;i++)	{	mask = (mask<<8)|1;	}
							mask = mask*0xff;
						}	else	mask = 0xffffffffffffffff;

						// ...............................................................................................
						switch (adrTx1)	{															
						case 1:
						case 2:		i = 0;		break;						// МУК1
						case 3:
						case 4:		i = 1;		break;						// МУК2
						case 5:
						case 6:		i = 2;		break;						// МУК3
						}
						// ...............................................................................................
						nom--;

						dataH	= MDR_CAN1->CAN_BUF[nbuf_RX].DATAH;
					
						switch (cod)	{
						case CAN_PI_Datch:																											// Получение телеметрии ДД, ДТ
							nfRec_CanDatch1[i] |= 1<<nom;
							Reciev_CanDatch[i][nom].data64 = mask & (dataH<<32 | MDR_CAN1->CAN_BUF[nbuf_RX].DATAL);
							break;
						case CAN_PI_AK:																													// Получение телеметрии аккумуляторов 
								nfRec_CanAK1[i] |= 1<<nom;
								Reciev_CanAB[i][nom].data64	 = mask & (dataH<<32 | MDR_CAN1->CAN_BUF[nbuf_RX].DATAL);
							break;
						case CAN_ResCmd:																												// Результат выполнения команд
							ResultCAN											 = 0x0000ffff & MDR_CAN1->CAN_BUF[nbuf_RX].DATAL;
							if ((ResultCAN&0x000000ff)==1)	{	ResultCAN = ResultCAN>>8;						// Команда активирована успешно
								if (ResultCAN == CurrentCmd)	{	bRunCmdCAN=0;	}
							}
							break;
						case CAN_StatErr:																												// Получение данных отказов аппаратуры
							Reciev_CanErrors.data64			= mask & (dataH<<32 | MDR_CAN1->CAN_BUF[nbuf_RX].DATAL);
							break;
						default:
						;
						}
						MDR_CAN1->CAN_BUF[nbuf_RX].DATAL=0;		MDR_CAN1->CAN_BUF[nbuf_RX].DATAH=0;
					}	// if (bGet)
				}	// if ((adrTx1<7))
					
				//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
				else	{		// обслужить запросы МУК1..3 ЗРУ. Адр 7,8; 9,10; 11,12/
					if (cod==CAN_CopyCmd)	{
						
						switch (adrTx1)	{															
						case 7:
						case 8:		i = 0;		break;						// МУК1
						case 9:
						case 10:	i = 1;		break;						// МУК2
						case 11:
						case 12:	i = 2;		break;						// МУК3
						}
						if (i != iMUK_ZRU)	{
							stat1[i] = MDR_CAN1->CAN_BUF[nbuf_RX].DATAL;
							stat2[i] = MDR_CAN1->CAN_BUF[nbuf_RX].DATAL >> 8;
							stat3[i] = MDR_CAN1->CAN_BUF[nbuf_RX].DATAL >> 16;
							stat4[i] = MDR_CAN1->CAN_BUF[nbuf_RX].DATAL >> 24;

							// Флаг на восстановление данных, общий. Равен 1 если хотя бы в одном из трех МУКов индивидуальный флаг равен 1
							// Учитывая, что в обмене между МУКами участвует индивидуальный флаг - нет необходимости организовывать паузу с помощью vRestData, tRestData
							// Покуда индивидуальный флаг в МУКе не будет сброшен - он будет вынуждать общий флаг быть 1, как только в трех МУКах индивидуальные флаги будут сброшены (ОЗВД) - сбросится и общий									
							bRestData = ((stat3[0] & RestData)>>3 ) | ((stat3[1] & RestData)>>3 ) | ((stat3[2] & RestData)>>3 ); 
							
							// это старая запись, на всякий случай пусть будет, если вдруг понадобится ждать время
							//if ((!bRestData)&&(!vRestData))		{
							//	bRestData = (stat3[i] & RestData)>>3;															// Флаг на восстановление данных
							//	if (bRestData)	vRestData = tRestData;														// Время ожидания повторного запроса
							//}
							
//							if (stat1[i] != 0x40)
//								stat1[i] = 0x40;											
						}
					}	
				}	
			} // if(CAN1->CAN_BUF_CON[nbuf_RX] & (1<<CAN_RX_FULL))											// 1 – принятое сообщение в буфере
	}
}

//=================================================================================================================================================
void CAN2_IRQHandler()																														// Получает данные с 1-го МУКа. С 31.08.20
//=================================================================================================================================================
{	uint64_t dataH;			int i, bGet;
	uint32_t can2_status, RecievCanID, nom, cod, nFrame, DLC_last;									//     adrRx;, adrTx
	
	can2_status = MDR_CAN2->STATUS;

	// .......................................................................................................
	if(can2_status & (1<<CAN_RX_READY))		{																					// Приём
			
			if(CAN2->CAN_BUF_CON[nbuf_RX] & (1<<CAN_RX_FULL))														// 1 – принятое сообщение в буфере
			{	
				MDR_CAN2->BUF_CON[nbuf_RX] &= ~(1<<CAN_RX_FULL);
				RecievCanID = MDR_CAN2->CAN_BUF[nbuf_RX].ID;
			
				nom		 = 0x0000003f & RecievCanID;																				// номер фрейма
				nFrame = 0x0000003f & RecievCanID >> 6;																		// Число фреймов в пакете 
				cod		 = 0x0000000f & RecievCanID >> 12;																	// Код пакета
				adrTx2 = 0x0000000f & RecievCanID >> 20;																	// Адрес узла-передатчика пакета (МУК1..3 соотв 2,4,6)
	
				if (adrTx2 < 7)	{																													// обслужить запросы только 1..6 адрес МУК1..3 БЭ

					cntSec_noCAN[adrTx2-1]=0;		bNoWrkCAN &= ~(1<<(adrTx2-1));
					
					bGet=0;
					if (nMUK_ZRU==nMUK3_ZRU)	{																							// Только для МУК3
						if ((adrTx2==(AdrMUK3_BE+1))&&(cod!=2))	bGet=1;												// Принять пакет 1,3 от МУК3				 
						if ((adrTx2 < AdrMUK3_BE	 )&&((cod==1)||(cod==2)))	bGet=1;						// Принять пакет 1,2 от МУК1, МУК2	 
					}
					else	{
						if ((adrTx2+6)==AdrCAN2_ZRU)	bGet=1;																	// Для этого МУКа
					}

					if (bGet)																																// Принять пакет
					{
						if (nom==nFrame)	{
							DLC_last = 0x0000000f & MDR_CAN2->CAN_BUF[nbuf_RX].DLC;							// Последний фрейм
							mask = 0x0;		for(i=1;i<DLC_last+1;i++)	{	mask = (mask<<8)|1;	}
							mask = mask*0xff;
						}	else	mask = 0xffffffffffffffff;

						// ...............................................................................................
						switch (adrTx1)	{															
						case 1:
						case 2:		i = 0;		break;						// МУК1
						case 3:
						case 4:		i = 1;		break;						// МУК2
						case 5:
						case 6:		i = 2;		break;						// МУК3
						}
						// ...............................................................................................
						nom--;

						dataH	= MDR_CAN2->CAN_BUF[nbuf_RX].DATAH;
					
						switch (cod)	{
						case CAN_PI_Datch:																										// Получение телеметрии ДД, ДТ
							nfRec_CanDatch2[i] |= 1<<nom;
							Reciev_CanDatch[i][nom].data64	= mask & (dataH<<32 | MDR_CAN2->CAN_BUF[nbuf_RX].DATAL);
							break;
						case CAN_PI_AK:																												// Получение телеметрии аккумуляторов 
								nfRec_CanAK2[i] |= 1<<nom;
								Reciev_CanAB[i][nom].data64	= mask & (dataH<<32 | MDR_CAN2->CAN_BUF[nbuf_RX].DATAL);
							//}
							break;
						case CAN_ResCmd:																											// Результат выполнения команд
							ResultCAN										= 0x0000ffff & MDR_CAN2->CAN_BUF[nbuf_RX].DATAL;
							if ((ResultCAN&0x000000ff)==1)	{	ResultCAN = ResultCAN>>8;					// Команда активирована успешно
								if (ResultCAN == CurrentCmd)	{	bRunCmdCAN=0;	}
							}
							break;
						case CAN_StatErr:																											// Получение данных отказов аппаратуры
							Reciev_CanErrors.data64			= mask & (dataH<<32 | MDR_CAN2->CAN_BUF[nbuf_RX].DATAL);
							break;
						default:
						;
						}
						MDR_CAN2->CAN_BUF[nbuf_RX].DATAL=0;		MDR_CAN2->CAN_BUF[nbuf_RX].DATAH=0;
					}	// if (bGet)
				}	// if ((adrTx2<7))
					
				//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
				else	{																																		// обслужить запросы МУК1..3 ЗРУ
					if (cod==CAN_CopyCmd)	{
						
						switch (adrTx2)	{															
						case 7:
						case 8:		i = 0;		break;						// МУК1
						case 9:
						case 10:	i = 1;		break;						// МУК2
						case 11:
						case 12:	i = 2;		break;						// МУК3
						}
						if (i != iMUK_ZRU)	{
							stat1[i] = MDR_CAN2->CAN_BUF[nbuf_RX].DATAL;
							stat2[i] = MDR_CAN2->CAN_BUF[nbuf_RX].DATAL >> 8;
							stat3[i] = MDR_CAN2->CAN_BUF[nbuf_RX].DATAL >> 16;
							stat4[i] = MDR_CAN2->CAN_BUF[nbuf_RX].DATAL >> 24;

							// Флаг на восстановление данных, общий. Равен 1 если хотя бы в одном из трех МУКов индивидуальный флаг равен 1
							// Учитывая, что в обмене между МУКами участвует индивидуальный флаг - нет необходимости организовывать паузу с помощью vRestData, tRestData
							// Покуда индивидуальный флаг в МУКе не будет сброшен - он будет вынуждать общий флаг быть 1, как только в трех МУКах индивидуальные флаги будут сброшены (ОЗВД) - сбросится и общий									
							bRestData = ((stat3[0] & RestData)>>3 ) | ((stat3[1] & RestData)>>3 ) | ((stat3[2] & RestData)>>3 ); 
							
							// это старая запись, на всякий случай пусть будет, если вдруг понадобится ждать время
							//if ((!bRestData)&&(!vRestData))		{
							//	bRestData = (stat3[i] & RestData)>>3;															// Флаг на восстановление данных
							//	if (bRestData)	vRestData = tRestData;														// Время ожидания повторного запроса
							//}
							
//							if (stat1[i] != 0x40)
//								stat1[i] = 0x40;									
						}
					}	
				}	
			} // if(CAN2->CAN_BUF_CON[nbuf_RX] & (1<<CAN_RX_FULL))													// 1 – принятое сообщение в буфере
	}
}



//=================================================================================================================================================
void SysTick_Handler()
//=================================================================================================================================================
{
	if ((bTimeOutCmd)&&(secTimeOutCmd<(10-iMUK_ZRU)))	{	secTimeOutCmd++;	}										// Обслуживание таймаута 1 сек
	else																		{	secTimeOutCmd = 0;	bTimeOutCmd = 0;	}
	
	secST++;
	if (secST >= 10)	{	secST = 0;	AddSec = 1;	}																			// Достигнута секунда

	secStat++;
	if (secStat >= 7)	{	secStat = 0;	bSendStatus = 1;	}														// Достигнуто 0.7 секунды

	if (tVkl_ZRU)		tVkl_ZRU--;																											// Ожидание Вкл_ЗРУ

	if (set100ms) 	{	yes100ms = 1;	set100ms = 0;}
	else							yes100ms = 0;
}
		

//=================================================================================================================================================
void UART1_IRQHandler(void)
//=================================================================================================================================================
{		uint32_t wRIS;		unsigned char data;
	
	secUart1=0;
	wRIS = MDR_UART1->RIS;
	
	// Приём .................................................................
	if (wRIS & UART_RIS_RXRIS)	{
		
		data = MDR_UART1->DR;
//		if (!(MDR_UART1->RSR_ECR & (UART_RSR_ECR_FE | UART_RSR_ECR_PE | 
//																UART_RSR_ECR_BE | UART_RSR_ECR_OE)))							// Проверка на наличие ошибок приёма байта по UART
//		{	
			// Байт в буфер ......................................................
			pack1[ind_pack1] = data;																										// Запись байта данных в буфер

			bBadCmd1 = 0;
			
			if (pack1[0]==START_BYTE)	{																									// Начало пакета
			
				if (ind_pack1>3)	{																												// Число байт в буфере
						
					if ((pack1[1]==nMUK_BCU)&&(pack1[2]==Adr_RS_ZRU))		{										// 1 адрес БЦУ
								
						switch (pack1[3])	{															
						case gStat_ZRU:																												// телеметрия ЗРУ
						case gStat_AB_short:																									// Сост_АБ_Кратк_БЭ - краткая телеметрия БЭ
						case gStat_AB_Full:																										// Сост_АБ_Полн_БЭ – полная телеметрия БЭ
						case gUstavki_Tst:																										// контроль параметров (уставок) алгоритмов ЗРУ
						case gSaveData_to_BCU:																								// запоминаемые для восстановления данные в БВС 
						case gParamZRU:																												// частные параметры ЗРУ 
						case gTstLine:						lngPack1 = lngStat_ZRU;			break;					// проверка связи
						
						case gCmd_for_ZRU:				lngPack1 = lngCmd_ZRU;			break;					// команда для ЗРУ
						case gUstavki_Curr:				lngPack1 = lngUstavki_Curr;	break;					// Уставки_Текущ - управление БЭ
						case gRestData_from_BCU:	lngPack1 = lngRestData_BCU;	break;					// данные для восстановления из БВС 
						default:							{	bBadCmd1 = 1;	}																// недопустимая команда
						}
					}
					else	{	bBadCmd1 = 1;		}	
				}
			}
			else	{		bBadCmd1 = 1;		}																									// В буфере нет стартового байта
			
			// Установка индекса буфера на свободное место .......................
			if (bBadCmd1)	{	ind_pack1=0;	lngPack1 = Npack_Cmd-1;		}
			else	{
				if (ind_pack1 < lngPack1-1)		ind_pack1++;
				else 												{	ind_pack1=0;		bReqBCU[0] = 1;	}
			}
			
//		}
//		else 	{	MDR_UART1->RSR_ECR = 0x00;	}																					// Сброс флагов ошибок
		
		MDR_UART1->ICR |= UART_ICR_RXIC;																							// Сброс флага прерывания от приёмника UART1
		return;
	}
	
	// Передача ..............................................................
	if (MDR_UART1->RIS & UART_RIS_TXRIS)	{

		MDR_UART1->ICR |= UART_ICR_TXIC;																							// Сброс флага прерывания от передатчика UART1

//		ind_mas_trans1++;																															// Наращиваем счетчик байтов
//		if (ind_mas_trans1 >= BatchSize1)	{																						// Достигли максимума
//			//MDR_UART1->ICR |= UART_ICR_TXIC;																						// Сброс флага прерывания от передатчика UART1
//			while (MDR_UART1->FR & UART_FR_BUSY);																				// Ждем освобождения UARTa
//			//for(i=0; i < waitOEoff; i++)	ind_mas_trans1 = 0;															// Ожидание передачи последнего байта
//			RS_RECEIVE1																																		// Режим приёмника RS-485. Uart - OE=1
//			BatchSize1 = 0;		//bTxOn1 = 0;
//			//bTxOn1 = 0;																																	//RS_RECEIVE2;	 Выключить передатчик ОЕ1
//		}
//		else		{																																			// Завершаем передачу
//			while (MDR_UART1->FR & UART_FR_TXFF); 																			// 0x20 ждем, пока точно не опустошится буфер
//			//MDR_UART1->DR = *(p_ParRs1+ind_mas_trans1);																	// Загружаем очередной байт
//			MDR_UART1->DR = p_ParRs1[ind_mas_trans1];																		// Загружаем очередной байт
//		}
		

//		if (ind_mas_trans1 < BatchSize1)	{																						// Достигли максимума
//			while (MDR_UART1->FR & UART_FR_TXFF); 																			// 0x20 ждем, пока точно не опустошится буфер
//			//MDR_UART1->DR = p_ParRs1[ind_mas_trans1];																	// Загружаем очередной байт
//			MDR_UART1->DR = *(p_ParRs1+ind_mas_trans1);																	// Загружаем очередной байт
//		}
//		else		{																																			// Завершаем передачу
//			while (MDR_UART1->FR & UART_FR_BUSY);																				// Ждем освобождения UARTa
//			bTxOn1 = 0;																																	//RS_RECEIVE1;		 Выключить передатчик ОЕ1
////			MDR_UART1->ICR |= UART_ICR_TXIC;																						// Сброс флага прерывания от передатчика UART1
//		}
	}
}

//=================================================================================================================================================
void UART2_IRQHandler(void)
//=================================================================================================================================================
{		uint32_t wRIS;		unsigned char data;

	secUart2=0;
	wRIS = MDR_UART2->RIS;
	
	// Приём .................................................................
	if (wRIS & UART_RIS_RXRIS)	{
		
		data = MDR_UART2->DR;
//		if (!(MDR_UART1->RSR_ECR & (UART_RSR_ECR_FE | UART_RSR_ECR_PE | 
//																UART_RSR_ECR_BE | UART_RSR_ECR_OE)))							// Проверка на наличие ошибок приёма байта по UART
//		{	
		// Ошибок нет ......................................................
			pack2[ind_pack2] = data;																										// Запись байта данных в буфер

			bBadCmd2 = 0;
			
			if (pack2[0]==START_BYTE)	{																									// Начало пакета
			
				if (ind_pack2>3)	{																																// Число байт в буфере
						
					if ((pack2[1]==nMUK_BCU)&&(pack2[2]==Adr_RS_ZRU))		{										// 1 адрес БЦУ
								
						switch (pack2[3])	{															
						case gStat_ZRU:																												// телеметрия ЗРУ
						case gStat_AB_short:																									// Сост_АБ_Кратк_БЭ - краткая телеметрия БЭ
						case gStat_AB_Full:																										// Сост_АБ_Полн_БЭ – полная телеметрия БЭ
						case gUstavki_Tst:																										// контроль параметров (уставок) алгоритмов ЗРУ
						case gSaveData_to_BCU:																								// запоминаемые для восстановления данные в БВС 
						case gParamZRU:																												// частные параметры ЗРУ 
						case gTstLine:						lngPack2 = lngStat_ZRU;			break;					// проверка связи
						
						case gCmd_for_ZRU:				lngPack2 = lngCmd_ZRU;			break;					// команда для ЗРУ
						case gUstavki_Curr:				lngPack2 = lngUstavki_Curr;	break;					// Уставки_Текущ - управление БЭ
						case gRestData_from_BCU:	lngPack2 = lngRestData_BCU;	break;					// данные для восстановления из БВС 
						default:							{	bBadCmd2 = 1;	}																// недопустимая команда
						}
					}
					else	{	bBadCmd2 = 1;		}	
				}
			}
			else	{		bBadCmd2 = 1;		}																									// В буфере нет стартового байта
			
			// Установка индекса буфера на свободное место .......................
			if (bBadCmd2)	{	ind_pack2=0;	lngPack2 = Npack_Cmd-1;		}
			else	{
				if (ind_pack2 < lngPack2-1)		ind_pack2++;
				else 											{		ind_pack2=0;		bReqBCU[1] = 1;		}
			}
			
//		}
		// Ошибки ..........................................................
//		else 	{	
//			data = MDR_UART2->DR;
//			ind_pack2=0;	}
		
		MDR_UART2->ICR |= UART_ICR_RXIC;																							// Сброс флага прерывания от приёмника UART1
		return;
	}
	
	// Передача ..............................................................
	if (wRIS & UART_RIS_TXRIS)	{

		MDR_UART2->ICR |= UART_ICR_TXIC;																							// Сброс флага прерывания от передатчика UART1

//		ind_mas_trans2++;																															// Наращиваем счетчик байтов
//		if (ind_mas_trans2 >= BatchSize2)	{																						// Достигли максимума
//			//MDR_UART2->ICR |= UART_ICR_TXIC;																						// Сброс флага прерывания от передатчика UART1
//			while (MDR_UART2->FR & UART_FR_BUSY);																				// Ждем освобождения UARTa
//			RS_RECEIVE2																																	// Режим приёмника RS-485. Uart - OE=1
//			BatchSize2 = 0;
//		}
//		else		{																																			// Завершаем передачу
//			while (MDR_UART2->FR & UART_FR_TXFF); 																			// 0x20 ждем, пока точно не опустошится буфер
//			//MDR_UART2->DR = *(p_ParRs2+ind_mas_trans2);																	// Загружаем очередной байт
//			MDR_UART2->DR = p_ParRs2[ind_mas_trans2];																		// Загружаем очередной байт
//		}
	}
}


//=================================================================================================================================================
//void UART2_IRQHandler_021220(void)			// Текущий рабочий																// до 03 11 20
////=================================================================================================================================================
//{	unsigned char data;	//int i;
//	//uint8_t LengthPacks[8]= {0,lngStat_ZRU,	lngCmd_ZRU,	lngStat_AB_short,	lngStat_AB_Full, lngUstavki_Curr,	lngUstavki_Tst,	lngCmd_BCU};		// команда для ЗРУ от БЦУ (солнце/тень)
//	
//	secUart2 = 0;																																		// Контроль работоспособности 
//	
//	// Переполнение ..........................................................
//	if (MDR_UART2->RIS & UART_RIS_OERIS)	{
//		MDR_UART2->ICR |= UART_ICR_OEIC;																							// Сброс флага прерывания переполнения

//		if (!(MDR_UART1->RSR_ECR & (UART_RSR_ECR_FE | UART_RSR_ECR_PE | 
//																UART_RSR_ECR_BE | UART_RSR_ECR_OE)))							// Проверка на наличие ошибок приёма байта по UART
//		data = MDR_UART2->DR;																													// Запись байта данных в буфер

//		MDR_UART1->RSR_ECR = 0x0f;																										// Сброс флагов ошибок
//	}
//		
//	// Приём .................................................................
//	if (MDR_UART2->RIS & UART_RIS_RXRIS)	{
//			data = MDR_UART2->DR;																												// Запись байта данных в буфер

//		if (!(MDR_UART1->RSR_ECR & (UART_RSR_ECR_FE | UART_RSR_ECR_PE | 
//																UART_RSR_ECR_BE | UART_RSR_ECR_OE)))							// Проверка на наличие ошибок приёма байта по UART
//			{	MDR_UART1->RSR_ECR = 0x0f;	}																							// Сброс флагов ошибок

//			pack2[ind_pack2] = data;																										// Запись байта данных в буфер

//			if (pack2[0]==START_BYTE)	{																									// Начало пакета
//			
//				if (ind_pack2 > 3)	{																											// Число байт в буфере
//						
//					if ((pack2[1]==nMUK_BCU)&&(pack2[2]==Adr_RS_ZRU))		{										// 1 адрес БЦУ
//						
//						switch (pack2[3])	{															
//						case gStat_ZRU:				lngPack2 = lngStat_ZRU;				break;						// телеметрия ЗРУ
//						case gCmd_for_ZRU:				lngPack2 = lngCmd_ZRU;				break;						// команда для ЗРУ
//						case gStat_AB_short:	lngPack2 = lngStat_AB_short;	break;						// Сост_АБ_Кратк_БЭ - краткая телеметрия БЭ
//						case gStat_AB_Full:		lngPack2 = lngStat_AB_Full;		break;						// Сост_АБ_Полн_БЭ – полная телеметрия БЭ
//						case gUstavki_Curr:		lngPack2 = lngUstavki_Curr;		break;						// Уставки_Текущ - управление БЭ
//						case gUstavki_Tst:		lngPack2 = lngUstavki_Tst;		break;						// контроль параметров (уставок) алгоритмов ЗРУ
//						default:						{	lngPack2 = 45;	}															// bBadCmd2 = 1;недопустимая команда
//						}
//					}
//					else	{	ind_pack2=0;	lngPack2 = Npack_Cmd-1;
//									MDR_UART2->ICR |= UART_ICR_RXIC;	return;}										// bBadCmd2 = 1;
//				}
//			}
//			else	{	ind_pack2=0;	lngPack2 = Npack_Cmd-1;
//							MDR_UART2->ICR |= UART_ICR_RXIC;	return;}												// bBadCmd2 = 1;	В буфере нет стартового байта
//			
//			// Установка индекса буфера на свободное место .......................
//			//if (bBadCmd2)	{	ind_pack2=0;	lngPack2 = Npack_Cmd-1;		}
//			if (lngPack2 == 45)	{	ind_pack2=0;	lngPack2 = Npack_Cmd-1;		}
//			else	{
//				if (ind_pack2 < lngPack2-1)		ind_pack2++;
//				else {
//					ind_pack2 = 0;	bReqBCU[1] = 1;
//				}
//			}
//		//for (i=0;i<135;i++)	ind_pack2 = ind_pack2;
//		MDR_UART2->ICR |= UART_ICR_RXIC;																							// Сброс флага прерывания от приёмника UART1
//		return;	
//	}
//	
//	// Передача ..............................................................
//	if (MDR_UART2->RIS & UART_RIS_TXRIS)	{

//		ind_mas_trans2++;																															// увеличиваем индекс массива
//		if (ind_mas_trans2 < BatchSize2) { 																						// отправляем данные, пока они есть
//			while (MDR_UART2->FR & UART_FR_TXFF); 																			// 0x20 ждем, пока точно не опустошится буфер
//			MDR_UART2->DR = *(p_ParRs2+ind_mas_trans2);																	// отправляем очередной байт
//		}
//		else 																																					// когда данные закончились, необходимо дождаться, чтобы сдвиговый регистр УАРТ1 опустишился,
//		{																																							// только после этого можно и нужно выключать передатчик и считать, что массив отправлен
//			while (MDR_UART2->FR & UART_FR_BUSY);																				// ждем, пока данные не будут переданы
//			RS_RECEIVE2																																	// включаем режим приемника
//			MDR_UART2->ICR |= UART_ICR_TXIC;																							// Сброс флага прерывания от передатчика UART1
//		}		
//	}
//}

// *** End of IrqHandlers ***
