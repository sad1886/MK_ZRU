/*
  *****************************************************************************
  * @file    Main.c, программа управления ЗРУ от 30.05.2014										*
  * @author  ЗАО Орбита Карташов Ю.Д.																					*
  * @version V2.0.0																														*
  * @date    20.07.2015																												*
  *****************************************************************************/

/* Includes ------------------------------------------------------------------------------------------------------------*/
#include <stdlib.h>	
#include "MDR32F9x.h"
#include "init.h"																							// Файл с описанием процедур инициализации и аппаратных настроек
#include "init_bkp.h"																					// Файл с описанием процедур инициализации часов реального времени
#include "init_IWDG.h"																				// Файл с описанием процедур инициализации IWDG
#include "Can.h"
#include "Uart.h"
#include "Work.h"

//--------------------------- Общие ---------------------------------------------------------------------------------------
volatile unsigned char mode, mode_last, bmode_mod;						// Текущий режим работы контроллера
int iMUK_ZRU;
int iMUK_ZRU2, iMUK_ZRU3;

//--------------------------- Уставки от БЦУ ------------------------------------------------------------------------------
//
volatile float	dU_u,																					// Максимально допусстимый разброс давления ΔДу
								dP_u;																					// Максимально допусстимый разброс напряжений ΔUр 
volatile float	T_max;																				// Предел доп темп АБ на заряде
volatile float	Pvuz1;																				// Верхний уровень заряженности НВАБ
volatile float	Pvuz2;																				// Верхний уровень заряженности НВАБ
volatile float	Tvuz2;																				// Предельно допустимая температура НВАБ при уровне заряженности Рвуз8
volatile float	Pvir3;																				// Верхний уровень заряженности для восстановления НВАБ
volatile float	Tvir3;																				// Предельно допустимая температура АБi при уровне заряженности РВЫР3, ТВЫР3
volatile float	Pnuz;																					// Верхний уровень заряженности НВАБ
volatile float	PvuzCK;																				// РВУЗ СК		11	Предел ур заряж АБ на СК при заряде током не более (1,5±0,5) А		ufix8,z = 0.06 кгс/см2, x0 = 50 кгс/см2
volatile float	TnuzAB;																				// ТНУЗАБ			12	Предел доп темп АБ в начале подзаряда на СК		ufix8,z = 0.08 0C, x0 = 5 0C
volatile float	TvuzCK;																				// ТВУЗ СК		13	Предел доп темп АБ  при заряде током не более (1,5±0,5) А до ур заряж РВУЗСК		ufix8,z = 0.04 0C, x0 = 15 0C
volatile float	Pn;																						// Давление нижнего уровня заряженности НВАБ при ТВЦ,  (40-50)
volatile float	Pv;																						// Давление верхнего уровня заряженности НВАБ при ТВЦ, (46-54)
volatile float	Tn3;																					// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности Р<0,8Pн,  (42-45)
volatile float	Tn2;																					// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности 0,8Pн<Р<Pн, (28-32)
volatile float	Tn1;																					// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности Pн<Р<Pв, (18-22)
volatile float	Tk;																						// Tv Предельно допустимая температура НВАБ при включении компенсационного подзаряда (22-28)
volatile float	tVir;																					// Время выравнивания заряженности НВАБ при ТВЦ, час  
volatile float	Pvz;																					// Среднее давление НВА при ТВЦ после подзаряда(48 часов), кгс/см2
volatile float	Traz;																					// Предельно допустимая температура в ТВЦ НВАБ при разряде (42-45)
volatile float	P0;																						// Среднее давление НВА при прекращении  тестового разряда НВАБ (НАБ (76+-1В)
volatile float	tRazr;																				// Время разряда на разрядное сопротивление, час (10-50)
volatile float	kWtst;																				// Коэфф. для расчёта текущего значения энергоёмкости
volatile float	tP_TVC;																				// от 10,0 до 70,0 мин	40	Время принудительного разряда в ТВЦ,ТВЦ	аналоговый, 

volatile float	PvzRas,																				// Расчетное среднее давление в АК для АБ в режиме ТВЦ, в конце компенсационного заряда
								P0_Ras;																				// Рассчитанное среднее давление в АК в режиме ТВЦ при прекращении разряда АБ
volatile float	time_Razr;

volatile unsigned char nBadAkBE[5];														// Номера отказавших аккумуляторов от БЭ
volatile unsigned char nBadAk[5];															// Номера отказавших аккумуляторов от БЦУ
volatile unsigned char nGudAk = nAllAE;												// Число исправных аккумуляторов 

//-------------------------------------------------------------------------------------------------------------------------
volatile float Pu[nUst] = {Pvuz1_def,Pvuz2_def,Pvir3_def};					// Уставки давления. Рвыр = Pu[nUst]
volatile float Tu[nUst]	= {Tvuz1_def,Tvuz2_def,Tvir3_def};					// Уставки температуры

volatile unsigned char iUst;																				// Индекс текущей уставки 0..nUst-1

volatile float	curW_zar,																						// Текущ заряд НВАБ		9
								W_raz,																							// Энергоёмк
								C_raz,																							// Ёмкость
								W_podzar,																						// Энергоёмк Подзаряда на СК
								Po_ras,																							// Средн_P_разр
								Pvz_ras;																						// Средн_P_подзар,		13

unsigned char stat1[3]={0,0,0},
							stat2[3]={0,0,0},
							stat3[3]={0,0,0},
							stat4[3]={0,0,0};		
unsigned char sync_AND_2 = 0; //служебная переменная для хранения результатов синхронизации, логические И двух других соседей (нужно оба других) 
unsigned char sync_OR_1 = 0; //служебная переменная для хранения результатов синхронизации, логические ИЛИ двух других соседей (достаточно хотя бы одного другого)

volatile float	P = 40,																							// [0] среднее значение ДД 
								dP,																									// [1] максимальное отклонение ДД
								T = 30,																							// [4] среднее значение ДТ 
 								Uab, Uab_old,																				// [30] значение напряжения АБ
								UsrAk,																							// [0] среднее значение напряжения АЭ
								dUak,																								// Разница напряжений макс и мин значений из 72 АК, В, было[1] максимальное отклонение напряжения АЭ ΔUрез
								Umin_ak=0.7;																				// минимальное значение напряжения АЭ

volatile unsigned char StepAlgortmRazr, StepAlgortmZar,
											 StepNext;
//volatile unsigned int Errors;																				// 
volatile int DataOk;																								// Признак недостоверных данных телеметрии АБ БЭ
unsigned char i;																										// 
//unsigned char errT, errP;																						// Байт отказов датчиков температуры, давления

//-------------------------------------------------------------------------------------------------------------------------
extern volatile unsigned char bNoWrkCAN;														// 1 - CAN работает, 0 - CAN не работает

extern volatile unsigned char bRestData, vRestData;									// 1 - восстановить данные

extern volatile unsigned char bSendStatus;													// послать байт состояния МУК ЗРУ

extern uint32_t bitNotZar, bitNotRaz;																// Состояние проводных линий

//--------------------------- ADC переменные ------------------------------------------------------------------------------
extern volatile int iadc;																						// Указатель на текущий канал измерения АЦП
extern volatile int iReadAdc;																				// Счётчик измерений текущего канала АЦП
extern volatile int NumReadAdc;																			// Число достоверных значений измерения АЦП текущего канала
extern volatile uint32_t cntReadADC;																// Счётчик циклов чтения АЦП
extern float Vals_ZRU[nParams];																			// Массив реальных измеренных значений параметров ЗРУ
extern float tVals_ZRU[11];																					// Массив тестовых значений параметров ЗРУ

extern float aI_razr, aI_razrOld, aI_zar;
extern float aU_zru, aU_zru_Old;;	

extern float cUsm[3][2];																						// (В) Измеренное напряжение средней точкой (смещение) для МУКов
extern float AUcc[3];																								// (В) Опорное напряжение АЦП МУК1, МУК2, МУК3
extern const float Uref[3];																					// (В) Прецизионное напряжение АЦП МУК1, МУК2, МУК3 на PD0
unsigned char OkResult;																							// Результат получен
extern volatile unsigned char OkDataADC;														// Значение канала АЦП готово
extern uint32_t summa;																							// Сумма значений
extern float Uadc;																									// Значения, прочитанные из АЦП
extern volatile uint32_t ADC1_REG_GO;																// #define ADC1_CFG_REG_GO  ((uint32_t)0x00000002)

//--------------------------- Преобразованные данные из БЭ ----------------------------------------------------------------
float flVdatchWrk[nfVdatch];
float flV_ABWrk[nfV_AB];			

int iVdatchWrk[niVdatch];
int iV_ABWrk[niV_AB];

extern union uBytesFloat16	fVdatch[nfVdatch];
extern int 									iVdatch[niVdatch];

extern union uBytesFloat16 	fV_AB[nfV_AB];
extern int 									iV_AB[niV_AB];

extern volatile unsigned char err_BE;

//--------------------------- CAN переменные ------------------------------------------------------------------------------
volatile unsigned char updateD1;																					// Флаги обновления данных от МУКов БЭ пакет №1
volatile unsigned char updateD2;																					// Флаги обновления данных от МУКов БЭ пакет №2
volatile unsigned char No_updateD;																				// Флаги нет обновления данных от МУКов БЭ

extern volatile union uBytes64 Reciev_CanErrors;													// Телеметрия отказов БЭ
extern volatile union uBytes64 Reciev_CanDatch[nFrameDatchCAN];						// Телеметрия датчиков
extern volatile union uBytes64 Reciev_CanAB[nFrameABCAN];									// Телеметрия АБ

volatile unsigned char bReciev_CanErrors[nMUKBE];													// Флаги принятых фреймов телеметрии отказов БЭ
volatile unsigned char bReciev_CanDatch[nMUKBE][nFrameDatchCAN];					// Флаги принятых фреймов телеметрии датчиков
volatile unsigned char bReciev_CanAB[nMUKBE][nFrameABCAN];								// Флаги принятых фреймов телеметрии АБ

int nfRec_CanDatch1, nfRec_CanDatch2;																			// Маска принятых фреймов телеметрии датчиков 0x3ff
int nfRec_CanAK1, nfRec_CanAK2;																						// Маска принятых фреймов телеметрии АК 0xfffff

extern volatile unsigned char bRunCmdCAN, CurrentCmd, CurrentDlc;					// Флаг отправки команды по CAN
extern uint32_t ResultCAN;																								// Результат выполнения команд Вкл_РС, Откл_РС, Байт ошибок БЭ  по CAN

extern int cntSec_noCAN[6];																								// счётчики секунд неработоспособности CAN (1..6)

//--------------------------- RS485 переменные ----------------------------------------------------------------------------
//UART1 . . . . . . . . . . . . . . . . . . . . . . . . . . .
extern unsigned char pack1[Npack_Cmd];																		// Буфер приёма пакетов
extern volatile int ind_pack1;																						// Индекс приёмного буфера
extern volatile int lngPack1;																							// Длина принятого пакета

extern volatile int BatchSize1;																						// Размер передаваемого пакета в байтах
extern volatile int ind_mas_trans1;																				// Индекс передаваемого пакета
extern unsigned char * p_ParRs1;
extern volatile unsigned short checksumCalc1, checksumIn1;

//UART2 . . . . . . . . . . . . . . . . . . . . . . . . . . .
extern unsigned char pack2[Npack_Cmd];																		// Буфер приёма пакетов
extern volatile int ind_pack2;																						// Индекс приёмного буфера
extern volatile int lngPack2;																							// Длина принятого пакета

extern volatile int BatchSize2;																						// Размер пакета в байтах
extern volatile int ind_mas_trans2;
extern unsigned char * p_ParRs2;
extern volatile unsigned short checksumCalc2, checksumIn2;

//.......................................................................................................................
extern unsigned short checksumCalc, checksumIn;

extern int indsData_p[nParams];																		// Номера индексов в блоке данных пакета телеметрии протокола RS485

extern unsigned char PackRs1[lngPackRs1];													// Ответ на пакет 1
extern unsigned char PackRs2[lngPackRs2];													// Ответ на пакет 2 или 7
extern unsigned char PackRs3[lngPackRs3];													// Ответ на пакет 3
extern unsigned char PackRs4[lngPackRs4];													// Ответ на пакет 4
extern unsigned char PackRs5[lngPackRs5];													// Ответ на пакет 5
extern unsigned char PackRs6[lngPackRs6];													// Ответ на пакет 6
extern unsigned char PackRs7[lngPackRs7];													// Ответ на пакет 7
extern unsigned char PackRs8[lngPackRs8];													// Ответ на пакет 8
extern unsigned char PackRs9[lngPackRs9];													// Ответ на пакет 9
extern unsigned char PackRs10[lngPackRs10];												// Ответ на пакет 10

extern unsigned char * p_ParRs;
extern unsigned char * p_InPack;

extern volatile unsigned char BatchSize;													// Размер пакета в байтах

extern float z_p [nParams+5];																			// z – цена (вес) младшего разряда;
extern float x0_p[nParams+5];																			// x0 – сдвиг нуля
	
extern float z_p3 [lngPackRs3-6];																	// z – цена (вес) младшего разряда;
extern float x0_p3[lngPackRs3-6];																	// x0 – сдвиг нуля

extern float z_p4 [lngPackRs4-6];																	// z – цена (вес) младшего разряда;
extern float x0_p4[lngPackRs4-6];																	// x0 – сдвиг нуля

extern float z_p5 [lngUstavki_Curr-6];														// z – цена (вес) младшего разряда;
extern float x0_p5[lngUstavki_Curr-6];														// x0 – сдвиг нуля

extern float z_p7 [lngPackRs7-7];																	// z – цена (вес) младшего разряда;
extern float x0_p7[lngPackRs7-7];																	// x0 – сдвиг нуля

extern float z_p9 [lngPackRs9-6];																	// z – цена (вес) младшего разряда;
extern float x0_p9[lngPackRs9-6];																	// x0 – сдвиг нуля

int cnt=0, cntTimeOut=0;

extern unsigned char bReqBCU[2];																		// Флаг: поступил запрос (команда) от БЦУ

unsigned char bUstavkiBCU, bGetData;

//--------------------------- Time переменные ---------------------------------------------------------------------------
extern tTime	sTime, sTimeCurnt;
extern int NewDay;
volatile unsigned char mCount5Main, mCountSecMain;								// Счётчик 5 мин для измерения температуры АБ, счётчик секунд
volatile unsigned char mCount5, mCountSec;												// Счётчик 5 мин для паузы
//volatile unsigned char sCount5, sWait5;														// Счётчик 5 сек для расчёта C и W
volatile unsigned char sCount20;																	// Счётчик 20 сек для задержки повтора 3 раза алгоритма заряда

volatile unsigned char bTimeOutCmd;																// Флаг Время ожидания ответа результата от БЭ команды 
volatile unsigned char bOneSec;																		// Флаг 1 секунда
volatile unsigned char bPauza5, bPauza20, bPauza5m;								// Флаги включения пауз 5 сек, 20 сек, 5 мин
volatile unsigned char nSutok;																		// Число суток с начала счёта часов в ТВЦ при разряде РС
unsigned char	secUart1, secUart2, NoWrkUart1;												// Счётчик секунд Uart1, флаг достижения двух секунд
unsigned char	secTimeOutCmd;

unsigned char	secCAN_err;

unsigned char EndTVC = 0; 																				// Флаг того, что начался процесс окончания ТВЦ (по команде или по окончанию алгоритма)
unsigned char bPauza, bPauza_R, bPauza_TVC,
							bCount_2h;																					// Флаг общей паузы, флаг - ждать 2 часа в ТВЦ
int sCount, sCount_R, sCount_2h;																	// Счётчик секунд общей паузы, Счётчик секунд для 2-х часов ожидания в ТВЦ (sCount_2h стала вспомогательной и используется в разных местах) 
int LimsCount, LimsCount_R;																				// Предельное (конечное) значение для счётчика секунд общей паузы

//........... T V C ...........
volatile unsigned char  StepAlgortm,
												bFlag,
												cntRazr, statTVC, 
												stapTVC; //эта переменная пока ни на что не влияет
int tstatTVC;																											// Текущее время от начала этапа 
//int tvcTimeRS;																									// Время разряда АБ в ТВЦ на разрядной нагрузке (РС), какая-то ненужная переменная

float calc_dt = (float)dt5/(60*60); 															//дельта времени, необходимая для расчета W и C, может меняться в зависимости от паузы между измерениями, по умолчанию соответствует дельте в 5 секунд
float calc_dt5 = (float)dt5/(60*60);															// 5 сек, типовое значение дельты времени 
float calc_dt20 = (float)dt20/(60*60);														// 20 сек, редкое значение дельты времени


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

int add_nbuf;
int tVkl_ZRU=0;

extern uint32_t Result;

unsigned char AddSec;

unsigned char set100ms, yes100ms;

unsigned char repeatOne;
//--------------------------- для алгоритма переменные ------------------------------------------------------------------------------
unsigned char mode_Razryad = 0; //если не 0, значит мы находимся в режиме разряда (ток разряда больше нуля)
unsigned char mode_Zaryad = 0; //если не 0, значит мы находимся в режиме заряда (ток заряда больше нуля) 

//-------------------------------------------------------------------------------------------------------------------------
// Подготовка мажоритированных данных для телеметрии
unsigned int MajorStatZRU(unsigned char * stat);

//-------------------------------------------------------------------------------------------------------------------------
void fDataConvert_(void)
{	int j,adj,ak,fr;
/*
1	1-2	среднее значение ДД 								float16	кг/см2
	3-4	максимальное отклонение ДД					float16	
	6-7	максимальное значение ДД						float16	кг/см2
2	1-2	минимальное значение ДД							float16	кг/см2
	4-5	среднее значение ДТ 								float16	°C
	6-7	максимальное отклонение ДТ					float16	
3	1-2	максимальное значение ДТ						float16	°C
	4-5	минимальное значение ДТ							float16	°C
	7-8	значение ДД1												float16	кг/см2
4	1-2	значение ДД2												float16	кг/см2
	3-4	значение ДД3 												float16	кг/см2
	5-6	значение ДД4												float16	кг/см2
	7-8	значение ДД5												float16	кг/см2
5	1-2	значение ДТ1												float16	°C
	3-4	значение ДТ2												float16	°C
	5-6	значение ДТ3												float16	°C
	7-8	значение ДТ4												float16	°C
6	1-2	значение ДТ5												float16	°C
	3-4	напряжение на ДД1										float16	В
	5-6	напряжение на ДД2										float16	В
	7-8	напряжение на ДД3										float16	В
7	1-2	напряжение на ДД4										float16	В
	3-4	напряжение на ДД5										float16	В
	5-6	напряжение на ДТ1										float16	В
	7-8	напряжение на ДТ2										float16	В
8	1-2	напряжение на ДТ3										float16	В
	3-4	напряжение на ДТ4										float16	В
	5-6	напряжение на ДТ5										float16	В
	7-8	напряжение АБ1											float16	В
9	1-2	напряжение АБ2											float16	В
	3-4	опорное напряжение ДД								float16	В
	5-6	опорное напряжение ДТ								float16	В
	7-8	общий ДД														float16	В
10	1-2	общий ДТ													float16	В
	4-5	температура МУК											float16	°C
	
	5	номер ДД с максимальным отклонением		char	
	8	номер ДД с максимальным значением			char	
	3	номер ДД с минимальным значением			char	
	8	номер ДТ с максимальным отклонением		char	
	3	номер ДТ с максимальным значением			char	
	6	номер ДТ с минимальным значением			char	
	3	состояние РС НВАБ											char **	
*/
	
	//.............................................................Телеметрия датчиков
		fr = 0;		adj=0;
		for (ak=0; ak < nfVdatch; ak++)	{
			for (j=0; j<2; j++)	{
				fVdatch[ak].b[j] = Reciev_CanDatch[fr].b[j+adj];								// 
			}
			switch (fr)		{
			case 0:	if (ak==0) 		adj = 2;
							if (ak==1) 		adj = 5;
							if (ak==2) {	adj = 0;	fr++;	}			break;
			case 1:	if (ak==3) 		adj = 3;
							if (ak==4)	 	adj = 5;
							if (ak==5) {	adj = 0;	fr++;	}			break;
			case 2:	if (ak==6) 		adj = 3;
							if (ak==7) 		adj = 6;
							if (ak==8) {	adj = 0;	fr++;	}			break;
			default:	
				if (adj<6) adj += 2;	else	{	adj=0;	fr++;	}
			}
		}
		iVdatch[0] = Reciev_CanDatch[0].b[4];																// номер ДД с максимальным отклонением
		iVdatch[1] = Reciev_CanDatch[0].b[7];																// номер ДД с максимальным значением
		iVdatch[2] = Reciev_CanDatch[1].b[2];																// номер ДД с минимальным значением
		iVdatch[3] = Reciev_CanDatch[1].b[7];																// номер ДТ с максимальным отклонением
		iVdatch[4] = Reciev_CanDatch[2].b[2];																// номер ДТ с максимальным значением
		iVdatch[5] = Reciev_CanDatch[2].b[5];																// номер ДТ с минимальным значением
		iVdatch[6] = Reciev_CanDatch[9].b[2];																// состояние РС НВАБ
		
		if (iVdatch[6])	stat1[iMUK_ZRU] |= bPC;
		else						stat1[iMUK_ZRU] &= ~bPC;
	
	//...........................................................Телеметрия АБ
		fr = 0;		adj=0;
		for (ak=0; ak < nfV_AB; ak++)	{
			for (j=0; j<2; j++)	{
				fV_AB[ak].b[j] = Reciev_CanAB[fr].b[j+adj];											// 
			}
			switch (fr)		{
			case 0:	if (ak==0) 	adj = 2;
							if (ak==1) 	adj = 6;
							if (ak==2) {	adj = 0;	fr++;	}			break;
			default:	
				if (adj<6) adj += 2;	else	{	adj=0;	fr++;	}
			}
		}
		iV_AB[0] = Reciev_CanAB[0].b[4];																		// 5	номер АЭ с максимальным отклонением
		iV_AB[1] = Reciev_CanAB[0].b[5];																		// 6	номер АЭ с максим. значением напряжения
		iV_AB[2] = Reciev_CanAB[19].b[4];																		// 5	номер АЭ с миним. значением напряжения

	//...........................................................Телеметрия отказов БЭ
		err_BE = Reciev_CanErrors.b[0];																			// Байт ошибок аппаратуры
		for(i=0; i < 5; i++)		{
			nBadAkBE[i] = Reciev_CanErrors.b[i+3];														// Номера отказавших аккумуляторов БЭ
			//if (nBadAkBE[i])	cntBadAk++;
		}

}

//-------------------------------------------------------------------------------------------------------------------------
// Получение параметра из телеметрии БЭ, установка флагов недостоверных значений
//-------------------------------------------------------------------------------------------------------------------------
float GetParam(char bArr, int iV)
{	float fres;
	if (bArr)	{		fres = fVdatch[iV].Fdata;	}
	else			{		fres = fV_AB[iV].Fdata;		}	
	return fres; 
}

//-------------------------------------------------------------------------------------------------------------------------
// Получение рабочих параметров
//-------------------------------------------------------------------------------------------------------------------------
void GetData(void)
{	float Uab1, Uab2, mUab;		//unsigned char err;		
	P   = GetParam(1, 0);																									// [0] среднее значение ДД 
	//dP  = GetParam(1, 1);																									// [1] максимальное отклонение ДД
	dP  = GetParam(1, 2)-GetParam(1, 3);																	// [1] разница между максимальным и минимальнымзначением ДД
	T   = GetParam(1, 4);																									// [4] среднее значение ДТ 
	
	// Выбор значения напряжения АБ
	Uab1 = GetParam(1, 28);																								// [28] значение напряжения АБ1
	Uab2 = GetParam(1, 29);																								// [29] значение напряжения АБ2
	if (Uab1 > Uab2)	 mUab = Uab1;		else	mUab = Uab2;
	Uab = 0;
	for (i=5; i < 77; i++)	{	Uab += GetParam(0, i);	}										// Сумма АК
	
	if ((abs_f(Uab1-Uab) < Uab*0.05)&&(abs_f(Uab2-Uab) < Uab*0.05))	{
		if (abs_f(Uab1-Uab2) <= mUab*0.01)				Uab = (Uab1+Uab2)/2;
		else {
			if (abs_f(Uab1-Uab) <= abs_f(Uab2-Uab))	Uab = Uab1;								// 
			else																		Uab = Uab2;
		}
	}	
	else {
		if (abs_f(Uab1-Uab) <= Uab*0.05)					Uab = Uab1;								// 
		if (abs_f(Uab2-Uab) <= Uab*0.05)					Uab = Uab2;								// 
	}

	UsrAk = GetParam(0, 0);																								// [0] среднее значение напряжения АЭ
//	dUak  = GetParam(0, 1);																								// [1] максимальное отклонение напряжения АЭ
	Umin_ak = GetParam(0, 3);																							// [3] минимальное значение напряжения АЭ

	//...........................................................Отказы БЭ
//	err = err_BE;
//	errT = 0x10 & err;																										// Отказ датчиков температуры
//	errP = 0x20 & err;																										// Отказ датчиков давления
}

//-------------------------------------------------------------------------------------------------------------------------
void Wait(int sWait)
{	int j;
	for (i=1;i<=sWait;i++)		{	j=j;	}																		// 
}

//-------------------------------------------------------------------------------------------------------------------------
void Init_Param(void)	// 
{	
	dU_u   = dU_u_def;																		 								// Максимально допусстимый разброс давления ΔДу
	dP_u   = dP_u_def;																										// Максимально допусстимый разброс напряжений ΔUр 
	T_max  = T_max_def;																										// Предел доп темп АБ на заряде

	Pvuz1  = Pvuz1_def;																										// Верхний уровень заряженности НВАБ
	Pvuz2  = Pvuz2_def;																										// Верхний уровень заряженности НВАБ
	Tvuz2  = Tvuz2_def;																										// Предельно допустимая температура НВАБ при уровне заряженности Рвуз8
	Pvir3  = Pvir3_def;																										// Верхний уровень заряженности для восстановления НВАБ
	Tvir3  = Tvir3_def;																										// Предельно допустимая температура АБi при уровне заряженности РВЫР3, ТВЫР3

	Pnuz   = Pnuz_def;																										// Верхний уровень заряженности НВАБ
	PvuzCK = PvuzCK_def;																									// РВУЗ СК		11	Предел ур заряж АБ на СК при заряде током не более (1,5±0,5) А		ufix8,z = 0.06 кгс/см2, x0 = 50 кгс/см2
	TnuzAB = TnuzAB_def;																									// ТНУЗАБ			12	Предел доп темп АБ в начале подзаряда на СК		ufix8,z = 0.08 0C, x0 = 5 0C
	TvuzCK = TvuzCK_def;																									// ТВУЗ СК		13	Предел доп темп АБ  при заряде током не более (1,5±0,5) А до ур заряж РВУЗСК		ufix8,z = 0.04 0C, x0 = 15 0C
	Pn  	 = Pn_def;																											// Давление нижнего уровня заряженности НВАБ при ТВЦ,  (40-50)
	Pv  	 = Pv_def;																											// Давление верхнего уровня заряженности НВАБ при ТВЦ, (46-54)
	Tn3 	 = Tn3_def;																											// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности Р<0,8Pн,  (42-45)
	Tn2 	 = Tn2_def;																											// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности 0,8Pн<Р<Pн, (28-32)
	Tn1 	 = Tn1_def;																											// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности Pн<Р<Pв, (18-22)
	Tk  	 = Tk_def;																											// Tv Предельно допустимая температура НВАБ при включении компенсационного подзаряда (22-28)
	tVir 	 = tVir_def;																										// Время выравнивания заряженности НВАБ при ТВЦ, час  
	Pvz  	 = Pvz_def;																											// Среднее давление НВА при ТВЦ после подзаряда(48 часов), кгс/см2
	Traz 	 = Traz_def;																										// Предельно допустимая температура в ТВЦ НВАБ при разряде (42-45)
	P0   	 = P0_def;																											// Среднее давление НВА при прекращении  тестового разряда НВАБ (НАБ (76+-1В)
	tRazr  = tRazr_def;																										// Время разряда на разрядное сопротивление, час (10-50)
	kWtst  = kWtst_def;																										// Коэфф. для расчёта текущего значения энергоёмкости
	tP_TVC = tP_TVC_def;																									// от 10,0 до 70,0 мин	40	Время принудительного разряда в ТВЦ,ТВЦ	аналоговый, 
}

/*
//-------------------------------------------------------------------------------------------------------------------------
void SetParamTest_NVAB (void)														// _Т_В_Ц___Н_В_А_Б_ 
{
	switch (StepAlgortm)																						// Обработчик состояний алгоритма заряда
	{
	// лист 34
	// ========== Ожидание разряда АБ .........................4
	//case bWaitRazryd:		P	 = 30;	T = 40;										break;
		
	// ========== Включение Заряда ............................5
	//case bVkl_Tst_Zarayd:																		break;
		
	// .......... Ожидание включения Тест Заряда ..............6
	case bWaitVkl_Tst_Zarayd:		aI_zar = 9;		aI_razr = 0;	break;
		
	// ========== Заряжаем батарею ............................7
	//case bVkl_Tst_Zarayd_On:		P	 = 42;	T = 40;						break;
		
	// ========== Отключаем Заряд .............................8
	//case bVkl_ZaprZarayd:																		break;
	
	// .......... Ожидание Отключения заряда ..................9
	case bWaitOtkl_Zarayd:	aI_zar = 2;		aI_razr = 0;			break;
		
	// лист 35
	// ========== Включение тестового разряда .................10
	//case bVkl_Test_Razr:																		break;
	
	// .......... Ожидание включения разряда ..................11
	//case bWaitVkl_Test_Razr:		aI_zar = 0;		aI_razr = 20;	break;
	
	// ========== Разряд АБ и вычисление C и W ................12
	//case bTst_T_Razryda:	P	 = 42;	T = 55;									break;
		
	// ========== Отключаем разряд ............................13
	//case bWaitOtkl_Razrayd:		P	 = 42;	T = 55;
	//													aI_zar = 0;		aI_razr = 2;		break;
	 
	// ========== Проверка температуры АБ .....................14
	//case bTst_T_NVAB:		P	 = 42;	T = 30;										break;   
		
	// лист 36
	// ========== Повторить разряд АБ .........................15
	//case bRepeatRazr:																				break;   

	// лист 37
	// ========== Ждём 2 часа .................................16
	//case bWait_2:																						break;
		
	// .......... Ожидание Вкл РС .............................17
	//case bWaitVkl_RS:		bPC[iMUK_ZRU] = 1;								break;
		
	// ========== Разряд АБ с РС не более 30 час ..............18
	//case bCountTimeRazr:	if (sCount_2h >= 10)	bPC[iMUK_ZRU] = 0;	break;
		
	// .......... Отключаем РС ................................19
	//case bWaitOtkl_RS:																			break;

	// ========== Включение КОМП заряда АБ  ...................20
	//case bVklKomp:																					break;
	
	// .......... Ожидание Вкл_КОМП ...........................21
	case bWaitVklKomp:		aI_zar = 4;		aI_razr = 0;				break;
		
	// ========== Заряд АБ компенсационным током ..............22
	//case bZarydComp:																				break;
		
	// лист 38
	// .......... Ожидание Откл_КОМП ..........................23
	case bWaitOtklKomp2:		aI_zar = 1;		aI_razr = 0;		break;
		
	// ========== Заряжаем батарею максимальным током .........24
	//case bVkl_Tst_Zarayd_On2:		P	 = 42;	T = 55;						break;
		
	// ========== Включение КОМП заряда АБ  ...................25
	//case bVklKomp2:																					break;
	
	// .......... Ожидание Вкл_КОМП ...........................26
	case bWaitVklKomp2:		aI_zar = 1;		aI_razr = 0;				break;
		
	// лист 39
	// ========== Заряжаем батарею компенсационным током "КОМП"27
	//case bZarydComp2:	P	 = 42;	T = 24;		tVir = 10;				break;
					
	// лист 41
	// ========== Принудительный заряд до рабочей уставки P2 ..28
	//case bVkl_Tst_Zarayd9:																	break;
			
	// .......... Ожидание включения Тест Заряда ..............29
	case bWaitVkl_Tst_Zar9:		aI_zar = 1;		aI_razr = 0;	break;
		
	// ========== Заряжаем батарею ............................30
//	case bVkl_Tst_Zar_On9:	P = Pvuz2;											break;
		
	// .......... Ожидание Отключения заряда ..................31
//	case bWaitOtkl_Tst_Zar9:																break;
		
	// ========== Выбор уставки ...............................32
//	case bViborUst:																					break;

	// ========== Завершение алгоритма ........................33
//	case bInitEnd_Alg_TVC:		
		
	// .......... Конец алгоритма .............................
//	case bEnd_Alg_TVC:																			break;
	
	} //end of switch (StatusZarRazr)

}
*/

//-------------------------------------------------------------------------------------------------------------------------
int hCount (void)
{	int h;
	if (NewDay) {	nSutok++; NewDay=0;	}

	if ((sTime.hour > sTimeCurnt.hour)&&(!nSutok))	{
		h = sTime.hour - sTimeCurnt.hour;	}
	else	{	
		h =  24 - sTimeCurnt.hour + 24*(nSutok-1) + sTime.hour;	}
	return h;
}	

//-------------Расчёт C W--------------------------------------------------------------------------------------------------
void Calculation (void)
{
	C_raz += ((aI_razrOld + aI_razr)/2)*calc_dt;																// Расчёт	C = C + Iab*dt, 		dt=5сек
	W_raz += ((aI_razrOld + aI_razr)/2)*calc_dt*((Uab_old + Uab)/2);						// Расчёт W = W + Iab*dt*Uab
	Uab_old = Uab;	aI_razrOld = aI_razr;
}

//-------------Расчёт C W при потерея обоих линий CAN--------------------------------------------------------------------------------------------------
void Calculation_noCAN (void)
{
	C_raz += ((aI_razrOld + aI_razr)/2)*calc_dt;																// Расчёт	C = C + Iab*dt, 		dt=5сек
	W_raz += ((aI_razrOld + aI_razr)/2)*calc_dt*((aU_zru_Old + aU_zru)/2);			// Расчёт W = W + Iab*dt*Uab
	aU_zru_Old = aU_zru;		aI_razrOld = aI_razr;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Test_NVAB (void)														/* _Т_В_Ц___Н_В_А_Б_ */
{		int ii;

	//SetParamTest_NVAB();
	switch (StepAlgortm)																									// Обработчик состояний алгоритма заряда
	{
	// .......... Инициализация ТВЦ .........................................................................................1
	case bInit_TVC:
		//сброс всех аварийных сообщений
		stat2[iMUK_ZRU] = 0;
		stat3[iMUK_ZRU] &= ~(errNoOgrTokRazr|errNoOgrTokZar|errPrevDopustT);
		//------------------------------
		statTVC = 0;	stapTVC = 0;

	// лист 35
	// ========== Отключение КОМП ...........................................................................................2
	case bOtklKomp:
		
		pOtkl_KOMP();
		LimsCount = vsCount5;	sCount=0;		bPauza=1;													// Активация паузы 5 сек
		StepAlgortm = bWaitOtklKomp;
		break;
	
	// .......... Ожидание отключения КОМП Заряда ...........................................................................3
	case bWaitOtklKomp:		

		if (!bPauza) {
			//if	(aI_zar <= aIkomp)	stat2[iMUK_ZRU] |= errNoOtklCompZar;		// Собщение "Не отключился КОМП Заряд АБ"=1
			StepAlgortm = bWaitRazryd;	stapTVC = 1;
		}
		break;
		
	// ========== Ожидание разряда АБ .......................................................................................4
	case bWaitRazryd:
		
		if (!bPauza) {																											// Пауза 1 мин bPauza1m
			if	(((P <= 0.8*Pn) && (T <= Tn3))||															// (P<=0.8Pn && T<=Tn3)||
					 ((0.8*Pn <= P) && (P <= Pn) && (T <= Tn2))||									// (0.8Pn<=P<=Pn && T<=Tn2)||
					 ((Pn <= P) && (P <= Pv) && (T <= Tn1)))	{										// (Pn<=P<=Pv && T<=Tn1)
						 					 
				pVkl_Zapr_Razrayd();
				pVkl_Zapr_Zarayd();			
				for (ii=0; ii < 500; ii++)	tstatTVC =0;			
				StepAlgortm = bVkl_Tst_Zarayd;							 
		  }
			else	{																														
				LimsCount = vmCount1;		sCount=0;	bPauza=1;											// Сброс счётчика 1 мин, включене паузы 1 мин
				StepAlgortm = bWaitRazryd;
			}
		}
		break;   
		
	// ========== Включение Заряда ..........................................................................................5
	case bVkl_Tst_Zarayd:		
		
		statTVC = 1;		tstatTVC =0;																				// Этап проведения ТВЦ. Время начала этапа.	
		pVkl_Test_Zarayd();	
		pOtkl_Zapr_Zarayd();
		LimsCount = vsCount20;	sCount=0;		bPauza=1;												// Активация паузы 20 сек
		StepAlgortm = bWaitVkl_Tst_Zarayd;
		break;
		
	// .......... Ожидание включения Тест Заряда ............................................................................6
	case bWaitVkl_Tst_Zarayd:		

		if (!bPauza) {
			if	(aI_zar > aIporog)	{	stat2[iMUK_ZRU] &= ~errNoVklZar;					// Собщение "Не включился Заряд АБ"=0
			//if	(aI_zar < 50)	{	stat2[iMUK_ZRU] &= ~errNoVklZar;							// Отладочная заглушка, Собщение "Не включился Заряд АБ"=0
				StepAlgortm = bVkl_Tst_Zarayd_On;
			}
			else	{									stat2[iMUK_ZRU] |= errNoVklZar;						// Собщение "Не включился Заряд АБ"=1
				StepAlgortm = bVkl_Tst_Zarayd;
			}	
		}
		break;
		
	// ========== Заряжаем батарею ..........................................................................................7
	case bVkl_Tst_Zarayd_On:			

		if (!bPauza) {
			if	((T >= Tn3)||																									// (T>Tn3)||
					 ((0.8*Pn <= P) && (P <= Pn) && (T >= Tn2))||									// (0.8Pn<=P<=Pn && T>=Tn2)||
					 ((Pn <= P) && (P <= Pv) && (T >= Tn1))||											// (Pn<=P<=Pv && T>=Tn1)
					 (P >= Pv) ||
					 ((stat3[iMUK_ZRU2] & bZaprZar)&&															// включен запрет Разряда
					 ( stat3[iMUK_ZRU3] & bZaprZar))
					)
			{			 
				if (P < Pv)	{	stat3[iMUK_ZRU] |= errPrevDopustT;								// "Превышение допустимой температуры НВАБ"
        //if (P < 30)	{	stat3[iMUK_ZRU] |= errPrevDopustT;								// Отладочная заглушка, "Превышение допустимой температуры НВАБ"
					StepAlgortm = bInitEnd_Alg_TVC;																// ******** Окончание по превышению температуры
				}
				else	{				
					StepAlgortm = bVkl_ZaprZarayd;																// Для первого заряда StepNext = bOtkl_Zarayd. Для второго StepNext = bCmp_T_Tv_P_Pn
				}	
			}	
			else	{																														// 
				LimsCount = vmCount1;		sCount=0;		bPauza=1;										// Включене паузы 1 мин
				StepAlgortm = bVkl_Tst_Zarayd_On;									
			}
		}	
		break;
		
	// ========== Отключаем Заряд ...........................................................................................8
	case bVkl_ZaprZarayd:			
		
		pVkl_Zapr_Zarayd();																									// ВКЛ ЗАПР ЗАР
		pOtkl_Test_Zarayd();																								// ОТКЛ ТЕСТ ЗАРЯД ВКЛ ЗАПР ЗАР и т.д.
		LimsCount = vsCount20;	sCount=0;		bPauza=1;												// Активация паузы 20сек
		StepAlgortm = bWaitOtkl_Zarayd;									
		break;
	
	// .......... Ожидание Отключения заряда ................................................................................9
	case bWaitOtkl_Zarayd:			
	
		if ((!bPauza)&&
				((stat3[iMUK_ZRU2] & bZaprZar)||																	// включен запрет Заряда
				(stat3[iMUK_ZRU3] & bZaprZar))) 
		{
			if	(aI_zar > aIporog)		stat2[iMUK_ZRU] |= errNoOtklZar;					// Собщение "Не отключился Заряд АБ"=1
			//if	(aI_zar < 50)		stat2[iMUK_ZRU] |= errNoOtklZar;						// Отладочная заглушка, Собщение "Не отключился Заряд АБ"=1
			else								stat2[iMUK_ZRU] &= ~errNoOtklZar;							// Собщение "Не отключился Заряд АБ"=0
			statTVC = 2;			tstatTVC =0;																		// Этап проведения ТВЦ
			cntRazr = 1; 																											// Сейчас будет первый разряд
			C_raz = 0;		W_raz = 0;																					// На всякий случай сбрасываем рассчетные параметры
			StepAlgortm = bVkl_Test_Razr;																			// 
		}
		break;
		
	// лист 36
	// ========== Включение тестового разряда ...............................................................................10
	case bVkl_Test_Razr:			

		pVkl_Test_Razrayd();
		pOtkl_Zapr_Razrayd();
		time_Razr = 0; 																										// Начала заново процесс разряда
		sCount_2h = 0; 																										//	подготавливаем переменную, с помощью которой будем ждать 20 секунд, прежде чем контролировать ток, переменная увеличивается раз в секунду
		LimsCount = vsCount5;	sCount=0;		bPauza=1;												// Активация паузы 5сек
		// Так как начался разряд, нужно уже начать Calculation, поэтому 
		calc_dt = calc_dt5; //дельта времени соответствует 20 секундам	
		Uab_old = Uab;	aI_razrOld = aI_razr; //фиксируем текущие U и I
		//
		StepAlgortm = bWaitVkl_Test_Razr;																
		break;
	
	// .......... Ожидание включения разряда ................................................................................11
	case bWaitVkl_Test_Razr:
		
		if (!bPauza) {
			Calculation();																										// После разрешения разряда и паузы нужно посчитать
			
			if (sCount_2h >= 20) //если прошло достаточно времени
			{
				if	(aI_razr <= aIporog)	
				//if	(40 <= aIkomp)																							// Отладочная заглушка
				{
					stat2[iMUK_ZRU] |= errNoVklRazr;																// Собщение "Не включился разряд"=1
					if (cntRazr==1) StepAlgortm = bVkl_Test_Razr;										// При "первом" разряде зацикливаемся
					else	{					StepAlgortm = bTst_T_Razryda;										// Иначе идем дальше
					}	
				}
				else	{						
					StepAlgortm = bTst_T_Razryda;
					stat2[iMUK_ZRU] &= ~errNoVklRazr;																// Собщение "Не включился разряд"=0
				}		

				LimsCount = dt5; 	sCount = 0;		bPauza = 1;												// vsCount5;Активация паузы 5 сек для расчёта W C
				calc_dt = calc_dt5; 																							//дельта времени соответствует 5 секундам			
				switch (cntRazr)	{																								// Счётчик "Разрядов" в ТВЦ
				case 1:	case 3:			bFlag = 1;	break;														// 
				case 2:	case 4:			bFlag = 0;	break;														// 
				}					
			}
			else  //если же ток еще рано измерять
			{
				LimsCount = vsCount5;	sCount=0;		bPauza=1;												// Активация паузы 5сек
				StepAlgortm = bWaitVkl_Test_Razr; 																//зацикливаемся на Calculation и ожидание time_Razr >= 20
			}
		}			
		break;
	 
	// ========== Разряд АБ и вычисление C и W ..............................................................................12
	case bTst_T_Razryda:				

		if (!bPauza) {
			if	(
					(T >= Traz)||																									// Контроль температуры
					((time_Razr >= tP_TVC)&&(bFlag))||														// tP_TVC = 40*60 сек
					(Uab <= Uminrazr)||																						// напряжения АБ
					(Umin_ak <= UAKminrazr)||
					((stat3[iMUK_ZRU2] & bZaprRazr)&&															// включен запрет Разряда
					( stat3[iMUK_ZRU3] & bZaprRazr))
					)																															// включен запрет Разряда
			{			 
				if	(T >= Traz)	{	stat3[iMUK_ZRU] |= errPrevDopustT;}						// Превышение допустимой температуры
				pVkl_Zapr_Razrayd();
				pOtkl_Test_Razrayd();
				stat2[iMUK_ZRU] &= ~errNoVklRazr;																// Собщение "Не включился разряд"=0
				LimsCount = vsCount20;	sCount = 0;		bPauza = 1;								// Активация паузы 20 сек
				if (cntRazr==4)	P0_Ras = P;																			// Среднее давление НВА при прекращении  тестового разряда НВАБ (НАБ (76+-1В)

				StepAlgortm = bWaitOtkl_Razrayd;																// Переход запрет разряда StepNext=bOtkl_Razrayd;
			}			 
			else	{																														// ((P < 3)||(U <= 76))
				Calculation();
				LimsCount = dt5;	sCount = 0;		bPauza = 1;											// Активация паузы 5 сек
				StepAlgortm = bTst_T_Razryda;
			}	
		}	
		break;
		
	// ========== Отключаем разряд ..........................................................................................13
	case bWaitOtkl_Razrayd:					

		if ((!bPauza)&&
					((stat3[iMUK_ZRU2] & bZaprRazr)||															// включен запрет Разряда
					( stat3[iMUK_ZRU3] & bZaprRazr)))
			{
			if (aI_razr > aIporog)	{																						// Сообщение "Не отключился разряд"=1
			//if (aI_razr > 5)	{																							// Отладочная заглушка
					stat2[iMUK_ZRU] |= errNoOtklRazr;															// Сообщение "Не отключился разряд"=1
					StepAlgortm = bInitEnd_Alg_TVC;																// ******** Окончание ТВЦ
			}
			else	{
				stat2[iMUK_ZRU] &= ~errNoOtklRazr;	
				
				//сброс флагов синхронизации в зависимости от номера разряда
				switch (cntRazr)																								
				{
					case 1:	
						stat4[iMUK_ZRU] &= ~bready3;	//cntRazr = 1, этап 2
						break;					
					case 2:	
						stat4[iMUK_ZRU] &= ~bready4;	//cntRazr = 2, этап 2
						break;	
					case 3:
						stat4[iMUK_ZRU] &= ~bready5;	//cntRazr = 3, этап 8
						break;					
					case 4:	
						stat4[iMUK_ZRU] &= ~bready6;	//cntRazr = 4, этап 8
						break;					
				}	
				
				LimsCount = vsCount5;	sCount = 0;		bPauza = 1;									// Активация паузы 5сек
				switch (cntRazr)																								// Обработчик состояний алгоритма заряда
				{
					case 2:	statTVC = 3;	tstatTVC = 0;		sCount_2h=0;							// Этап проведения ТВЦ						
									// Следующий этап выбирается исходя из необходимости: либо происходят наземные испытания, либо полет
									#ifdef HOURS2 
										StepAlgortm = bWait_2; //Наземные испытания, ждем 2 часа														
									#else					
										StepAlgortm = bTst_T_NVAB; // Полет, идем на проверку температуры
									#endif					
									break;
					case 1:	
					case 3:	
					case 4:	StepAlgortm = bTst_T_NVAB;
									break;
				}	
				}	
			}	
		break;
	 
	// ========== Проверка температуры АБ ...................................................................................14
	case bTst_T_NVAB:
	
		switch (cntRazr)																								
		{
			case 1:	//cntRazr = 1, этап 2
				sync_AND_2 = ( stat4[iMUK_ZRU2] & bready3 ) && ( stat4[iMUK_ZRU3] & bready3 ); 	
				sync_OR_1 = ( stat4[iMUK_ZRU2] & bready3 ) || ( stat4[iMUK_ZRU3] & bready3 );
				break;					
			case 2:	//cntRazr = 2, этап 2
				sync_AND_2 = ( stat4[iMUK_ZRU2] & bready4 ) && ( stat4[iMUK_ZRU3] & bready4 ); 	
				sync_OR_1 = ( stat4[iMUK_ZRU2] & bready4 ) || ( stat4[iMUK_ZRU3] & bready4 );	
				break;	
			case 3: //cntRazr = 3, этап 8
				sync_AND_2 = ( stat4[iMUK_ZRU2] & bready5 ) && ( stat4[iMUK_ZRU3] & bready5 ); 	
				sync_OR_1 = ( stat4[iMUK_ZRU2] & bready5 ) || ( stat4[iMUK_ZRU3] & bready5 );	
				break;					
			case 4:	//cntRazr = 4, этап 8
				sync_AND_2 = ( stat4[iMUK_ZRU2] & bready6 ) && ( stat4[iMUK_ZRU3] & bready6 ); 	
				sync_OR_1 = ( stat4[iMUK_ZRU2] & bready6 ) || ( stat4[iMUK_ZRU3] & bready6 );	
				break;
			default:
				sync_AND_2 = 0;
				sync_OR_1 = 0;
				break;
		}		
		
		if (!bPauza) {																											// Пауза 
			if	((T <= Tn2_def)||																								// Tn2_def = 30
						sync_AND_2) 		//если в двух других МК уже есть нужный флаг
			{			
				//если мы попали внутрь, значит нужно изменить состояние флага синхронизации соответствующего этапа
				switch (cntRazr)																								
				{
					case 1:	
						stat4[iMUK_ZRU] |= bready3;	//cntRazr = 1, этап 2
						break;					
					case 2:	
						stat4[iMUK_ZRU] |= bready4;	//cntRazr = 2, этап 2
						break;	
					case 3:
						stat4[iMUK_ZRU] |= bready5;	//cntRazr = 3, этап 8
						break;					
					case 4:	
						stat4[iMUK_ZRU] |= bready6;	//cntRazr = 4, этап 8
						break;					
				}						
				
				stat3[iMUK_ZRU] &= ~errPrevDopustT;
				
				if (sync_OR_1) //если хотя бы водном из двух других МК есть флаг синхронизации
				{
					if (cntRazr==2) { StepAlgortm = bVkl_RS; bPauza = 0; }					// если мы находимся в начале третьего этапа
					else 
					{
						if (cntRazr==4) { StepAlgortm = bVkl_Tst_Zarayd9;	tstatTVC = 0; }	// идем в начало 9-ого этапа, а время tstatTVC сбрасываем тут, чтобы при восстановлении этапа и переходе на шаг bVkl_Tst_Zarayd9 с оно не обнулялось
						else						StepAlgortm = bRepeatRazr;
					}
				}
				else	{
					bPauza = 0;
//					LimsCount = vsCount2;		sCount=0;		bPauza=1;										// Сброс счётчика, включене паузы 2 сек
					StepAlgortm = bTst_T_NVAB;	
				}
		  }
			else	{	// if	((T <= Tn2_def)||	
				LimsCount = vsCount5;		sCount=0;		bPauza=1;										// Сброс счётчика 5 сек, включене паузы 5 сек
				StepAlgortm = bTst_T_NVAB;	}
		}
		break;   
		
	// лист 37
	// ========== Повторить разряд АБ .......................................................................................15
	case bRepeatRazr:
		
			cntRazr++;
			StepAlgortm = bVkl_Test_Razr;																				// Следующий шаг алгоритма	лист 35
		break;   

	// лист 38
	// ========== Ждём 2 часа ...............................................................................................16
	case bWait_2:																													// В режиме отладки	vhCount2 = 20 сек
		
		//statTVC = 3;			tstatTVC =0;																		// Этап проведения ТВЦ
	 if (!bPauza)	{
		if ((sCount_2h >= vhCount2) ||																			// 2*60*60сек
				((stat4[iMUK_ZRU2] & bready4)&&																	// включен запрет Разряда
				( stat4[iMUK_ZRU3] & bready4)))
		{
			stat4[iMUK_ZRU] |= bready4;	

			stat3[iMUK_ZRU] &= ~errPrevDopustT;

			bPauza = 0;	//LimsCount = vsCount5;	sCount = 0;											// Активация паузы 20сек
			StepAlgortm = bVkl_RS;
		}	
 	 }	
		break;
		
	// .......... Ожидание Вкл РС ...........................................................................................17
	case bVkl_RS:								
		
		if (!bPauza)	{
			if ((stat4[iMUK_ZRU2] & bready4)||																	// 
					(stat4[iMUK_ZRU3] & bready4))
				{
					statTVC = 4;				tstatTVC =0;																	// Этап проведения ТВЦ
					pVkl_RS(0);																												// Включаем РС
					LimsCount = vsCount20;	sCount = 0;		bPauza = 1;									// Активация паузы 20сек
					StepAlgortm = bWaitVkl_RS;
				}	
				else	{
					//LimsCount = vsCount5;	sCount = 0;		bPauza = 1;										// Активация паузы 20сек
					StepAlgortm = bVkl_RS;
				}	
	}
		break;
		
	// .......... Ожидание Вкл РС ...........................................................................................17
	case bWaitVkl_RS:								
		
		//statTVC = 4;			tstatTVC =0;																		// Этап проведения ТВЦ
		if (!bPauza)
		{
			stat3[iMUK_ZRU] &= ~bready;
			if	(stat1[iMUK_ZRU] & bPC)	{	
				stat2[iMUK_ZRU] &= ~errNoVklRS;																	// Собщение "Не включается РС"=0
				sCount_2h=0;
				LimsCount = vsCount20;	sCount = 0;		bPauza = 1;								// Активация паузы 20 сек
				StepAlgortm = bCountTimeRazr;																		// Переход на счёт времени разряда напряжения до 0
			}	
			else	{																														// 
				stat2[iMUK_ZRU] |= errNoVklRS;																	// Собщение "Не включается РС"=1
				LimsCount = vmCount1;		sCount = 0;		bPauza = 1;																				// Активация паузы 1 мин
				StepAlgortm = bVklKomp;																					// Переход на включение КОМП
			}		
		}
//		else	{
//			LimsCount = vsCount5;	sCount = 0;		bPauza = 1;										// Активация паузы 20сек
//			StepAlgortm = bWaitVkl_RS;
//		}	
		break;
		
	// ========== Разряд АБ с РС не более 30 час ............................................................................18
	case bCountTimeRazr:				
																																				// При отладке ждём 10 сек и сбрасываем флаг bPC[iMUK_ZRU]
		if (!bPauza) {
			if ((sCount_2h >= tRazr)||																				// Если время разряда достигло предела, timeRazr = 108000сек (30*60*60)
				(Umin_ak <= UakminPC)||(!(stat1[iMUK_ZRU] & bPC))||							// или  АБ разряжена, или поступила команда ОТКЛ РС
				((stat3[iMUK_ZRU2] & bready)&&																	// включен запрет Разряда
				( stat3[iMUK_ZRU3] & bready))
				)
			{			
				stat3[iMUK_ZRU] |= bready;																			// На откл РС

				//LimsCount = vsCount40;	sCount = 0;		bPauza = 1;								// Активация паузы 20сек
				StepAlgortm = bOtkl_RS;																					// Переход на отключение разряда РС
			}	
			else	{																														// Счёт времени
				LimsCount = vsCount5;	sCount = 0;		bPauza = 1;								// Активация паузы 5 сек
				StepAlgortm = bCountTimeRazr;
			}	
		}	
		break;
		
	// .......... Отключаем РС ..............................................................................................19
	case bOtkl_RS:
		
		if (//(!bPauza) ||																										// 40сек
				((stat3[iMUK_ZRU2] & bready)||																	// включен запрет Разряда
				( stat3[iMUK_ZRU3] & bready))
			 ) {
			 pOtkl_RS(0);																											// Отключаем РС
			 LimsCount = vsCount20;	sCount = 0;		bPauza = 1;									// Активация паузы 20сек
			 StepAlgortm = bWaitOtkl_RS;																			// Переход на отключение разряда РС
		}	
		break;
		
	case bWaitOtkl_RS:							

		if (!bPauza) {
			if	(stat1[iMUK_ZRU] & bPC)		stat2[iMUK_ZRU] |= errNoOtklRS;			// Собщение "Не отключается РС"
			LimsCount = vmCount1;		sCount = 0;		bPauza = 1;									// При отладке vmCount5 = 2 сек Активация паузы
			StepAlgortm = bVklKomp;																						// Переход на включение КОМП
		}
		break;

	// ========== Включение КОМП заряда АБ  .................................................................................20
	case bVklKomp:			
		
		if (!bPauza) {
			statTVC = 5;				tstatTVC =0;																	// Этап проведения ТВЦ
			pVkl_KOMP();
			pVkl_Test_Zarayd();
			pOtkl_Zapr_Zarayd();
			LimsCount = vsCount20;	sCount = 0;		bPauza = 1;									// Активация паузы 20сек
			StepAlgortm = bWaitVklKomp;																				// Переход на включение КОМП
		}
		break;
	
	// .......... Ожидание Вкл_КОМП .........................................................................................21
	case bWaitVklKomp:						

		if (!bPauza) {
			if (aI_zar > aIkomp)	{	stat2[iMUK_ZRU] |= errNoVklCompZar;				// aIkomp = 3; Сообщение "Не включился КОМП заряд"=1
				pOtkl_KOMP();
				LimsCount = vsCount20;	 sCount = 0;	bPauza = 1;								// Включене паузы 20 сек
				StepAlgortm = bWaitOtklKomp2;																		// Откл КОМП
			}
			else	{									stat2[iMUK_ZRU] &= ~errNoVklCompZar;
				sCount_2h=0;
				StepAlgortm = bZarydComp;
			}	
		}	
		break;
		
	// ========== Заряд АБ компенсационным током ............................................................................22
	case bZarydComp:					
		
	if (!bPauza) {																												// При отладке vhCount2 = 20 сек
			if	((Uab > 80)&&(sCount_2h >= vmCount10))	{												// 
				pOtkl_KOMP();
				LimsCount = vsCount20;	 sCount = 0;	bPauza = 1;								// Включене паузы 20 сек
				StepAlgortm = bWaitOtklKomp2;																		// 
			}
			else	{																														// 
				LimsCount = vsCount20;	 sCount = 0;	bPauza = 1;	}							// Включене паузы 20 сек
		}
		break;
		
	// лист 39
	// .......... Ожидание Откл_КОМП ........................................................................................23
	case bWaitOtklKomp2:						

		statTVC = 6;					tstatTVC =0;																	// Этап проведения ТВЦ
		if (!bPauza) {
			stat2[iMUK_ZRU] &= ~errNoVklCompZar;															// Сообщение "Не включился КОМП заряд"=0
			if (aI_zar <= aIporog)		stat2[iMUK_ZRU] |= errNoOtklCompZar;			// Сообщение "Не отключился КОМП заряд"=1			
			stat4[iMUK_ZRU] &= ~bready1; //обнуляем переменную синхронизации
			StepAlgortm = bVkl_Tst_Zarayd_On2;
		}	
		break;

	// ========== Вспомогательные шаги для восстановления 6-го этапа тестирования .......................................................................
	//этот вспомогательный шаг выполняет команды, которые нужны, чтобы начать шестой этап, при восстановлении надо подать их заново
	case bTest_Vosst_et6_command:			
		if (!bPauza) {
			pVkl_Test_Zarayd();		pOtkl_Zapr_Zarayd(); 			
			LimsCount = vsCount20;	sCount = 0;		bPauza = 1;					// Активация паузы 20сек
			StepAlgortm = bTest_Vosst_et6_checkI;									
		}	
		break;
		
	//этот вспомогательный шаг контролирует ток после подачи команд
	case bTest_Vosst_et6_checkI:			
		if (!bPauza) {
			stat2[iMUK_ZRU] &= ~errNoVklCompZar;															// Сообщение "Не включился КОМП заряд"=0
			if (aI_zar <= aIporog)		stat2[iMUK_ZRU] |= errNoOtklCompZar;			// Сообщение "Не отключился КОМП заряд"=1			
			stat4[iMUK_ZRU] &= ~bready1; //обнуляем переменную синхронизации
			StepAlgortm = bVkl_Tst_Zarayd_On2; //все вспомогательные шаги выполнены, можем возвращаться к штатному алгоритму					
		}	
		break;
		
	// ========== Заряжаем батарею максимальным током .......................................................................24
	case bVkl_Tst_Zarayd_On2:			

		if (!bPauza) {
			if	((T >= Tn3)||																									// (T>Tn3)||
					 ((0.8*Pn <= P) && (P <= Pn) && (T >= Tn2))||									// (0.8Pn<=P<=Pn && T>=Tn2)||
					 ((Pn <= P) && (P <= Pv) && (T >= Tn1))||											// (Pn<=P<=Pv && T>=Tn1)
					 (P >= Pv)||																									// (P>=Pv)
					 ((stat4[iMUK_ZRU2] & bready1)&&															// 
					  (stat4[iMUK_ZRU3] & bready1))
					)	
			{
				if (P < Pv)		stat3[iMUK_ZRU] |= errPrevDopustT;								// "Превышение допустимой температуры НВАБ"
				stat4[iMUK_ZRU] |= bready1;
				LimsCount = vsCount5;		sCount=0;	bPauza=1;											// Для отладки vmCount1 = 2 сек	Сброс счётчика, включене паузы
				StepAlgortm = bVklKomp2;																				// Для первого заряда StepNext = bOtkl_Zarayd. Для второго StepNext = bCmp_T_Tv_P_Pn
			}	
			else	{																														// 
				LimsCount = vmCount1;		sCount=0;	bPauza=1;											// Для отладки vmCount1 = 2 сек	Сброс счётчика, включене паузы
				StepAlgortm = bVkl_Tst_Zarayd_On2;									
			}
		}	
		break;
	
	// ========== Вспомогательные шаги для восстановления 7-го этапа тестирования .......................................................................
	//этот вспомогательный шаг выполняет команды, которые нужны, чтобы начать седьмой этап, при восстановлении надо подать их заново
	case bTest_Vosst_et7_command:			
		if (!bPauza) {		
			pVkl_KOMP();	
			stat2[iMUK_ZRU] &= ~errNoOtklCompZar;															// Сообщение "Не отключился КОМП заряд"=0			
			pVkl_Test_Zarayd();		pOtkl_Zapr_Zarayd(); 
			LimsCount = vsCount20;	sCount = 0;		bPauza = 1;					// Активация паузы 20сек
			StepAlgortm = bWaitVklKomp2;									
		}	
		break;
		
	// ========== Включение КОМП заряда АБ  .................................................................................25
	case bVklKomp2:			
		
		if (!bPauza) {
			if 	((stat4[iMUK_ZRU2] & bready1)||																	// включен запрет Разряда
					( stat4[iMUK_ZRU3] & bready1))
			{
				statTVC = 7;					tstatTVC =0;																// Этап проведения ТВЦ
				pVkl_KOMP();
				stat2[iMUK_ZRU] &= ~errNoOtklCompZar;															// Сообщение "Не отключился КОМП заряд"=0
				LimsCount = vsCount20;	sCount = 0;		bPauza = 1;									// Активация паузы 20сек
				StepAlgortm = bWaitVklKomp2;																			// Переход на проверку включения КОМП
			}
			else	{
				LimsCount = vsCount5;	sCount = 0;		bPauza = 1;										// Активация паузы 5сек
			}	
		}
		break;
	
	// .......... Ожидание Вкл_КОМП .........................................................................................26
	case bWaitVklKomp2:						

		if (!bPauza)		{
			if (aI_zar > aIkomp)	{																									// 
					stat2[iMUK_ZRU] |= errNoVklCompZar;														// Сообщение "Не включился КОМП заряд"=1
					StepAlgortm = bInitEnd_Alg_TVC;																// ******** Окончание ТВЦ
			}
			else	{									
					stat2[iMUK_ZRU] &= ~errNoVklCompZar;
					sCount_2h=0;	
					stat4[iMUK_ZRU] &= ~bready2;
					LimsCount = vsCount10;	sCount = 0;		bPauza = 1;									// Активация паузы 5сек
					StepAlgortm = bZarydComp2;
			}	
		}	
		break;
		
	// лист 40
	// ========== Заряжаем батарею компенсационным током "КОМП" .............................................................27
	case bZarydComp2:			

		if (!bPauza) {
			if (((sCount_2h >= tVir)||(T >= Tk))||																// Для отладки tVir = 10 сек
					((stat4[iMUK_ZRU2] & bready2)&&
					( stat4[iMUK_ZRU3] & bready2))
				)
			{
				stat4[iMUK_ZRU] |= bready2;
				if 	((stat4[iMUK_ZRU2] & bready2)||																	// 
						( stat4[iMUK_ZRU3] & bready2))
				{
					PvzRas = P;
					pVkl_Zapr_Zarayd();
					pOtkl_Test_Zarayd();																						// РћРўРљР› РўР•РЎРў Р—РђР РЇР” Р’РљР› Р—РђРџР  Р—РђР  Рё С‚.Рґ.
					pOtkl_KOMP();

					statTVC = 8;				tstatTVC = 0;																// Р­С‚Р°Рї РїСЂРѕРІРµРґРµРЅРёСЏ РўР’Р¦

					LimsCount = vsCount20;	sCount = 0;	bPauza = 1;									// Р’РєР»СЋС‡РµРЅРµ РїР°СѓР·С‹ 20 СЃРµРє
					StepAlgortm = bWaitVkl_Test_Razr8;															// РќР° РѕР¶РёРґР°РЅРёРµ РІРєР»СЋС‡РµРЅРёСЏ СЂР°Р·СЂСЏРґР°
				}
			}
			else	{
				LimsCount = vsCount5/*vmCount1*/;		sCount = 0;	bPauza = 1;			// Р’РєР»СЋС‡РµРЅРµ РїР°СѓР·С‹ 5 РјРёРЅ
				StepAlgortm = bZarydComp2;																			// Р—Р°СЂСЏРґ РєРѕРјРїРµРЅСЃР°С†РёРѕРЅРЅС‹Рј С‚РѕРєРѕРј
			}	
		}
		break;
					
	// ========== Р—Р°СЂСЏР¶Р°РµРј Р±Р°С‚Р°СЂРµСЋ РєРѕРјРїРµРЅСЃР°С†РёРѕРЅРЅС‹Рј С‚РѕРєРѕРј "РљРћРњРџ" .............................................................27
	case bWaitVkl_Test_Razr8:			

		if (!bPauza) {
			stat4[iMUK_ZRU] &= ~bready2;
			// ............................................................... РџРµСЂРµС…РѕРґ РЅР° "Р РђР—Р РЇР”" .............
			C_raz = 0;		W_raz = 0;			time_Razr = 0;	
			cntRazr = 3;																												// cntRazr = 3
			StepAlgortm = bVkl_Test_Razr;																			// РџРµСЂРµС…РѕРґ РЅР° "Р РђР—Р РЇР”"	Р»РёСЃС‚ 35
		}
		break;
					
	// лист 42
	// ========== Принудительный заряд до рабочей уставки P2 ................................................................28
	case bVkl_Tst_Zarayd9:
		
		statTVC = 9;																						// Этап проведения ТВЦ
		iUst = 1;																														// N=2
		pOtkl_KOMP();
		pVkl_Test_Zarayd();																									// 
		pOtkl_Zapr_Zarayd();
		LimsCount = vsCount20;	sCount = 0;		bPauza = 1;										// Активация паузы 20сек
		StepAlgortm = bWaitVkl_Tst_Zar9;																		// На ожидание Заряда
		break;
			
	// .......... Ожидание включения Тест Заряда ............................................................................29
	case bWaitVkl_Tst_Zar9:		

		if (!bPauza) {
			//if	(5 > aIkomp)	{	stat2[iMUK_ZRU] &= ~errNoVklZar;					// Собщение "Не включился Заряд АБ"=0
			if	(aI_zar > aIporog)	{	stat2[iMUK_ZRU] &= ~errNoVklZar;					// Собщение "Не включился Заряд АБ"=0
				LimsCount = vsCount20;	sCount = 0;		bPauza = 1;								// Активация паузы 20сек
				StepAlgortm = bVkl_Tst_Zar_On9;
			}
			else	{									stat2[iMUK_ZRU] |= errNoVklZar;						// Собщение "Не включился Заряд АБ"=1
				StepAlgortm = bInitEnd_Alg_TVC;
			}	
		}
		break;
		
	// ========== Заряжаем батарею ..........................................................................................30
	case bVkl_Tst_Zar_On9:			

		if (!bPauza) {
			if	((P >= Pvuz2)||((T >= T_max)&&(P>=0.8*Pvuz2))||
					 ((stat3[iMUK_ZRU2] & bZaprZar)&&															// включен запрет Разряда
					  (stat3[iMUK_ZRU3] & bZaprZar))
					)
			{			 
				pVkl_Zapr_Zarayd();																							// ВКЛ ЗАПР ЗАР
				pOtkl_Test_Zarayd();																						// ОТКЛ ТЕСТ ЗАРЯД ВКЛ ЗАПР ЗАР и т.д.
				StepAlgortm = bWaitOtkl_Tst_Zar9;									
			}	
			else	{																														// 
				StepAlgortm = bVkl_Tst_Zar_On9;									
			}
			LimsCount = vsCount20;	sCount = 0;		bPauza = 1;									// Активация паузы 20сек
		}	
		break;
		
	// .......... Ожидание Отключения заряда ................................................................................31
	case bWaitOtkl_Tst_Zar9:			
	
		if (!bPauza) {
			if 	((stat3[iMUK_ZRU2] & bZaprZar)||															
					( stat3[iMUK_ZRU3] & bZaprZar))
			{
				if	(aI_zar > aIporog)	stat2[iMUK_ZRU] |= errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=1
				else									stat2[iMUK_ZRU] &= ~errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=0
				StepAlgortm = bViborUst;																					// 
			}
			else	{
				StepAlgortm = bWaitOtkl_Tst_Zar9;									
				LimsCount = vsCount5;	sCount = 0;		bPauza = 1;										// Активация паузы 5сек
			}	
		}
		break;
		
	// ========== Выбор уставки .............................................................................................32
	case bViborUst:

		if	((dP >= dP_u)||(dUak>=dU_u))	iUst=2;															// N=3
		else	{
			if	(T >= Tu[1])								iUst=0;															// N=1
			else														iUst=1;															// N=2
		}	
		StepAlgortm = bInitEnd_Alg_TVC;
		break;

	// ========== Завершение алгоритма ......................................................................................33
	case bInitEnd_Alg_TVC:		
		StepAlgortm = bEnd_Alg_TVC;
		// Эти функции мы будем выполнять в функции main в режиме mode = Otkl_TEST	
		//		pVkl_Zapr_Zarayd();																										// «ЗАПРЕТ ЗАРЯД»  = 1
		//		pVkl_Zapr_Razrayd();																									// «ЗАПРЕТ РАЗРЯД» = 1
		//		pOtkl_Test_Zarayd();																									// "Откл_ТЕСТ ЗАР" 
		//		pOtkl_Test_Razrayd();																									// "Откл_ТЕСТ РАЗР" 	
		// 		LimsCount = vsCount20;	sCount=0;		bPauza_TVC=1;											// Активация паузы 20 сек
	
	// .......... Конец алгоритма ...........................................................................................
	case bEnd_Alg_TVC:				
		
		//if (!bPauza_TVC) 	{
			mode = Otkl_TEST;
		//}
		break;
	
	} //end of switch (StatusZarRazr)
}


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Podzarayd (void)														/*	_Подзаряд__Н_В_А_Б_ */
{	

	switch (StepAlgortm)																									// Обработчик состояний алгоритма заряда
	{
	// .......... Инициализация .............................................................................................
	case bInitPodzaryd:
		
		stat2[iMUK_ZRU] = 0;
		stat3[iMUK_ZRU] &= ~(errNoOgrTokRazr|errNoOgrTokZar|errPrevDopustT);
		StepNext = bStartPodzaryd;

	// .......... Включение запрета Заряда ..................................................................................
	case bVkl_ZaprZarayd:
		
		pVkl_Zapr_Zarayd ();																								// Запрет заряда=1
		LimsCount = vsCount20;	sCount=0;		bPauza=1;												// Активация паузы 20 сек
		StepAlgortm = bWaitVkl_ZaprZara;																		// След шаг алгоритма Ожидание включения запрета заряда
		break;

	// .......... Ожидание включения запрета Заряда .........................................................................
	case bWaitVkl_ZaprZara:		

		if (!bPauza) {
			if	(aI_zar > aIporog)	stat2[iMUK_ZRU] |= errNoOtklZar;					// Собщение "Не отключился Заряд АБ"=1
			StepAlgortm = StepNext;																						// След шаг алгоритма
		}
		break;

	// ......................................................................................................................
	case bStartPodzaryd:			
		
		if (P >= Pnuz)																											// 
			StepAlgortm = bVklKomp;																						// Переход на включение компенсационного тока
		else
			StepAlgortm = bTst_Tnuzab;																				// P < Pнуз идём по алгортму
		break;
					
	// ......................................................................................................................
	case bTst_Tnuzab:			
		
		if (T < TnuzAB)	{
			stat3[iMUK_ZRU] &= ~errPrevDopustT;																// "Превышение допустимой температуры при заряде" = 0

			StepNext = bTst_Tnuzab2;
			StepAlgortm = bOtkl_ZaprZarayd;																		// Переход на отключение запрета заряда
		}
		else	{	stat3[iMUK_ZRU] |= errPrevDopustT;													// Сообщение "Превышение допустимой температуры при заряде"
			StepAlgortm = bTst_Tnuzab;																				// Ожидание остывания АБ
		}
		break;
					
	// .......... Отключаем запрет заряда ...................................................................................
	case bOtkl_ZaprZarayd:						
		
		pOtkl_Zapr_Zarayd();																								// ОТКЛ ЗАПР ЗАРЯДА
		LimsCount = vsCount5;	sCount=0;		bPauza=1;													// Активация паузы 5 сек
		StepAlgortm = bWaitOtkl_ZaprZar;
		break;

	// .......... Ожидание отключения запрета Заряда ........................................................................
	case bWaitOtkl_ZaprZar:		

		if (!bPauza) {
			StepAlgortm = StepNext;																						// След шаг алгоритма
		}
		break;
	 
	// .......... Заряд током 7-10А .........................................................................................
	case bTst_Tnuzab2:			
		
			if (T >= TnuzAB)	{	
				stat3[iMUK_ZRU] |= errPrevDopustT;															// Сообщение "Превышение допустимой температуры при заряде"
				StepNext = bTst_Tnuzab_2;
				StepAlgortm = bVkl_ZaprZarayd;																	// След шаг алгоритма Ожидание включения запрета заряда
			}
			else	{
				stat3[iMUK_ZRU] &= ~errPrevDopustT;															// "Превышение допустимой температуры при заряде" = 0
				LimsCount = dt5;	sCount=0;		bPauza=1;													// Активация паузы 5сек
				StepAlgortm = bZaryd_Pnuz;																			// Переход на заряд
			}
		//}
		break;

	// ......................................................................................................................
	case bTst_Tnuzab_2:			
		
		if (!bPauza) {
			if (T <= (TnuzAB-2))	{
				StepNext = bTst_Tnuzab2;
				StepAlgortm = bOtkl_ZaprZarayd;																	// След шаг алгоритма Ожидание включения запрета заряда
			}
			else	{
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20сек
				StepAlgortm = bTst_Tnuzab_2;																		// Переход на отключение запрета заряда
			}
		}
		break;

	// .......... Заряд АБ током 7-10А до Pнуз...............................................................................
	case bZaryd_Pnuz:			
		
		if (!bPauza) {
			if (P >= Pnuz)	{
				StepAlgortm = bVklKomp;																					//	P >= Pнуз идём по алгортму
			}	
			else	{
				//Calculation_Podzaryad(aI_zar);
				StepAlgortm = bTst_Tnuzab2;																			//  Заряжаем АБ током 7-10А
			}
		}
		break;
		
	// .......... Вкл_КОМП ..................................................................................................
	case bVklKomp:
		
		pVkl_KOMP();																												// Вкл_КОМП
		pOtkl_Zapr_Zarayd();																								// ОТКЛ ЗАПР ЗАРЯДА
		LimsCount = vsCount20;	sCount=0;		bPauza=1;												// Активация паузы 20сек
		StepAlgortm = bWaitVklKomp;																					// Ожидание остывания АБ
		break;
	 
	// .......... Ожидание включения КОМП Заряда ............................................................................
	case bWaitVklKomp:		

		if (!bPauza) {
			if	(aI_zar > aIkomp)	stat2[iMUK_ZRU] |= errNoVklCompZar;					// Собщение "Не включился КОМП Заряд АБ"=1
			StepAlgortm = bVklKomp;																						// След шаг алгоритма
		}
		else	{
			stat2[iMUK_ZRU] &= ~errNoVklCompZar;
			LimsCount = dt5;	sCount=0;		bPauza=1;														// Активация паузы 5сек
			StepAlgortm = bZarydComp;																					// След шаг алгоритма
		}
		break;
		
	// .......... Заряд АБ компенсационным током..........................................................................
	case bZarydComp:					
		
		if (!bPauza) {
			if	((P >= PvuzCK)||(T >= (TvuzCK-1)))		{				
				if (T >= (TvuzCK-1))	{																					// T >= 25
					stat3[iMUK_ZRU] |= errPrevDopustT;														// Сообщение "Превышение допустимой температуры при заряде"
					StepNext = bTst_Tnuzab_22;
				}
				else	StepNext = bOtklKomp;
				StepAlgortm = bVkl_ZaprZarayd;																	// След шаг алгоритма Ожидание включения запрета заряда
			}
			else	{
				LimsCount = vsCount5;	sCount=0;		bPauza=1;											// Активация паузы 5сек
				StepAlgortm = bZarydComp;																				// След шаг алгоритма
			}
		}
		break;
		
	// ......................................................................................................................
	case bTst_Tnuzab_22:			
		
		if (!bPauza) {
			if (T <= (TvuzCK-2))	{
				stat3[iMUK_ZRU] &= ~errPrevDopustT;															// Сообщение "Превышение допустимой температуры при заряде"
				StepNext = bZarydComp;
				StepAlgortm = bOtkl_ZaprZarayd;																	// След шаг алгоритма Ожидание включения запрета заряда
			}
			else	{
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20сек
				StepAlgortm = bTst_Tnuzab_22;																		// Переход на отключение запрета заряда
			}
		}
		break;

	// .......... Откл_КОМП .................................................................................................
	case bOtklKomp:
		
		pOtkl_KOMP();																												// Откл_КОМП
		LimsCount = vsCount5;	sCount=0;		bPauza=1;													// Активация паузы 20сек
		StepAlgortm = bWaitOtklKomp;																				// Ожидание остывания АБ
		break;
	 
	// .......... Ожидание включения КОМП Заряда ............................................................................
	case bWaitOtklKomp:		

		if (!bPauza) {
			if	(aI_zar > aIkomp)	stat2[iMUK_ZRU] |= errNoOtklCompZar;			// Собщение "Не отключился Заряд АБ"=1
			StepAlgortm = bEndPodzaryda;																			// След шаг алгоритма
		}
		break;
		
 
	// .......... Отключение Подзаряда .......................................................................................
	case bEndPodzaryda:				
		
		stat1[iMUK_ZRU] &= ~bPodzaryad;																			// Команда "Подзаряд" снята
		mode = Otkl_Podzarayd;																							// 
	
 }	//end of case
}

//-------------------------------------------------------------------------------------------------------------------------
void Curr_W(void)	// Расчёт текущего уровня заряженности АБ
{	// Wтест от 0 до 4000 Вт·ч				2900	Коэффициент для расчёта текущего значения энергоёмкости АБi, Wтест
/*
Расчет значения текущей энергоемкости АБ проводится по формуле:
				W = ((P-P0)/(Pвз- P0))·Wтест
*/	
	//curW_zar = ((P-P0)/(Pvz-P0)) * W_raz;
	if (DataOk)	curW_zar = ((P-P0)/(Pvz-P0)) * kWtst;
	else				curW_zar = 0;
	//	if (curW_zar>0) Vals_ZRU[5] = curW_zar;//	else  Vals_ZRU[5] = 0;
	//curW_zar = 5000;
	Vals_ZRU[5] = curW_zar;
}

//-------------------------------------------------------------------------------------------------------------------------
//void MakePackTst(unsigned char *PackRs, int length)	// Заполнение пакета тестовыми данными	
//{	int i;

//	for(i=4; i < length-2; i++)	{	PackRs[i] = i-3;	}
//	
//	checksumCalc = Crc16(PackRs, length-2);													// Выисление контрольной суммы
//	*(PackRs+length-1) =  checksumCalc;	
//	*(PackRs+length-2) =  checksumCalc >> 8;													// Добавить контрольную сумму

//}

//-------------------------------------------------------------------------------------------------------------------------
void MakePack1(void)	// Заполнение пакета телем. данными	
{	int i;	unsigned int tmp;
/*
1		Ток заряда АБ, датчик 2						ufix8, z = 0.12 А, 		 x0 = 0 А
2		Ток заряда АБ, датчик 1						ufix8, z = 0.12 А, 		 x0 = 0 А
3		Ток разряда АБ, датчик 2					ufix8, z = 0.14 А, 		 x0 = 0 А
4		Ток разряда АБ, датчик 1					ufix8, z = 0.14 А,		 x0 = 0 А
5		Напряжение на АБ от ЗРУ						ufix8, z=0.48 В,			 x0 = 0 В
6		Текущий уровень заряженности АБ 	ufix8, z = 15.7 Вт*ч,	 x0 = 0 Вт*ч
7		Температура 2 корпуса прибора 		ufix8, z = 0.36 0C, 	 x0 = 0 0C
8		Температура 1 корпуса прибора 		ufix8, z = 0.36 0C,		 x0 = 0 0C
	
9	 6 7 Номер текущей уставки в ЗРУ
	 4 5 В АБ включены разрядные сопротивления
	 2 3 Состояние силового канала ЗРУ
	 0 1 Включен режим «Подзаряд на СК» ЗРУ
10 6 7 Включен режим «ТВЦ» ЗРУ
	 4 5 Включен режим «Основной» ЗРУ
	 2 3 Разрешен заряд ЗРУ	
	 0 1 Разрешен разряд ЗРУ
11 6 7 Не подключились РС
	 4 5 Не отключились РС
	 2 3 Не отключился разряд АБ
	 0 1 Не отключился заряд АБ
12 6 7 Не включился КОМП подзаряд
	 4 5 Не отключился КОМП подзаряд
	 2 3 Не включился разряд АБ в ТВЦ
	 0 1 Не включился заряд АБ в ТВЦ
13 6 7 Резерв
	 4 5 Не ограничивается ток разряда
	 2 3 Не ограничивается ток заряда
	 0 1 Превышена допустимая температура АБ
*/	
	// Заполнение пакета телем. данными	АЦП
	// Ток Зар2, Ток Зар1, Ток Разр2, Ток Разр1, U_АБ2, Wтекущий, ТМП2, ТМП1:	4..11 (1-8)
	for(i=0; i < nParams; i++)	{																					// nParams = 8

		if (Vals_ZRU[i] >= x0_p[i])		{																			// Vals_ZRU[i] >= 0
			tmp = (int) ((Vals_ZRU[i]-x0_p[i])/z_p[i]+0.5);
			if (tmp>255)	PackRs1[indsData_p[i]] = 255;												// indsData_p[nParams] = {5,4,7,6,9,8,11,10};
			else					PackRs1[indsData_p[i]] = tmp;
		}	
		else	PackRs1[indsData_p[i]] = 0;
	}

	tmp = MajorStatZRU(stat1);
	PackRs1[12] = tmp|((iUst+1)<<6);		PackRs1[13] = tmp>>8;
	
	tmp = MajorStatZRU(stat2);
	PackRs1[14] = tmp;		PackRs1[15] = tmp>>8;

	tmp = MajorStatZRU(stat3);
	PackRs1[16] = tmp;
	
//	MakePackTst(PackRs1, lngPackRs1);

	checksumCalc = Crc16(PackRs1, lngPackRs1-2);													// Выисление контрольной суммы
	*(PackRs1+lngPackRs1-1) =  checksumCalc;	
	*(PackRs1+lngPackRs1-2) =  checksumCalc >> 8;													// Добавить контрольную сумму

}

//-------------------------------------------------------------------------------------------------------------------------
void MakePack2_5_8_10(void)	// Заполнение пакетов-ответов, которые не содержат данные
{
	checksumCalc = Crc16(PackRs2, lngPackRs2-2);													// Выисление контрольной суммы
	*(PackRs2+lngPackRs2-1) =  checksumCalc;	
	*(PackRs2+lngPackRs2-2) =  checksumCalc >> 8;													// Добавить контрольную сумму
	
	checksumCalc = Crc16(PackRs5, lngPackRs5-2);													// Выисление контрольной суммы
	*(PackRs5+lngPackRs5-1) =  checksumCalc;	
	*(PackRs5+lngPackRs5-2) =  checksumCalc >> 8;													// Добавить контрольную сумму

	checksumCalc = Crc16(PackRs8, lngPackRs8-2);													// Выисление контрольной суммы
	*(PackRs8+lngPackRs8-1) =  checksumCalc;	
	*(PackRs8+lngPackRs8-2) =  checksumCalc >> 8;													// Добавить контрольную сумму

	checksumCalc = Crc16(PackRs10, lngPackRs10-2);												// Выисление контрольной суммы
	*(PackRs10+lngPackRs10-1) =  checksumCalc;	
	*(PackRs10+lngPackRs10-2) =  checksumCalc >> 8;												// Добавить контрольную сумму
}

//-------------------------------------------------------------------------------------------------------------------------
unsigned char fByte(int nArr, int iarr, int ind)
{	unsigned char b;	int tmp;	float fv;
	if (nArr)	fv = fVdatch[iarr].Fdata;
	else			fv = fV_AB[iarr].Fdata;
	
	if (fv >= x0_p3[ind])		{
		tmp = (int) ((fv-x0_p3[ind])/z_p3[ind]+0.5);
		if (tmp>255)	b = 255;	else	b = tmp;	}	
	else	b = 0;

	return b;
}	

//-------------------------------------------------------------------------------------------------------------------------
void MakePack3(void)	// Заполнение пакета Краткой телем. данными	
{	float minUak=2, maxUak=0;
	
	//Коррекция напряжения 72-го АК АБ
	if (mode_Razryad) {	fV_AB[76].Fdata += aI_razr*0.0;	}
	if (mode_Zaryad)  {	fV_AB[76].Fdata -= aI_zar *0.0;	}
	if (stat1[iMUK_ZRU] & bPC)		  {	fV_AB[76].Fdata += 0.1;	}
	//Поиск мин макс Uak
	for(i=4; i < 76; i++)		{
		if (fV_AB[i+1].Fdata < minUak)	minUak = fV_AB[i+1].Fdata;
		if (fV_AB[i+1].Fdata > maxUak)	maxUak = fV_AB[i+1].Fdata;
	}
	fV_AB[2].Fdata = maxUak; 
	fV_AB[3].Fdata = minUak;
	dUak  = maxUak - minUak;																							// 	 Разница напряжений макс и мин значений из 72 АК, В

  //PackRs3[4]  = fByte(1,28, 0);                                                                                // 1 Напряжение АБ, В
  PackRs3[4]  = (int) ( (Uab - x0_p3[0])/z_p3[0] + 0.5 );                                // 1 Напряжение АБ, В
	PackRs3[5]  = fByte(1, 0, 1);																					// 6 Текущее значение среднего давления в НВА, кгс/см2
	PackRs3[6]  = fByte(0, 0, 2);																					// 2 Среднее значение напряжения по 72 АК, В
	PackRs3[7]  = fByte(0, 2, 3);																					// 4 Максимальное напряжение АК,В
	PackRs3[8]  = fByte(0, 3, 4);																					// 3 Минимальное напряжение АК, В
	//fV_AB[2].Fdata = fV_AB[2].Fdata - fV_AB[3].Fdata;
	fV_AB[2].Fdata = dUak;
	PackRs3[9]  = fByte(0, 2, 5);																					// 5 Разница напряжений макс и мин значений из 72 АК, В
	
	fVdatch[2].Fdata = fVdatch[2].Fdata - fVdatch[3].Fdata;
	PackRs3[10] = fByte(1, 2, 6);																					// 9 Разница макс и мин давлений из 5-ти изм НВА, кгс/см2
	PackRs3[11] = fByte(1, 4, 7);																					// 10	Текущее значение средней температуры НВАБ, С					

	PackRs3[12] = ((bNoWrkCAN >> 5) & 0x1) |															// 5->0	Состояние резервного канала связи CAN с МК3 БЭ
								((bNoWrkCAN >> 2) & 0x2) |															// 3->1	Состояние резервного канала связи CAN с МК2 БЭ
								((bNoWrkCAN << 1) & 0x4) |															// 1->2	Состояние резервного канала связи CAN с МК1 БЭ
								((bNoWrkCAN >> 1) & 0x8) |															// 4->3	Состояние основного  канала связи CAN с МК3 БЭ	
								((bNoWrkCAN << 2) & 0x10)|															// 2->4	Состояние основного  канала связи CAN с МК2 БЭ	
								((bNoWrkCAN << 5) & 0x20);															// 0->5	Состояние основного  канала связи CAN с МК1 БЭ	

//	MakePackTst(PackRs3, lngPackRs3);

	checksumCalc = Crc16(PackRs3, lngPackRs3-2);													// Выисление контрольной суммы
	*(PackRs3+lngPackRs3-1) =  checksumCalc;	
	*(PackRs3+lngPackRs3-2) =  checksumCalc >> 8;													// Добавить контрольную сумму

}

//-------------------------------------------------------------------------------------------------------------------------
void ClearPack3(void)	// Очистить пакет Краткой телем.
{	int i;
	for(i= 4; i < lngPackRs3-7; i++)		PackRs3[i]  = 0;
	
	PackRs3[12] = ((bNoWrkCAN >> 5) & 0x1) |															// 5->0	Состояние резервного канала связи CAN с МК3 БЭ
								((bNoWrkCAN >> 2) & 0x2) |															// 3->1	Состояние резервного канала связи CAN с МК2 БЭ
								((bNoWrkCAN << 1) & 0x4) |															// 1->2	Состояние резервного канала связи CAN с МК1 БЭ
								((bNoWrkCAN >> 1) & 0x8) |															// 4->3	Состояние основного  канала связи CAN с МК3 БЭ	
								((bNoWrkCAN << 2) & 0x10)|															// 2->4	Состояние основного  канала связи CAN с МК2 БЭ	
								((bNoWrkCAN << 5) & 0x20);															// 0->5	Состояние основного  канала связи CAN с МК1 БЭ	

	checksumCalc = Crc16(PackRs3, lngPackRs3-2);													// Выисление контрольной суммы
	*(PackRs3+lngPackRs3-1) =  checksumCalc;	
	*(PackRs3+lngPackRs3-2) =  checksumCalc >> 8;													// Добавить контрольную сумму
}

//-------------------------------------------------------------------------------------------------------------------------
unsigned char fByte4(int nArr, int iarr, int ind)
{	unsigned char b;	int tmp;	float fv;
	if (nArr)	fv = fVdatch[iarr].Fdata;
	else			fv = fV_AB[iarr].Fdata;
	
	if (fv >= x0_p4[ind])		{
		tmp = (int) ((fv-x0_p4[ind])/z_p4[ind]+0.5);
		if (tmp>255)	b = 255;	else	b = tmp;	}	
	else	b = 0;

	return b;
}	

//-------------------------------------------------------------------------------------------------------------------------
void MakePack4(void)	// Заполнение пакета Полной телем. данными
{	int j;
	
	for(i=4; i < 76; i++)		{
//		if (i == 75)	{
//			if (stat1[iMUK_ZRU]	& bRazryad) {	fV_AB[i+1].Fdata = fV_AB[i+1].Fdata + aI_razr*0.002;	}
//			if (stat1[iMUK_ZRU]	& bZaryad)  {	fV_AB[i+1].Fdata = fV_AB[i+1].Fdata - aI_zar *0.002;	}
//			if (stat1[iMUK_ZRU] & bPC)		  {	fV_AB[i+1].Fdata = fV_AB[i+1].Fdata + 0.1;	}
//		}
		PackRs4[i] = (int)((fV_AB[i+1].Fdata-x0_p4[0])/z_p4[0]+0.5);				//	Напряжение 1..72-ого НВА НВАБ БЭ
	}
	j=i;

	for(i=0; i < 5; i++)		{
		PackRs4[j] = fByte4(1, 8+i, 1);	j++; 																//	значение ДД1..5	fVdatch[0][ak].Fdata;
	}
	for(i=0; i < 5; i++)		{
		PackRs4[j] = fByte4(1, 13+i, 2);	j++; 															//	значение ДД1..5	fVdatch[0][ak].Fdata;
	}
	
//	MakePackTst(PackRs4, lngPackRs4);

	checksumCalc = Crc16(PackRs4, lngPackRs4-2);													// Выисление контрольной суммы
	*(PackRs4+lngPackRs4-1) =  checksumCalc;	
	*(PackRs4+lngPackRs4-2) =  checksumCalc >> 8;													// Добавить контрольную сумму

//	for(i=4; i < lngPackRs4; i++)		{ PackRs4_2[i] = PackRs4[i];}
}

//-------------------------------------------------------------------------------------------------------------------------
void ClearPack4(void)	// Заполнение пакета Полной телем. данными	
{	int i;
	for(i=4; i < 86; i++)		{	PackRs4[i] = 0;	}														// Напряжение 1..72-ого НВА НВАБ БЭ и т.д.

	checksumCalc = Crc16(PackRs4, lngPackRs4-2);													// Выисление контрольной суммы
	*(PackRs4+lngPackRs4-1) =  checksumCalc;	
	*(PackRs4+lngPackRs4-2) =  checksumCalc >> 8;													// Добавить контрольную сумму

}

//-------------------------------------------------------------------------------------------------------------------------
void MakePack5(void)
{	int cntBadAk=0;																												// x = x’ ∙ z + x0, c

	dU_u 		 = *(p_InPack)* z_p5[0]+ x0_p5[0];														// ΔUр				1		Макс доп разброс напр между АК в АБ
	dP_u 		 = *(p_InPack+1)* z_p5[1]+ x0_p5[1];													// ΔДу				2		Макс доп разброс давл  между АК в АБ
	T_max 	 = *(p_InPack+2)*z_p5[2]+ x0_p5[2];														// ТМАХ				3		Предел доп темп АБ на заряде

	Pvuz1 	 = *(p_InPack+3)*z_p5[3]+ x0_p5[3];			Pu[0] =	Pvuz1;				// РВУЗ1			4		Верх ур зар АБ при темп АБ более ТВУЗ1
	Pvuz2 	 = *(p_InPack+4)*z_p5[4]+ x0_p5[4];			Pu[1] =	Pvuz2;				// РВУЗ2			5		Верх ур зар АБ при темп АБ не более ТВУЗ2
	Tvuz2 	 = *(p_InPack+5)*z_p5[5]+ x0_p5[5];			Tu[1] =	Tvuz2;				// ТВУЗ2			7		Предел доп темп АБ при уровне заряженности РВУЗ2
	Pvir3 	 = *(p_InPack+6)*z_p5[6]+ x0_p5[6];			Pu[2] =	Pvir3;				// РВЫР3			8		Верх ур заряж АБ при темп АБ не более ТВЫР3
	Tvir3 	 = *(p_InPack+7)*z_p5[7]+ x0_p5[7];			Tu[2] =	Tvir3;				// ТВЫР3			9		Предел доп темп АБ при уровне заряженности РВЫР3
	
	Pnuz 		 = *(p_InPack+8)*z_p5[8]+ x0_p5[8];														// РНУЗ				10	Верх ур заряжен АБ на СК при заряде током не более 10 А
	PvuzCK 	 = *(p_InPack+9)*z_p5[9]+ x0_p5[9];														// РВУЗ СК		11	Предел ур заряж АБ на СК при заряде током не более (1,5±0,5) А
	TnuzAB 	 = *(p_InPack+10)*z_p5[10]+ x0_p5[10];												// ТНУЗАБ			12	Предел доп темп АБ в начале подзаряда на СК
	TvuzCK 	 = *(p_InPack+11)*z_p5[11]+ x0_p5[11];												// ТВУЗ СК		13	Предел доп темп АБ  при заряде током не более (1,5±0,5) А до ур заряж РВУЗСК
	Pn 		 	 = *(p_InPack+12)*z_p5[12]+ x0_p5[12];												// РН					14	Нижн ур заряж АБ в режиме ТВЦ
	Pv 			 = *(p_InPack+13)*z_p5[13]+ x0_p5[13];												// РВ					15	Верх ур заряж АБ в режиме ТВЦ
	Tn3 		 = *(p_InPack+14)*z_p5[14]+ x0_p5[14];												// ТН3				16	Предел доп темпер АБ в реж ТВЦ при текущем уровне заряж Р<0,8PН
	Tn2 		 = *(p_InPack+15)*z_p5[15]+ x0_p5[15];												// ТН2				17	Предел доп темпер АБ в режиме ТВЦ при тек ур заряж Р в диап от 0,8PН до PН
	Tn1 		 = *(p_InPack+16)*z_p5[16]+ x0_p5[16];												// ТН1				18	Предел доп темпер АБ в режиме ТВЦ при тек ур заряж Р в диапазоне от PН до PВ
	Tk 	 		 = *(p_InPack+17)*z_p5[17]+ x0_p5[17];												// ТК					19	Предел доп темпер АБ при включ  комп заряда
	tVir 	 	 = (*(p_InPack+18)*z_p5[18]+ x0_p5[18])*60;										// tВЫР				20	Время выравн давл в АК в АБ в режиме ТВЦ при комп заряде
	Pvz 		 = *(p_InPack+19)* z_p5[19]+ x0_p5[19];												// Рвз				21	Средн давл в АК в режиме ТВЦ после комп заряда
	Traz 		 = *(p_InPack+20)* z_p5[20]+ x0_p5[20];												// Траз				22	Предел доп темпер АБ в режиме ТВЦ при разряде
	P0		 	 = *(p_InPack+21)* z_p5[21]+ x0_p5[21];												// Р0					23	Средн давл в АК в реж ТВЦ при прекр разряда АБ
	tRazr 	 = (*(p_InPack+22)*z_p5[22]+ x0_p5[22])*60;										// tРАЗ				24	Время разр АБ в реж ТВЦ на разрядное сопротивление
	kWtst 	 = *(p_InPack+23)* z_p5[23]+ x0_p5[23];												// Wтест			25	Коэфф для расчёта тек значения энергоёмкости АБ
	tP_TVC 	 = *(p_InPack+24)*z_p5[24]+ x0_p5[24];												// tР ТВЦ			26	Время принуд разряда в ТВЦ
	
	cntBadAk = 0;
	for(i=0; i < 5; i++)		{
		nBadAk[i] = *(p_InPack+25+i);																				// Номера отказавших аккумуляторов 
		if (nBadAk[i])	cntBadAk++;
	}
	nGudAk = nAllAE - cntBadAk;

//	if (cntBadAk)	{
		CurrentDlc = 5; CurrentCmd = CAN_NumBadAk;
		CAN_SendCmd(AdrMUK_ZRU, CurrentDlc, CurrentCmd);										// отправки пакета в БЭ
		bRunCmdCAN = 1;		bTimeOutCmd = 1;
//		CAN_SendCmd(AdrMUK2_ZRU, CurrentDlc, CurrentCmd);										// отправки пакета в БЭ
//		bRunCmdCAN = 1;		bTimeOutCmd = 1;
//		CAN_SendCmd(AdrMUK3_ZRU, CurrentDlc, CurrentCmd);										// отправки пакета в БЭ
//		bRunCmdCAN = 1;		bTimeOutCmd = 1;
//	}
}

//-------------------------------------------------------------------------------------------------------------------------
void MakePack6(void)
{	float add = 0.5;																											// x = x’ ∙ z + x0

	PackRs6[4]   = (dU_u   - x0_p5[0])/z_p5[0] + add;											// Максимально допусстимый разброс напряжений ΔUр 
	PackRs6[5]   = (dP_u   - x0_p5[1])/z_p5[1] + add;											// Максимально допусстимый разброс давления ΔДу
	PackRs6[6]   = (T_max  - x0_p5[2])/z_p5[2] + add;											// Предел доп темп АБ на заряде
	PackRs6[7]   = (Pvuz1  - x0_p5[3])/z_p5[3] + add;											// Верхний уровень заряженности НВАБ
	PackRs6[8]   = (Pvuz2  - x0_p5[4])/z_p5[4] + add;											// Верхний уровень заряженности НВАБ
	PackRs6[9]   = (Tvuz2  - x0_p5[5])/z_p5[5] + add;											// Предельно допустимая температура НВАБ при уровне заряженности Рвуз1
	PackRs6[10]  = (Pvir3  - x0_p5[6])/z_p5[6] + add;											// Верхний уровень заряженности для восстановления НВАБ
	PackRs6[11]  = (Tvir3  - x0_p5[7])/z_p5[7] + add;											// Предельно допустимая температура НВАБ при уровне заряженности Рвуз8
	PackRs6[12]  = (Pnuz   - x0_p5[8])/z_p5[8] + add;											// Верхний уровень заряженности НВАБ
	PackRs6[13]  = (PvuzCK - x0_p5[9])/z_p5[9] + add;											// Верхний уровень заряженности НВАБ
	PackRs6[14]  = (TnuzAB - x0_p5[10])/z_p5[10] + add;										// Предельно допустимая температура НВАБ при уровне заряженности Рвуз8
	PackRs6[15]  = (TvuzCK - x0_p5[11])/z_p5[11] + add;										// Верхний уровень заряженности НВАБ
	PackRs6[16]  = (Pn 		 - x0_p5[12])/z_p5[12] + add;										// Давление нижнего уровня заряженности НВАБ при ТВЦ,  (40-50)
	PackRs6[17]  = (Pv 		 - x0_p5[13])/z_p5[13] + add;										// Давление верхнего уровня заряженности НВАБ при ТВЦ, (46-54)
	PackRs6[18]  = (Tn3 	 - x0_p5[14])/z_p5[14] + add;										// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности Р<0,8Pн,  (42-45)
	PackRs6[19]  = (Tn2 	 - x0_p5[15])/z_p5[15] + add;										// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности 0,8Pн<Р<Pн, (28-32)
	PackRs6[20]  = (Tn1 	 - x0_p5[16])/z_p5[16] + add;										// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности Pн<Р<Pв, (18-22)
	PackRs6[21]  = (Tk 		 - x0_p5[17])/z_p5[17] + add;										// Tv Предельно допустимая температура НВАБ при включении компенсационного подзаряда (22-28)
	PackRs6[22]  = (tVir/60- x0_p5[18])/z_p5[18] + add;										// Время выравнивания заряженности НВАБ при ТВЦ, час  
	PackRs6[23]  = (Pvz 	 - x0_p5[19])/z_p5[19] + add;										// Среднее давление НВА при ТВЦ после подзаряда
	PackRs6[24]  = (Traz 	 - x0_p5[20])/z_p5[20] + add;										// Предельно допустимая температура в ТВЦ НВАБ при разряде (42-45)
	PackRs6[25]  = (P0		 - x0_p5[21])/z_p5[21] + add;										// Среднее давление НВА при прекращении  тестового разряда НВАБ (НАБ (76+-1В)
	PackRs6[26]  = (tRazr/60-x0_p5[22])/z_p5[22] + add;										// Время разряда на разрядное сопротивление, час (10-50)
	PackRs6[27]  = (kWtst	 - x0_p5[23])/z_p5[23] + add;										// Коэфф. для расчёта текущего значения энергоёмкости
	PackRs6[28]  = (tP_TVC - x0_p5[24])/z_p5[24] + add;										// Время разряда на разрядное сопротивление, час (10-50)
	
	for(i=0; i < 5; i++)		{
		PackRs6[29+i] = nBadAk[i];																					// Номера отказавших аккумуляторов от БЦУ
		//PackRs6[29+i] = nBadAkBE[i];																				// Номера отказавших аккумуляторов из БЭ
	}

//	MakePackTst(PackRs6, lngPackRs6);
	checksumCalc = Crc16(PackRs6, lngPackRs6-2);													// Выисление контрольной суммы
	*(PackRs6+lngPackRs6-1) =  checksumCalc;	
	*(PackRs6+lngPackRs6-2) =  checksumCalc >> 8;													// Добавить контрольную сумму
}

//-------------------------------------------------------------------------------------------------------------------------
void MakePack7(void)	// Заполнение пакета 7	
{	float tMin;
	/*
1	7						Запрос на восстановление данных, ЗВД	1/0
	6, 5, 4			Резерв	
	3, 2, 1, 0	Этап проведения ТВЦ, ЭТВЦ	Код:
								0 – нет ТВЦ;
								1 – тестовый (принудительный) заряд АБ;
								2 – тестовый (принудительный) разряд;
								3 – пауза 2 часа;
								4 – разряд АБ на РС;
								5 – компенсационный заряд АБ током 1,5 А после разряда АБ на РС;
								6 - тестовый (принудительный) заряд АБ;
								7 – компенсационный заряд АБ током 1,5 А; 
								8 - тестовый (принудительный) разряд АБ;
								9 - принудительный заряд АБ до уставки РВУЗ2.
2							Энергоемкость АБ при разряде, в том числе в ТВЦ			ufix8,z = 15.7 Вт*ч, x0 = 0 Вт*ч
3							Емкость АБ при разряде,  в том числе в ТВЦ					ufix8,z = 0.2 А*ч, x0 = 0 А*ч
4							Текущее время от начала этапа 											ufix8,z = 0.283 ч, x0 = 0 ч
*/

	PackRs7[4] = (bRestData<<7) | statTVC;																// Запрос на восстановление данных, ЗВД	1/0
	PackRs7[5] = (int) ((W_raz - x0_p7[0])/z_p7[0]+0.5);									// Энергоемкость АБ при разряде, в том числе в ТВЦ
	PackRs7[6] = (int) ((C_raz - x0_p7[1])/z_p7[1]+0.5);									// Емкость АБ при разряде,  в том числе в ТВЦ

	tMin = (float)(tstatTVC/60);																					// Текущее время от начала этапа в минутах
	PackRs7[7] = (tMin - x0_p7[2])/z_p7[2];																// Текущее время от начала этапа

	checksumCalc = Crc16(PackRs7, lngPackRs7-2);													// Выисление контрольной суммы
	*(PackRs7+lngPackRs7-1) =  checksumCalc;	
	*(PackRs7+lngPackRs7-2) =  checksumCalc >> 8;													// Добавить контрольную сумму
}

//-------------------------------------------------------------------------------------------------------------------------
void MakePack8(void)	// Получение данных восстановления пакета 8	
{	float tHour;
	statTVC 	= 0xf & *(p_InPack);
	W_raz 	 	= *(p_InPack+1)*z_p7[0]+x0_p7[0];														// Энергоемкость АБ при разряде, в том числе в ТВЦ	// Верхний уровень заряженности НВАБ
	C_raz 	 	= *(p_InPack+2)*z_p7[1]+x0_p7[1];														// Емкость АБ при разряде,  в том числе в ТВЦ
	tHour		 	= *(p_InPack+3)*z_p7[2]+x0_p7[2];														// Текущее время от начала этапа в минутах
	tstatTVC 	= (int)tHour*60;																						// Текущее время от начала этапа в секундах

	if (statTVC)	{
		pOtkl_KOMP();																												// "ВКЛ КОМП"	«ВКЛ КОМП» = 1,
		pVkl_Zapr_Zarayd();																									// «ЗАПРЕТ ЗАРЯД» = 1, 
		pOtkl_Test_Zarayd();																								// «ОТКЛ ТЕСТ ЗАРЯД»,
		pVkl_Zapr_Razrayd();																								// «ЗАПРЕТ РАЗРЯД» = 1,
		pOtkl_Test_Razrayd();																								// «ОТКЛ ТЕСТ РАЗРЯД»,
		
		//сбрасываем все биты, участвующие в синхронизации
		stat3[iMUK_ZRU] &= ~bready;
		stat3[iMUK_ZRU] &= ~bZaprZar;
		stat3[iMUK_ZRU] &= ~bZaprRazr;
		stat4[iMUK_ZRU] = 0;	
		
		switch (statTVC)	{																									// 
		case 	1:	StepAlgortm = bVkl_Tst_Zarayd;	cntRazr = 1;
							break;
		case	2:	StepAlgortm = bVkl_Test_Razr;		cntRazr = 1;
							break;
		case 	3:	
							// Следующий этап выбирается исходя из необходимости: либо происходят наземные испытания, либо полет
							#ifdef HOURS2 
								StepAlgortm = bWait_2; //Наземные испытания, ждем 2 часа
								sCount_2h = tstatTVC;			
							#else					
								StepAlgortm = bTst_T_NVAB; // Полет, идем на проверку температуры
							#endif	
							cntRazr = 2;	
							break;
		case 	4:	StepAlgortm = bVkl_RS;	cntRazr = 2;			
							stat4[iMUK_ZRU] |= bready4;
							break;
		case	5:	StepAlgortm = bVklKomp;	cntRazr = 2;			
							break;
		case	6:	StepAlgortm = bTest_Vosst_et6_command;	cntRazr = 2;	//идем на вспомогательный шаг, так как нужно сначала подать команды
							break;
		case	7:	StepAlgortm = bTest_Vosst_et7_command;	cntRazr = 2;	//идем на вспомогательный шаг, так как нужно сначала подать команды		
							break;
		case	8:	StepAlgortm = bVkl_Test_Razr;		cntRazr = 3;
							C_raz = 0;		W_raz = 0;			time_Razr = 0;	
							break;
		case	9:	StepAlgortm = bVkl_Tst_Zarayd9;
							break;
		}
		mode = TEST;
		stat1[iMUK_ZRU] &= ~bMain;
		stat1[iMUK_ZRU] |= bTest;
		
		//Обязательно нужно следить, чтобы не было одновременно двух пауз bPauza_TVC=1 и bPauza = 1, иначе в функции счета счетчик будет увеличиваться два раза
		LimsCount = vsCount20;	sCount=0;		bPauza_TVC=1;										// Активация паузы 20 сек
		//stat3[iMUK_ZRU] = 0;
	}
}

//-------------------------------------------------------------------------------------------------------------------------
void MakePack9(void)	// Заполнение пакета 9	
{	int tmp;	//PvzRas = 68;

//	PackRs9[4] = (int) ((PvzRas -x0_p9[0])/z_p9[0]+0.5);								// Расчетное среднее давление в АК для АБ в режиме ТВЦ, в конце компенсационного заряда
	if (PvzRas >= x0_p9[0])		{	
		tmp = (int) ((PvzRas -x0_p9[0])/z_p9[0]+0.5);
		if (tmp>255)	PackRs9[4] = 255;
		else					PackRs9[4] = tmp;
	}	
	else	PackRs9[4] = 0;
	

//	PackRs9[5] = (int) ((P0_Ras -x0_p9[1])/z_p9[1]+0.5);								// Рассчитанное среднее давление в АК в режиме ТВЦ при прекращении разряда АБ
	if (P0_Ras >= x0_p9[1])		{																	
		tmp = (int) ((P0_Ras -x0_p9[1])/z_p9[1]+0.5);
		if (tmp>255)	PackRs9[5] = 255;											
		else					PackRs9[5] = tmp;
	}	
	else	PackRs9[5] = 0;

	checksumCalc = Crc16(PackRs9, lngPackRs9-2);													// Выисление контрольной суммы
	*(PackRs9+lngPackRs9-1) =  checksumCalc;	
	*(PackRs9+lngPackRs9-2) =  checksumCalc >> 8;													// Добавить контрольную сумму
}

//-------------------------------------------------------------------------------------------------------------------------
// Отправка пакетов по CAN (имитация 3-х МУКов)
//void CAN_Cmd_Tst (void)
//{	//int c;
//	add_nbuf=0;
//	CAN_SendCmd(AdrMUK1_ZRU, 0, CAN_Otkl_RS);		add_nbuf=1;	//for(c=0; c < 1500; c++)		{ nBadAk[0]=nBadAk[0];	}
//	CAN_SendCmd(AdrMUK2_ZRU, 0, CAN_Otkl_RS);		add_nbuf=2;	//for(c=0; c < 1500; c++)		{ nBadAk[0]=nBadAk[0];	}
//	CAN_SendCmd(AdrMUK3_ZRU, 0, CAN_Otkl_RS);
//}	

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Получение среднего значения параметра измерения в вольтах по nReadADC точкам
void bOkDataADC (void)
{	int ind;

	OkResult = 0;
	iReadAdc++;																														// 

	if (iReadAdc < nReadADC)	{																						// Промежуточный массив значений текущего канала заполнен
		if (Result>0)	{	
			summa+=Result;	NumReadAdc++;		}																	// Допустимый результат
		MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO & ADC1_REG_GO;									// Повторить измерение
	}
	
	else	{
		if ((NumReadAdc)&&(summa)) {
			Result = summa/NumReadAdc;		ind = nMUK_ZRU-nMUK1_ZRU;
			if (iadc)	{	Uadc = ((Result)*(AUcc[ind]/0xfff));					}				// Пересчёт значения АЦП в вольты
			else			{	AUcc[ind] = (Uref[ind]*0xfff)/(Result);				}				// Коррекция AUcc
		}	
		else	{	Uadc=0;	}																										// Канал вышел из строя
		OkDataADC = 1;	summa=0;	NumReadAdc=0;	 iReadAdc=0;
	}	
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Zaryd_NVAB (void)													/* _З_А_Р_Я_Д___Н_В_А_Б_ */
{

	switch (StepAlgortmZar)																								// Обработчик состояний алгоритма заряда
	{
	// .......... Инициализация алгоритма заряда ............................................................................
	case bInitZarayd:
	//обнаружилось, что обнулять флаги в начале алгоритма некорректно - иногда мы сюда попадаем не в момент перехода из заряда в разряд (и наоборот)
	//поэтому обнуляем флаги в функции PutParamADC в момент перехода из режима в режим		
	//	stat2[iMUK_ZRU] = 0;
	//stat3[iMUK_ZRU] &= ~(errNoOgrTokRazr|errNoOgrTokZar|errPrevDopustT);
		
	// .......... Отключение КОМП ...........................................................................................
	case bOtklKomp:
		
		pOtkl_KOMP();
		//LimsCount = vsCount5;	sCount=0;		bPauza=1;													// Активация паузы 5 сек
		bPauza=0;																														// Нет паузы 5 сек
//		StepAlgortmZar = bWaitOtklKomp;
//		break;
	
	// .......... Ожидание отключения КОМП Заряда ...........................................................................
	case bWaitOtklKomp:		

		//if (!bPauza) {
			//if	(aI_zar <= aIkomp)	stat2[iMUK_ZRU] |= errNoOtklCompZar;			// Собщение "Не отключился КОМП Заряд АБ"=1
			StepAlgortmZar = bTst_P_NVAB;
		//}
		break;
		
	// .......... Проверка давления АБ перед зарядом ........................................................................
	case bTst_P_NVAB:

		if (!bPauza) {
			if	(P <= 0.95*Pu[iUst])	{																				// Включение заряда
				if ((iUst==2)&&(T >= Tu[2])&&(P >= 0.5*Pu[2]))	iUst=1;					// N=2
				pOtkl_Zapr_Zarayd ();																						// Запрет заряда=0
// удаляем сообщения об ошибках из мест, где им не место					
//				stat2[iMUK_ZRU] &= ~errNoVklZar;																// Собщение "Не включился заряд АБ"=0			
				stat2[iMUK_ZRU] &= ~errNoOtklZar;																// Собщение "Не отключился заряд АБ"=0	
				LimsCount = vsCount5;		sCount=0;		bPauza=1;										// Активация паузы 5 сек
				StepAlgortmZar = bWaitOtkl_ZaprZar;
			}	
			else	{																														// Ожинание уменьшения давления
				pVkl_Zapr_Zarayd ();																						// Запрет заряда=1
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20 сек
			}
		}
		break;

	// .......... Ожидание отключения запрета Заряда ........................................................................
	case bWaitOtkl_ZaprZar:		

		if (!bPauza) {
// удаляем сообщения об ошибках из мест, где им не место			
//			if	((aI_zar < aIkomp)&&
//					 (stat1[iMUK_ZRU] & bZaryad))	stat2[iMUK_ZRU] |= errNoVklZar;	// Собщение "Не включился Заряд АБ"=1
			LimsCount = vsCount20;	sCount=0;		bPauza=1;											// Активация паузы 20 сек
			StepAlgortmZar = bVkl_Zarayd;
					
		}
		break;
		
	// .......... Разрешен заряд НВАБ .......................................................................................
	case bVkl_Zarayd:																											// ОтклЗапЗар

		if (!bPauza) {
			if (aI_zar <= aIzard) {	stat3[iMUK_ZRU] &= ~errNoOgrTokZar;				// Собщение "Не ограничен ток заряда"=0, aIzard = 25
				StepAlgortmZar = bTst_T_NVAB;																		// След шаг алгоритма
			}	
			else	{									stat3[iMUK_ZRU] |= errNoOgrTokZar;				// Собщение "Не ограничен ток заряда"=1
				//StepAlgortmZar = HARD_ERR;																			// ОТКЛ ЗРУ
			}
			StepAlgortmZar = bTst_T_NVAB;																			// След шаг алгоритма
		}
		break;

	// .......... Проверка превышения допустимой температуры ................................................................
	case bTst_T_NVAB:																											// ОтклЗапЗар

		if	(T >= T_max)	{																									// Отключаем заряд. Вкл_КОМП = ОТКЛ ЗАРЯД
			stat3[iMUK_ZRU] |= errPrevDopustT;																// "Превышение допустимой температуры НВАБ"=1
			if	(P >= 0.8*Pu[iUst])	{																					// Отключаем заряд. Вкл_КОМП = ОТКЛ ЗАРЯД
				pVkl_Zapr_Zarayd ();																						// Запрет заряда=1
				LimsCount = vsCount5;	sCount=0;		bPauza=1;											// Активация паузы 5 сек
				StepAlgortmZar = bWaitVkl_ZaprZara;															// След шаг алгоритма Ожидание включения запрета заряда
			}	
			else	{																														// Не включился заряд.
				StepAlgortmZar = bVkl_Zarayd_On;																// След шаг алгоритма
			}	
		}
		else	{																															// Не включился заряд.
				StepAlgortmZar = bVkl_Zarayd_On;																// След шаг алгоритма
		}
		break;

	// .......... Ожидание включения запрета Заряда ...........................................................................
	case bWaitVkl_ZaprZara:		

		if (!bPauza) {
			if	(aI_zar > aIporog)	stat2[iMUK_ZRU] |= errNoOtklZar;					// Собщение "Не отключился Заряд АБ"=1
			LimsCount = vsCount20;	sCount=0;		bPauza=1;											// Активация паузы 20сек
			StepAlgortmZar = bWaitTmax_2;																			// След шаг алгоритма Ожидание нормализации температуры
		}
		break;
		
	// .......... Заряжаем батарею ..........................................................................................
	case bVkl_Zarayd_On:

		if	(P >= Pu[iUst])	{																								// Отключаем заряд. Вкл_КОМП = ОТКЛ ЗАРЯД
			pVkl_Zapr_Zarayd ();																							// Включить Запрет заряда=1
			LimsCount = vsCount20;	sCount=0;		bPauza=1;											// Активация паузы 20сек
			StepAlgortmZar = bVklKomp;
		}	
		else	{
			if (mode_Razryad)		StepAlgortmZar = bInitZarayd;									//
			else								StepAlgortmZar = bVkl_Zarayd;									// "Петля" - процесс заряда
		}
		break;

	// .......... Проверка значения температуры НВАБ ........................................................................ 
	case bWaitTmax_2:
		
		if (!bPauza) {
			if	(T <= T_max-2)	{																							// Отключаем заряд. Вкл_КОМП = ОТКЛ ЗАРЯД
				stat3[iMUK_ZRU] &= ~errPrevDopustT;															// "Превышение допустимой температуры НВАБ"=0
				pOtkl_Zapr_Zarayd ();																						// Запрет заряда=0
				LimsCount = vsCount5;	sCount=0;		bPauza=1;											// Активация паузы 20сек
				StepAlgortmZar = bWaitOtkl_ZaprZar2;														// След шаг алгоритма Ожидание отключения запрета заряда
			}
			else	{	StepAlgortmZar = bWaitTmax_2;															// След шаг алгоритма
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20сек
			}	
		}
		break;

	// .......... Ожидание отключения запрета Заряда ........................................................................
	case bWaitOtkl_ZaprZar2:		

		if (!bPauza) {
// удаляем сообщения об ошибках из мест, где им не место				
//			if	(aI_zar < aIkomp)	stat2[iMUK_ZRU] |= errNoVklZar;							// Собщение "Не включился Заряд АБ"=1
			LimsCount = vsCount20;	sCount=0;		bPauza=1;											// Активация паузы 20 сек
			StepAlgortmZar = bVkl_Zarayd_On;
		}
		break;
		
		// .......... Ток заряда упал до 2А .....................................................................................
	case bVklKomp:		

		if (!bPauza) {
			if	(aI_zar > aIporog)	stat2[iMUK_ZRU] |= errNoOtklZar;						// Собщение "Не отключился заряд АБ"=1
			StepAlgortmZar = bViborUst;
		}
		break;
		
	// .......... Выбор уставок температуры и давления.......................................................................
	case bViborUst:

		if	((dP >= dP_u)||(dUak>=dU_u))	iUst=2;														// N=3
		else	{
			if	(T >= Tu[1])								iUst=0;														// N=1
			else														iUst=1;														// N=2
		}	
		StepAlgortmZar = bInitZarayd;
		break;

	// .......... Ошибка аппаратуры модуля ..................................................................................
//	case HARD_ERR:					
//		
//		pOtkl_Shim_ZRU(0);
//		mode=0;																															// 
//		break;
	
	default:
		StepAlgortmRazr = bInitZarayd;																			// Переход на начало алгоритма
		break;
		
	// .......... Конец алгоритма ...........................................................................................
//	case bEnd_Alg_Zarayda:		
//		StepAlgortmZar = bInitZarayd;
	} //end of switch (StepAlgortm)
	
}	// End of Zaryd_NVAB


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Zaryd_NVAB_noCAN (void)											/* _З_А_Р_Я_Д___Н_В_А_Б_ по проводным командам */
{

	switch (StepAlgortmZar)																								// Обработчик состояний алгоритма заряда
	{
	// .......... Инициализация алгоритма заряда ............................................................................
	case bInitZarayd:
//обнаружилось, что обнулять флаги в начале алгоритма некорректно - иногда мы сюда попадаем не в момент перехода из заряда в разряд (и наоборот)
//поэтому обнуляем флаги в функции PutParamADC в момент перехода из режима в режим				
		//stat2[iMUK_ZRU] &= ~errNoOtklCompZar;
//		StepAlgortmZar = bOtklKomp;
//		break;
		
	// .......... Отключение КОМП ...........................................................................................
	case bOtklKomp:
		
		pOtkl_KOMP();
		//LimsCount = vsCount5;	sCount=0;		bPauza=1;													// Активация паузы 5 сек
		//StepAlgortmZar = bWaitOtklKomp;
		StepAlgortmZar = bTst_P_NVAB;
		break;
	
	// .......... Ожидание отключения КОМП Заряда ...........................................................................
//	case bWaitOtklKomp:		

//		if (!bPauza) {
//			if	(aI_zar <= aIkomp)	stat2[iMUK_ZRU] |= errNoOtklCompZar;			// Собщение "Не отключился КОМП Заряд АБ"=1
//			StepAlgortmZar = bTst_P_NVAB;
//		}
//		break;
		
	// .......... Ожидание снятия запрета заряда ............................................................................
	case bTst_P_NVAB:
		if	(!bitNotZar)	{																								// Включение заряда
			pOtkl_Zapr_Zarayd ();																						// Запрет заряда=0
// удаляем сообщения об ошибках из мест, где им не место	
//				stat2[iMUK_ZRU] &= ~errNoVklZar;																// Собщение "Не включился заряд АБ"=0
			LimsCount = vsCount5;	sCount=0;		bPauza=1;											// Активация паузы 5 сек
			StepAlgortmZar = bWaitOtkl_ZaprZar;
		}	
		else	{																														// Ожинание снятия Запрета Заряда
			LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20 сек
		}
		break;

	// .......... Ожидание отключения запрета Заряда ........................................................................
	case bWaitOtkl_ZaprZar:		

		if (!bPauza) {
// удаляем сообщения об ошибках из мест, где им не место	
//			if	(aI_zar < aIkomp)	stat2[iMUK_ZRU] |= errNoVklZar;							// Собщение "Не включился Заряд АБ"=1
			LimsCount = vsCount20;	sCount=0;		bPauza=1;											// Активация паузы 20 сек
			StepAlgortmZar = bVkl_Zarayd;
		}
		break;
		
	// .......... Разрешен заряд НВАБ .......................................................................................
	case bVkl_Zarayd:																											// ОтклЗапЗар

		if (!bPauza) {
			if (aI_zar <= aIzard) {	stat3[iMUK_ZRU] &= ~errNoOgrTokZar;				// Собщение "Не ограничен ток заряда"=0, aIzard = 25
				StepAlgortmZar = bTst_T_NVAB;																		// След шаг алгоритма
			}	
			else	{									stat3[iMUK_ZRU] |= errNoOgrTokZar;				// Собщение "Не ограничен ток заряда"=1
				//StepAlgortmZar = HARD_ERR;																			// ОТКЛ ЗРУ
			}
			StepAlgortmZar = bTst_T_NVAB;																			// След шаг алгоритма
		}
		break;

	// .......... Проверка запрета заряда ...................................................................................
	case bTst_T_NVAB:																											// ОтклЗапЗар

		if	(bitNotZar)	{																										// Отключаем заряд. Вкл_КОМП = ОТКЛ ЗАРЯД
				pVkl_Zapr_Zarayd ();																						// Запрет заряда=1
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20 сек
				StepAlgortmZar = bWaitVkl_ZaprZara;															// След шаг алгоритма Ожидание включения запрета заряда
		}
		else	{																															// Не включился заряд.
			if (mode_Razryad)		StepAlgortmZar = bInitZarayd;		// Выход из "Петли"
			else	{														StepAlgortmZar = bVkl_Zarayd;		// "Петля" - процесс заряда
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20 сек
			}
		}
		break;

	// .......... Ожидание включения запрета Заряда ...........................................................................
	case bWaitVkl_ZaprZara:		

		if (!bPauza) {
			if	(aI_zar > aIporog)	stat2[iMUK_ZRU] |= errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=1
			StepAlgortmZar = bViborUst;																				// След шаг алгоритма Ожидание нормализации температуры
		}
		break;

	// .......... Выбор уставок температуры и давления.......................................................................
	case bViborUst:

		iUst=1;
		StepAlgortmZar = bInitZarayd;
		break;

	// .......... Ошибка аппаратуры модуля ..................................................................................
//	case HARD_ERR:					
//		
//		pOtkl_Shim_ZRU(0);
//		mode=0;																															// 
//		break;
	
	default:
		StepAlgortmRazr = bInitZarayd;																			// Переход на начало алгоритма
		break;
		
	// .......... Конец алгоритма ...........................................................................................
//	case bEnd_Alg_Zarayda:		
//		StepAlgortmZar = bInitZarayd;
	} //end of switch (StepAlgortm)
	
}	// End of Zaryd_NVAB


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Razryd_NVAB (void)													/* _Р_А_З_Р_Я_Д___Н_В_А_Б_ */
{	//unsigned char bLogic;

	switch (StepAlgortmRazr)																							// Переключатель состояний алгоритма разряда
	{

	// .......... Инициализация разряда .....................................................................................
	case bInitRazryda:
		pOtkl_Zapr_Razrayd();
		stat2[iMUK_ZRU] &= ~errNoOtklRazr; 																	//Не отключился разряд АБ = 0
	// Запрет РАЗРЯДА = 0   firstInRaz = 0;
	//обнаружилось, что обнулять флаги в начале алгоритма некорректно - иногда мы сюда попадаем не в момент перехода из режима в режим
	//поэтому обнуляем флаги в функции PutParamADC в момент перехода из режима в режим		
	//	stat2[iMUK_ZRU] = 0;
	//	stat3[iMUK_ZRU] &= ~(errNoOgrTokRazr|errNoOgrTokZar|errPrevDopustT);
		StepAlgortmRazr = bWaitOtkl_ZaprRaz;
		break;

	// .......... Ожидание отключения запрета Разряда ........................................................................
	case bWaitOtkl_ZaprRaz:		
	//обнаружилось, что обнулять флаги в начале алгоритма некорректно - иногда мы сюда попадаем не в момент перехода из режима в режим
	//поэтому обнуляем флаги в функции PutParamADC в момент перехода из режима в режим			
	//		stat3[iMUK_ZRU] &= ~(errNoOgrTokRazr | errPrevDopustT | errNoOtklRazr);
			StepAlgortmRazr = bTst_I_Razryda;																	// Переход на разряд с подсчётом C и W
		break;
		
	// .......... Проврка ограничения тока разряда ..........................................................................
	case bTst_I_Razryda:
		
		if (aI_razr > aIrazr_ogrn) {stat3[iMUK_ZRU] |= errNoOgrTokRazr;			// aIrazr_ogrn = 30 Собщение "Не ограничен ток разряда"
			//StepAlgortmRazr = HARD_ERR;																				// Переход на ОТКЛ ЗРУ
		}	
		else											 {stat3[iMUK_ZRU] &= ~errNoOgrTokRazr;
		}	
		LimsCount_R = dt5; 	 sCount_R=0;	bPauza_R=1;												/*Razr;*/
		calc_dt = calc_dt5; //дельта времени соответствует 5 секундам
		Uab_old = Uab;		aI_razrOld = aI_razr;
		StepAlgortmRazr = bTst_U_Razryda;																		// Переход на разряд с подсчётом C и W
		break;

	// .......... Р_а_з_р_я_д АБ, контроль напряжения АБ ....................................................................
	case bTst_U_Razryda:

		if (!bPauza_R) {																										// Пауза 5 сек
			
			if	((Uab <= 72)||(Umin_ak <= 0.2))	{	
				StepAlgortmRazr = bOtkl_Razrayd;
			}	
			else	{																														// ((P < 3)||(U <= 76))
				Calculation();
				if	(T >= 50)	
					stat3[iMUK_ZRU] |=  errPrevDopustT;								// "Превышение допустимой температуры АБ" - процесс разряда
				else					
					stat3[iMUK_ZRU] &= ~errPrevDopustT;
				//LimsCount = vsCount20;	 sCount=0;	bPauza=1;
				StepAlgortmRazr = bInitRazryda;
			}	
		}	
		break;

	// .......... Отключаем разряд ..........................................................................................
	case bOtkl_Razrayd:
		
	  pVkl_Zapr_Razrayd();																								// Запрет РАЗРЯДА = 1
		LimsCount_R = vsCount20;	 sCount_R=0;	bPauza_R=1;									// Сброс счётчика , включене паузы 20 сек
		StepAlgortmRazr = bTst_I_Comp2;
		break;

	// .......... Проврка ограничения тока разряда ..........................................................................
	case bTst_I_Comp2:
		
		if (!bPauza_R) {
			if (aI_razr > aIporog) {stat2[iMUK_ZRU] |= errNoOtklRazr;					// aIkomp = 2
				//StepAlgortmRazr = HARD_ERR;																		// Переход на ОТКЛ ЗРУ
			}	
			else									{stat2[iMUK_ZRU] &= ~errNoOtklRazr;
			}	
			StepAlgortmRazr = bTst_U_Razryda_end;															// Переход на ожидание подъёма напряжения АБ до 80В
			LimsCount_R = vsCount20;	 sCount_R=0;	bPauza_R=1;								// Сброс счётчика , включене паузы 20 сек
		}	
		break;

	// .......... Проврка роста напряжения при запрете разряда ............................................................
	case bTst_U_Razryda_end:
		
		if (!bPauza_R) {
			if	(Uab >= 88)	{																									// Напряжение АБ достигло 88В
				StepAlgortmRazr = bInitRazryda;																	// Переход на начало алгоритма
			}	
			else	{																														//
				StepAlgortmRazr = bOtkl_Razrayd;																// Переход на запрт разряда
			}		
		}
		break;
	
	default:
		StepAlgortmRazr = bInitRazryda;																			// Переход на начало алгоритма
		break;
		
	// .......... Ошибка аппаратуры модуля ..................................................................................
//	case HARD_ERR:					
//		
//		pOtkl_Shim_ZRU(0);
//		mode=0;																															// 
//		break;
	
	} //end of switch (StatusZarRazr)
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Razryd_NVAB_noCAN (void)													/* _Р_А_З_Р_Я_Д___Н_В_А_Б_ по проводным командам */
{

	switch (StepAlgortmRazr)																							// Переключатель состояний алгоритма разряда
	{

	// .......... Инициализация разряда .....................................................................................
	case bInitRazryda:		
		StepAlgortmRazr = bUabCheck;
		break;

	// .......... Контроль Uаб .....................................................................................
	case bUabCheck:
			if	((aU_zru >= 88) && (!bitNotRaz))	{														// Напряжение АБ достигло 88В И нет запрета разряда ЗРП = 0	
				StepAlgortmRazr = bWaitPause;																		// Начинаем ждать 20 секунд
				LimsCount_R = vsCount20;	 sCount_R=0;	bPauza_R=1;							// Сброс счётчика , включене паузы 20 сек				
			}	
			else	{																														//
				StepAlgortmRazr = bOtkl_Razrayd;																// Переход на запрет разряда
			}				
		break;	
	
	// .......... Задержка ............................................................
	case bWaitPause:
		
		if (!bPauza_R) {																							// Если дождались 
			StepAlgortmRazr = bWaitOtkl_ZaprRaz;												// Переход на отключение запрета Разряда
		}
		break;			
			
	// .......... Ожидание отключения запрета Разряда ........................................................................
	case bWaitOtkl_ZaprRaz:	
			pOtkl_Zapr_Razrayd();		
			stat2[iMUK_ZRU] &= ~errNoOtklRazr; 															//Не отключился разряд АБ = 0	
			StepAlgortmRazr = bTst_I_Razryda;																	// Переход на разряд с подсчётом C и W
		break;
		
	// .......... Проврка ограничения тока разряда ..........................................................................
	case bTst_I_Razryda:
		
		if (aI_razr > aIrazr_ogrn) {stat3[iMUK_ZRU] |= errNoOgrTokRazr;			// aIrazr_ogrn = 30 Собщение "Не ограничен ток разряда"
		}	
		else											 {stat3[iMUK_ZRU] &= ~errNoOgrTokRazr;
		}	
		LimsCount_R = dt5; 	 sCount_R=0;	bPauza_R=1;												/*Razr;*/
		calc_dt = calc_dt5; //дельта времени соответствует 5 секундам		
		aU_zru_Old = aU_zru;		aI_razrOld = aI_razr;
		StepAlgortmRazr = bZRPCheck;																		// Переход на контроль ЗРП с подсчётом C и W
		break;

	// .......... Контроль ЗРП ....................................................................
	case bZRPCheck:

		if (!bPauza_R) {																										// Пауза 5 сек		
			if	(bitNotRaz)	{																									//	Если есть запрет разряда ЗРП = 1
				StepAlgortmRazr = bOtkl_Razrayd;
			}	
			else	{																														// ((P < 3)||(U <= 76))
				Calculation_noCAN();
				StepAlgortmRazr = bWaitOtkl_ZaprRaz;														// По сути возвращаемся в начало алгоритма
			}	
		}	
		break;

	// .......... Отключаем разряд ..........................................................................................
	case bOtkl_Razrayd:
		
	  pVkl_Zapr_Razrayd();																								// Запрет РАЗРЯДА = 1
		LimsCount_R = vsCount20;	 sCount_R=0;	bPauza_R=1;									// Сброс счётчика , включене паузы 20 сек
		StepAlgortmRazr = bTst_I_Comp2;
		break;

	// .......... Проврка ограничения тока разряда ..........................................................................
	case bTst_I_Comp2:
		
		if (!bPauza_R) {
			if (aI_razr > aIporog) {stat2[iMUK_ZRU] |= errNoOtklRazr;					// aIkomp = 2
			}	
			else									{stat2[iMUK_ZRU] &= ~errNoOtklRazr;
			}	
			StepAlgortmRazr = bUabCheck;															// Переход на ожидание подъёма напряжения АБ до 80В
		}	
		break;


	
	default:
		StepAlgortmRazr = bInitRazryda;																			// Переход на начало алгоритма
		break;

	} //end of switch (StatusZarRazr)
}

//-------------------------------------------------------------------------------------------------------------------------
void OneSecAdd (void)													/* Добавить секунду */
{	int i;
	AddSec = 0;		bOneSec = 1;
	
	//......................................................................
	if (vRestData)	vRestData--;
	
	//......................................................................
	if (stat1[iMUK_ZRU] & bTest)	tstatTVC++;															// Время этапа ТВЦ

	//......................................................................
	if (secUart1 < 1)	{secUart1++;	NoWrkUart1=0;}	
	else 	{	NoWrkUart1 = 1;
		RS_RECEIVE1;  ind_pack1 = 0;																					//UART1_Init(); Счётчик 2-х секунд Uart1, флаг достижения двух секунд
	}
	if (secUart2 < 1)	{	secUart2++;	NoWrkUart1=0;	}
	else 	{	NoWrkUart1 = 1;
		RS_RECEIVE2;	ind_pack2 = 0;			
	}
	
	//......................................................................
	for (i=0;i<6;i++)	{																										// Установка/сброс флагов неработоспособности CAN узлов
		if (cntSec_noCAN[i] > 4)	{cntSec_noCAN[i] = 0;	bNoWrkCAN |= 1<<i; }	// 0x3f;
		else											{cntSec_noCAN[i]++;	}											//bNoWrkCAN &= ~(1<<i);
	}

	//......................................................................
	sCount_2h++;
	
	//!!!нужно внимательно следить, чтобы bPauza_TVC и bPauza не были запущены одновременно, иначе sCount будет увеличиваться два раза
	//!!!да и LimsCount у них тоже общий. Либо нужно делать два независимых счетчика и два независимых порога для счетчиков
	//......................................................................
	if (bPauza_TVC)	{	sCount++;
		if (sCount >= LimsCount)	{ sCount = 0;	bPauza_TVC = 0;}	}							// Обслуживание паузы ТВЦ

	//......................................................................
	if (bPauza)	{	sCount++;
		if (sCount >= LimsCount)	{ sCount = 0;	bPauza = 0;}	}									// Обслуживание универсальной паузы

	//......................................................................
	if (bPauza_R)	{	sCount_R++;
		if (sCount_R >= LimsCount_R)	{ sCount_R = 0;	bPauza_R = 0;}	}					// Обслуживание паузы РАЗРЯДА
	
	//......................................................................
	mCountSecMain++;																											// Счётчик 1 мин
	if (mCountSecMain == 59)	{	mCountSecMain = 0;	mCount5Main++;						// Счётчик 5 мин в main
			time_Razr++;
	}
	//......................................................................
//	if (bPauza5)	{	sCount5++;
//		if (sCount5 >= 5)	{ sCount5=0;	bPauza5=0;}	}												// Обслуживание паузы 5 сек
	//......................................................................
	sTime.sec++;
  if (sTime.sec == 59)	{		sTime.sec = 0;																// Прошло 60 сек
		if (sTime.min < 59)		{	sTime.min++;	}															// mCount5Main++;}
		else	{									sTime.min = 0;	
			if (sTime.hour < 23)	{	sTime.hour++;	}
			else	{									sTime.hour = 0;		NewDay=1;								// Смена суток
				Date_Update();     																							// Increment the date
			}
		}	
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Инициализация переменных
void Var_init()
{	
	iMUK_ZRU = nMUK_ZRU-1;
	switch (iMUK_ZRU)	{																								// 
	case 0:	iMUK_ZRU2=1; iMUK_ZRU3=2;	break;													// 
	case 1:	iMUK_ZRU2=0; iMUK_ZRU3=2;	break;													// 
	case 2:	iMUK_ZRU2=0; iMUK_ZRU3=1;	break;													// 
	}
	
	iadc = 1;																															// Чтение АЦП начинаем с 1-го канала
	iReadAdc = 0;	 																												// Индекс рабочего массива измерения АЦП для получения среднего значения текущего канала измерения
	OkDataADC = 0;																												// Флаг "значение канала чтения АЦП отцифровано"
	ADC1_REG_GO = 2;																											// Запись “1” в 1-й бит MDR_ADC->ADC1_CFG начинает процесс преобразования, сбрасывается автоматически
	
	iUst = 1;																															// Индекс текущей (2-й) уставки
	
	stat1[iMUK_ZRU] = bMain;																							// Статус и сообщения ЗРУ stat1[3] = {РС | ЗРУ | Подзаряд| ТВЦ |Основной режим |Заряд  |Разряд};
	stat2[iMUK_ZRU] = 0;																									// Сообщения ЗРУ 					stat2[3], stat3[3];
	stat3[iMUK_ZRU] = 0;
	stat4[iMUK_ZRU] = 0;
	sync_AND_2 = 0; //служебная переменная для хранения результатов синхронизации, логические И двух других соседей (нужно оба других) 
	sync_OR_1  = 0; //служебная переменная для хранения результатов синхронизации, логические ИЛИ двух других соседей (достаточно хотя бы одного другого)

	bRestData = 1;																												// 1 - восстановить данные

	ind_pack1 = 0;						ind_pack2 = 0;
	lngPack1 = Npack_Cmd-1;		lngPack2 = Npack_Cmd-1;											// Длина пакета RS485 максимальная

	bPauza = 0;
	bPauza20 = 0;																													// Флаг пауза 20 сек
	bOneSec = 0;																													// Флаг для чтения параметров АБ в БЭ через 1 сек
	
	bmode_mod = 0;

	EndTVC = 0; 																														// Флаг начала процесса окончания ТВЦ

//	StepAlgortmZar = bInitZarayd;																					// StepAlgortmZar = bInitZarayd
//	StepAlgortmRazr = bInitRazryda;
}	

//-------------------------------------------------------------------------------------------------------------------------
void ZRU_Init(void)
{	unsigned char OnZRU;
	Init_Param();
	/*	ОТКЛ.АБ, ОТКЛ.СЭС, ВКЛ КОМП, ВКЛ ЗАПРЕТ ЗАРЯД, ВКЛ ЗАПРЕТ РАЗРЯД, ВЫКЛ ТЕСТ ЗАРЯД, ВЫКЛ ТЕСТ РАЗРЯД, ЗАПР ШИМ ЗРУ. №уст = 2 */
	
	pOtkl_KOMP();																													// «ВКЛ КОМП» = 1,
	pOtkl_Zapr_Zarayd();																									// «ЗАПРЕТ ЗАРЯД» = 0, 
	pOtkl_Zapr_Razrayd();																									// «ЗАПРЕТ РАЗРЯД» = 0,
	pOtkl_Test_Zarayd();																									// «ОТКЛ ТЕСТ ЗАРЯД»,
	pOtkl_Test_Razrayd();																									// «ОТКЛ ТЕСТ РАЗРЯД»,
	pOtkl_AB_CEC_Shim(0);																									// «Вкл АБ» = 0, «Подкл СЭС» = 0, «Разр ШИМ ЗРУ» = 0 
	pOtkl_RS(0);
	Wait(30);																															// Ожидание завершения коммутации силовый цепей

	Var_init();																														// Инициализация переменных

	OnZRU = 0x08 & MDR_PORTA->RXTX;																				// PA3 = 1, ЗРУ включено
	if (OnZRU)	pVkl_AB_CEC_Shim(0);																			// Подключить ЗРУ к АБ		22/PF3
	
	EnableIRQ_ADC_CAN_UART();
}

//-------------------------------------------------------------------------------------------------------------------------
// Отработка команды принятого пакета по RS-485 Uart1 и отправка пакета по RS-485 Uart1 в БЦУ
void WrkCmd_1(void)		
{	int i, j;
	unsigned char cmd;																														// cmd_curr, 

	switch (pack1[3])	{																														// Код пакета получено по RS-485
	case 	gStat_ZRU:			p_ParRs1 = PackRs1;	BatchSize1 = lngPackRs1;	break;		// телеметрия ЗРУ
		
	case	gCmd_for_ZRU:
						cmd	= 0x0f & *(pack1+4);																						// В 5-ом байте принятого пакета команда (код)
						//if (!bmode_mod) {	mode_last = mode;	bmode_mod = 1;	}
						switch (cmd)	{
						case gVkl_ZRU:				mode = Vkl_ZRU;  									 						// 0x1 Вкл_ЗРУ
											if (!tVkl_ZRU) tVkl_ZRU = 10;				break;								// 1 сек
						case gOtkl_ZRU:				mode = Otkl_ZRU;				break;			 					// 0x2 Откл_ЗРУ
						case gVkl_Test:	 			if(DataOk)	mode = initTEST;				break;		// 0x3 Вкл_ТЕСТ. Первый шаг алгоритма ТВЦ
						case gOtkl_Test: 			mode = Otkl_TEST;				break;								// 0x4 Откл_ТЕСТ
						case gOtkl_RS:	 			pOtkl_RS(0);						break;								// 0x5 ОТКЛ РС 	отключать разрядные сопротивления в БЭ mode = Otkl_RS;
						case gVkl_Podzarayd:	if(DataOk)	mode = initPodzarayd;		break;		// 0x6 ВКЛ Подзаряд
						case gOtkl_Podzarayd:	mode = Otkl_Podzarayd;	break;								// 0x7 ОТКЛ Подзаряд
						case OZVD:						bRestData = 0;					break;								// 0x8 «Отключение запроса на восстановление данных»
						}
												p_ParRs1 = PackRs2;	BatchSize1 = lngPackRs2;	break;
		
	case 	gStat_AB_short:	p_ParRs1 = PackRs3;	BatchSize1 = lngPackRs3;	break;		// Сост_АБ_Кратк_БЭ - краткая телеметрия БЭ

	case 	gStat_AB_Full:	p_ParRs1 = PackRs4;	BatchSize1 = lngPackRs4;	break;		// Сост_АБ_Полн_БЭ – полная телеметрия БЭ

	case	gUstavki_Curr:	bUstavkiBCU = 1;			p_InPack = pack1+4;								// Уставки_Текущ - управление БЭ
												p_ParRs1 = PackRs5;	BatchSize1 = lngPackRs5;	break;
		
	case	gUstavki_Tst:		p_ParRs1 = PackRs6;	BatchSize1 = lngPackRs6;	break;		// контроль параметров (уставок) алгоритмов ЗРУ

	case	gSaveData_to_BCU:
												p_ParRs1 = PackRs7;	BatchSize1 = lngPackRs7;	break;		// 0x7	запоминаемые для восстановления данные в БВС 
	case	gRestData_from_BCU:		bGetData = 1;		p_InPack = pack1+4;		
												p_ParRs1 = PackRs8;	BatchSize1 = lngPackRs8;	break;		// 0x8	данные для восстановления из БВС 
	case	gParamZRU:
												p_ParRs1 = PackRs9;	BatchSize1 = lngPackRs9;	break;		// 0x9	частные параметры ЗРУ 
	case	gTstLine:				
												p_ParRs1 = PackRs10;BatchSize1 = lngPackRs10;	break;		// 0xFF	проверка связи
	}

	RS_TRANSMIT1;																																	// Режим передатчика RS-485. Uart - OE=1
	for(i=0; i < waitOEon; i++)	ind_mas_trans1 = 0;																// Ожидание переключения OE после приёма на передачу
//	MDR_UART1->DR = *(p_ParRs1);																									//отправляем байт	UART2_trans_byte();
	
	for (i=0;	i < BatchSize1;	i++)	{	
		MDR_UART1->DR = (*p_ParRs1++);		j=0;																			// Послать байт
		while ((MDR_UART1->FR & UART_FR_BUSY) && (j<Transmit_wait)) {j++;	};				// Ожидание отправки байта TRANSMIT_WAIT; i = 162 UART_FR_TXFF
		//while ((MDR_UART1->FR & UART_FR_TXFF) && (j<Transmit_wait)) {j++;	};				// Ожидание отправки байта TRANSMIT_WAIT; i = 162 UART_FR_TXFF
	}

	//CAN2_TstMSG(1, BatchSize1, p_ParRs1-BatchSize1);

	for(i=0; i < waitOEoff; i++)	ind_mas_trans1 = 0;															// Ожидание передачи последнего байта
	RS_RECEIVE1																																		// Режим приёмника RS-485. Uart - OE=1
	BatchSize1 = 0;		
}


//-------------------------------------------------------------------------------------------------------------------------
// Отработка команды принятого пакета по RS-485 Uart2 и отправка пакета по RS-485 Uart2 в БЦУ
void WrkCmd_2(void)		
{	int i, j;
	unsigned char cmd;																														// cmd_curr, 

	switch (pack2[3])	{																														// Код пакета получено по RS-485
	case 	gStat_ZRU:			p_ParRs2 = PackRs1;	BatchSize2 = lngPackRs1;	break;		// телеметрия ЗРУ
		
	case	gCmd_for_ZRU:
						cmd	= 0x0f & *(pack2+4);																						// В 5-ом байте принятого пакета команда (код)
						//if (!bmode_mod) {	mode_last = mode;	bmode_mod = 1;	}
						switch (cmd)	{
						case gVkl_ZRU:				mode = Vkl_ZRU;  												 			// 0x1 Вкл_ЗРУ
																	if (!tVkl_ZRU) tVkl_ZRU = 10;				break;
						case gOtkl_ZRU:				mode = Otkl_ZRU;				break;			 					// 0x2 Откл_ЗРУ
						case gVkl_Test:	 			if(DataOk)	mode = initTEST;				break;		// 0x3 Вкл_ТЕСТ. Первый шаг алгоритма ТВЦ
						case gOtkl_Test: 			mode = Otkl_TEST;				break;								// 0x4 Откл_ТЕСТ
						case gOtkl_RS:	 			pOtkl_RS(0);						break;								// 0x5 ОТКЛ РС 	отключать разрядные сопротивления в БЭ mode = Otkl_RS;
						case gVkl_Podzarayd:	if(DataOk)	mode = initPodzarayd;		break;		// 0x6 ВКЛ Подзаряд
						case gOtkl_Podzarayd:	mode = Otkl_Podzarayd;	break;								// 0x7 ОТКЛ Подзаряд
						case OZVD:						bRestData = 0;					break;								// 0x8 «Отключение запроса на восстановление данных»
						}
												p_ParRs2 = PackRs2;	BatchSize2 = lngPackRs2;	break;
		
	case 	gStat_AB_short:	p_ParRs2 = PackRs3;	BatchSize2 = lngPackRs3;	break;		// Сост_АБ_Кратк_БЭ - краткая телеметрия БЭ

	case 	gStat_AB_Full:	p_ParRs2 = PackRs4;	BatchSize2 = lngPackRs4;	break;		// Сост_АБ_Полн_БЭ – полная телеметрия БЭ

	case	gUstavki_Curr:	bUstavkiBCU = 1;			p_InPack = pack2+4;								// Уставки_Текущ - управление БЭ
												p_ParRs2 = PackRs5;	BatchSize2 = lngPackRs5;	break;
		
	case	gUstavki_Tst:		p_ParRs2 = PackRs6;	BatchSize2 = lngPackRs6;	break;		// контроль параметров (уставок) алгоритмов ЗРУ

	case	gSaveData_to_BCU:
												p_ParRs2 = PackRs7;	BatchSize2 = lngPackRs7;	break;		// 0x7	запоминаемые для восстановления данные в БВС 
	case	gRestData_from_BCU:		bGetData = 1;		p_InPack = pack2+4;
												p_ParRs2 = PackRs8;	BatchSize2 = lngPackRs8;	break;		// 0x8	данные для восстановления из БВС 
	case	gParamZRU:
												p_ParRs2 = PackRs9;	BatchSize2 = lngPackRs9;	break;		// 0x9	частные параметры ЗРУ 
	case	gTstLine:				
												p_ParRs2 = PackRs10;BatchSize2 = lngPackRs10;	break;		// 0xFF	проверка связи
	}

	RS_TRANSMIT2;																																// Режим передатчика RS-485. Uart - OE=1
	for(i=0; i < waitOEon; i++)	ind_mas_trans2 = 0;															// Ожидание переключения OE после приёма на передачу
	//MDR_UART2->DR = *(p_ParRs2);																								//отправляем байт	UART2_trans_byte();

	for (i=0;	i < BatchSize2;	i++)	{	
		MDR_UART2->DR = (*p_ParRs2++);		j=0;																		// Послать байт
		while ((MDR_UART2->FR & UART_FR_BUSY) && (j<Transmit_wait)) {j++;	};			// Ожидание отправки байта TRANSMIT_WAIT; i = 162 UART_FR_TXFF
		//while ((MDR_UART2->FR & UART_FR_TXFF) && (j<Transmit_wait)) {j++;	};			// Ожидание отправки байта TRANSMIT_WAIT; i = 162 UART_FR_TXFF
	}

	for(i=0; i < waitOEoff; i++)	ind_mas_trans2 = 0;														// Ожидание передачи последнего байта
	RS_RECEIVE2																																	// Режим приёмника RS-485. Uart - OE=1
	BatchSize2 = 0;
}


//-------------------------------------------------------------------------------------------------------------------------
// Подготовка мажоритированных данных для телеметрии
unsigned int MajorStatZRU (unsigned char * stat)
{	unsigned int res=0;
	unsigned char is, st0, st1, st2;
	unsigned char bit1, bit2, maska;		

 for (is=0;is<8;is++)	{
	maska = 1<<is;

	st0 = (maska & stat[0])>>is;
	st1 = (maska & stat[1])>>is;
	st2 = (maska & stat[2])>>is;

	if ( (st0==st1) && (st0==st2) )	{
			bit1 = st0;
			bit2 = 0;}
	else	{
		bit1=0;
		if (st0==st1)	{		bit1 = st0;	}
		if (st0==st2)	{		bit1 = st0;	}
		if (st1==st2)	{		bit1 = st1;	}
		bit2=1;
	}		
	
	res |= (bit1<<(is*2)) | (bit2<<((is*2)+1));
 }
	return res;
}

//-------------------------------------------------------------------------------------------------------------------------
// Статус ЗРУ
//void StatZRU (void)
//{	unsigned char is, st0, st1, st2;
//	unsigned char maska;		

// for (is=0;is<8;is++)	{
//	maska = 1<<is;

//	st0 = maska & stat1[0];
//	st1 = maska & stat1[1];
//	st2 = maska & stat1[2];

//	if (st0==st1)			wrk_stat |= st0;
//	if (st0==st2)			wrk_stat |= st0;
//	if (st1==st2)			wrk_stat |= st1;
//	
// }
//}

/**************************************************************************************************************************
***************************************************************************************************************************/
int main(void)
{	int bZarRazr;//	unsigned int tmp;
	
	//__disable_irq(); 																											// Запрет Interrupts
	Clock_Init();																													// Инициализация тактового генератора
	Ports_Init();
//	Ports_Init_Tst(); 																									// Инициализация портов для отладочной платы
	ADC_Init();
	UART1_Init();	
	UART2_Init();	
 	CAN1_Init(); 
 	CAN2_Init();
//	__enable_irq ();																											// Глобальное разрешение прерываний
	SysTickInit(100000);
	//initInternalWatchdog();																								// Запуск сторожевого таймера
	
	ZRU_Init();																														// 

	mode = START;																													// Начальрый режим
	bPauza5 = 1;	bOneSec = 0;
	calc_dt = calc_dt5; 																									//по умолчанию дельта времени для расчета W и С будет соответствовать 5 секундам
	
	
	MakePack2_5_8_10();
	MakePack3();	MakePack4();
	MakePack1();	MakePack6();

	while (1)												
	{
	
		if (AddSec) OneSecAdd();																						// 
			
		//.....................................................................................................................
		if ((bRunCmdCAN)&&(!bTimeOutCmd))	{																	// Ожидание (400мс) подтверждения о получении команды БЭ и повтор команды
			if (cntTimeOut < nTimeOut)	{	cntTimeOut++;
				CAN_SendCmd(AdrMUK_ZRU, CurrentDlc, CurrentCmd);								// Повтор команды, которая не прошла в БЭ
				bTimeOutCmd = 1;
			}
			else	{		bRunCmdCAN = 0;		cntTimeOut = 0;	}											// Сброс активации команды
		}	
			
		//.....................................................................................................................
		if (bOneSec)	{		bOneSec=0;																				// Раз в секунду

			if (!mode)	{	mode = Init_Run;	}																	// Переход в рабочий режим при старте программы
			// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .	
			if	((updateD1)&&(updateD2))	{																		// Получены все фрейимы пакета1 и пакета2
				fDataConvert_();
				GetData();																											// Получить значения P, T, Usr, АБ из БЭ
				MakePack3();	MakePack4();																			// if ((!bReqBCU[0])&&(!bReqBCU[1]))	{	MakePack3();	MakePack4(); }
				cnt=0;	updateD1=0;		updateD2=0;
				if (mode == CAN_not_working)	{mode = Init_Run;}
				DataOk = 1;
			}
			// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
			else {																														// Данные телеметрии НЕ получены
				if (cnt<4)	{	cnt++;	}
				else	{																													// 5сек. нет связи по CAN
					ClearPack3();		ClearPack4();
					if ((mode != CAN_not_working)&&(mode != Vkl_ZRU))	{
						DataOk = 0;		 																								// Установка "Нет данных телеметрии АБ"
						if ((stat1[iMUK_ZRU] & bTest) || (stat1[iMUK_ZRU] & bPodzaryad))	{						
							pOtkl_Test_Zarayd();		pOtkl_Test_Razrayd();								// "Откл_ТЕСТ ЗАР"  "Откл_ТЕСТ РАЗР" 
							pOtkl_KOMP();																								// pVkl_Zapr_Zarayd ();		Запрет заряда=1			Откл_КОМП
							stat1[iMUK_ZRU] &= ~bTest;		statTVC = 0;									// перестаем находиться в режиме "тест" 
							stat1[iMUK_ZRU] &= ~bPodzaryad;															// перестаем находиться в режиме "подзаряд"
						}
						stat1[iMUK_ZRU] &= ~bPC;																			//сбрасываем состояние РС, ведь теперь мы его не знаем
						stat1[iMUK_ZRU] |= bMain;																			//в любом случае переходим в "основной режим работы"
						StepAlgortmZar = bInitZarayd;			StepAlgortmRazr = bInitRazryda;
						mode = CAN_not_working;																				// Сообщение в БЦУ "Ошибка приёма телеметрии АБ"
					}
				}
			}
			// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .			
			MakePack6();
			MakePack7();																											//if ((!(MDR_PORTA->RXTX & 0x20))&&(!(MDR_PORTF->RXTX & 0x04)))	{	MakePack7(); }
			MakePack9();
		}									

		//.....................................................................................................................
		//if (Errors)			{	pOtkl_Shim_ZRU(0);	mode=HARD_ERR;	}							// Ошибки аппаратуры: "ОТКЛ. ЗРУ" 

		//.......................................................... Контроль обновления данных телеметрии.....................
		if (nfRec_CanDatch1 == okFrameDatch)	{															// Получены все фреймы
			add_nbuf = 0;
			CAN_SendConf_1(1);																								// Подтверждение: Даные от МУК БЭ получены
			nfRec_CanDatch1 = 0;																							// Сброс битов получения фреймов
			updateD1 = 1;																											// Установить флаг "Обновление данных от МУК БЭ"
		}
		if (nfRec_CanAK1 == okFrameAK)	{																		// Получены все фреймы
			add_nbuf = 1;
			CAN_SendConf_1(2);																								// Подтверждение: Даные от МУК БЭ получены
			nfRec_CanAK1 = 0;																									// Сброс битов получения фреймов
			updateD2 = 1;																											// Установить флаг "Обновление данных от МУК БЭ"
		}
		
		if (nfRec_CanDatch2 == okFrameDatch)	{															// Получены все фреймы
			add_nbuf = 0;
			CAN_SendConf_2(1);																								// Подтверждение: Даные от МУК БЭ получены
			nfRec_CanDatch2 = 0;																							// Сброс битов получения фреймов
			updateD1 = 1;																											// Установить флаг "Обновление данных от МУК БЭ"
		}
		if (nfRec_CanAK2 == okFrameAK)	{																		// Получены все фреймы
			add_nbuf = 1;
			CAN_SendConf_2(2);																								// Подтверждение: Даные от МУК БЭ получены
			nfRec_CanAK2 = 0;																									// Сброс битов получения фреймов
			updateD2 = 1;																											// Установить флаг "Обновление данных от МУК БЭ"
		}

		//.....................................................................................................................
		switch (mode)																												// Обработчик состояний
		{
			case Init_Run:																										// Инициализация всех процессов при старте или сбоях
				Var_init();																											// Инициализация переменных
		
			case START:																												// Начальный запуск
				stat1[iMUK_ZRU] |= bMain;
				StepAlgortmZar = bInitZarayd;																		// StepAlgortmZar = bInitZarayd
				StepAlgortmRazr = bInitRazryda;
				mode = Work;
				break;
			
			case Work:																												// Штатная работа
				if (DataOk)	{
					bZarRazr = !bZarRazr;
					if (bZarRazr)		Zaryd_NVAB();
					else						Razryd_NVAB();
				}
				break;
			
			case Vkl_ZRU:																											// Подключить ЗРУ к АБ (включить силовые ключи ЗРУ)
				pVkl_Zapr_Zarayd();																							// «ЗАПРЕТ ЗАРЯД»  = 1
				if (tVkl_ZRU)	{		mode = Vkl_ZRU;	}
				else	{						mode = START;																	//Work;
					pVkl_AB_CEC_Shim(0);
				}	
				break;
			
			case Otkl_ZRU:																										// Отключить ЗРУ от АБ (отключить силовые ключи ЗРУ)
				pOtkl_AB_CEC_Shim(0);
				mode = Work;
				break;
			
			case initTEST:																										// Запуск подпрограммы ТВЦ АБ (определение ёмкости АБ) 
				if (!(stat1[iMUK_ZRU] & bTest))	{
					stat1[iMUK_ZRU] &= ~bMain;
					stat1[iMUK_ZRU] |= bTest;		StepAlgortm = bInit_TVC;
					bPauza_TVC = 0;	
					//сбрасываем все биты, участвующие в синхронизации
					//раньше сбрасывали весь байт stat3[iMUK_ZRU] = 0;
					//теперь сбрасываем конкретные биты, участвующие в ТВЦ, другие биты не трогаем
					stat3[iMUK_ZRU] &= ~bready;
					stat3[iMUK_ZRU] &= ~bZaprZar;
					stat3[iMUK_ZRU] &= ~bZaprRazr;
					stat4[iMUK_ZRU] = 0;	
					//----------------------------------------------------------------------------
				}
				mode = TEST;		
				
			case TEST:																												// Запуск подпрограммы ТВЦ АБ (определение ёмкости АБ) 
				if (!bPauza_TVC) {
					Test_NVAB();
				}
				break;
			
			case Otkl_TEST:	
				if(EndTVC) //если процесс окончания ТВЦ уже был запущен
				{					
					if (!bPauza_TVC) 	{ // то контролируем окончание задержки, прежде чем перейти в основной режим
						stat1[iMUK_ZRU] &= ~bTest;		statTVC = 0;  //мы закончили тестирование
						mode = START; //Начинаем обычный режим работы
						
						EndTVC = 0; // Процесс окончания ТВЦ закончен, мы переходим в режим mode = START;, можно сбросить этот флаг
					}							
				}
				else //нужно запустить процесс окончания ТВЦ: выполнить команды и установить паузу ожидания исполнения этих команд
				{
					LimsCount = vsCount20;	sCount=0;		bPauza_TVC=1;								// Подготовавливаем паузу 20 секунд			
					
					// По принятии команды Откл.Тест или при выходе из алгоритма тестирования вызываем эти функции
					pVkl_Zapr_Zarayd();																							// «ЗАПРЕТ ЗАРЯД»  = 1
					pVkl_Zapr_Razrayd();																						// «ЗАПРЕТ РАЗРЯД» = 1
					pOtkl_Test_Zarayd();																						// "Откл_ТЕСТ ЗАР" 
					pOtkl_Test_Razrayd();																						// "Откл_ТЕСТ РАЗР" 		

					EndTVC = 1; //Устанавливаем флаг того, что процесс окончания ТВЦ запущен
				}
	
				break;

			case initPodzarayd:																								// Запуск подпрограммы Подзаряд АБ
				if ((!(stat1[iMUK_ZRU] & bTest)) &&
						(!(stat1[iMUK_ZRU] & bPodzaryad)))	{						
					stat1[iMUK_ZRU] &= ~bMain;
					stat1[iMUK_ZRU] |= bPodzaryad;	StepAlgortm = bInitPodzaryd;
				}
				mode = Vkl_Podzarayd;
					
			case Vkl_Podzarayd:																								// Запуск подпрограммы Подзаряд АБ
				Podzarayd();
				break;
			
			case Otkl_Podzarayd:																							// Останов подпрограммы Подзаряд АБ
				stat1[iMUK_ZRU] &= ~bPodzaryad;
				mode = START;
				break;
			
			case ADC_ERR:																											// Повтор чтения текущего канала АЦП
//				ADC_Start(iadc);																								// Перезапуск опроса каналов АЦП
//				MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO;														// Запуск преобразования
				mode=Init_Run;																									// 
				break;
			
			case RESTART:																											// Рестарт UART и переход в прерванный режим
				mode=Init_Run;																									// 
				break;
			
			case CAN_not_working:																							// Отказ CAN
				pNotCan();
				bZarRazr = !bZarRazr;
				if (bZarRazr)		Zaryd_NVAB_noCAN();															//
				else						Razryd_NVAB_noCAN();														//
				break;
		} //end of switch (mode)
				
		//..............................................................................................................................
 		if (bReqBCU[0])		{																									// Запрос от Uart1 БЦУ
			checksumCalc = Crc16(pack1, lngPack1-2);													// Выисление контрольной суммы
			checksumIn = pack1[lngPack1-2];																		// Принятая контрольная сумма 
			checksumIn = (checksumIn<<8) | pack1[lngPack1-1];									// Принятая контрольная сумма 
			if (checksumCalc==checksumIn)	{
				WrkCmd_1();																											// Отработать команду от Uart1 БЦУ и отправить пакет по RS-485 в БЦУ
			}
			bReqBCU[0] = 0;		lngPack1 = Npack_Cmd-1;													// Длина пакета RS485 максимальная
		}

 		if (bReqBCU[1])		{	
			checksumCalc = Crc16(pack2, lngPack2-2);													// Выисление контрольной суммы
			checksumIn = pack2[lngPack2-2];																		// Принятая контрольная сумма 
			checksumIn = (checksumIn<<8) | pack2[lngPack2-1];									// Принятая контрольная сумма 
			if (checksumCalc==checksumIn)	{
				WrkCmd_2();																											// Отработать команду от Uart2 БЦУ и отправить пакет по RS-485 в БЦУ
			}
			bReqBCU[1] = 0;	lngPack2 = Npack_Cmd-1;														// Запрос от Uart2 БЦУ. Длина пакета RS485 максимальная	 
		}
		
		//..............................................................................................................................
		//ReadADC();																												// Чтение напряжения на 6-ти датчиках ЗРУ
		if (OkResult)		{	bOkDataADC();	 }																	// Значение одной точки параметра получено
		if (OkDataADC)	{	PutParamADC(); }																	// Запись напряжения соответствующего параметра
		
		//..............................................................................................................................
		Curr_W();																														// Текущий уровень заряженности НВАБ (по давлению) 

		//..............................................................................................................................
		if (bUstavkiBCU)	{	
			MakePack5();	bUstavkiBCU = 0;	}																	// Получены уставки БЦУ
		
		//..............................................................................................................................
		if (bGetData)			{ MakePack8();	bGetData = 0;			}								// Получение данных восстановления этапа ТВЦ (из пакета 8)	
/*		
		//..............................................................................................................................
		if ((stat1[iMUK_ZRU] & bRazryad)||(stat1[iMUK_ZRU] & bZaryad))			// Ток > 1
																			stat1[iMUK_ZRU]	|= pwrZRU;				// Состояние силового канала i-го ЗРУ
		else	{
			if (stat3[iMUK_ZRU] & vklZRU)	{																		// ЗРУ включено
				if ((MDR_PORTC->RXTX | 0x2)&&																		// Включен запред заряда АБ 20/PC1
						(MDR_PORTC->RXTX | 0x1))																		// Включен запред разряда АБ 19/PC0
																			stat1[iMUK_ZRU]	&= ~pwrZRU;
				else													stat1[iMUK_ZRU]	|= pwrZRU;		}
			else														stat1[iMUK_ZRU]	&= ~pwrZRU;
		}	
*/
		//..............................................................................................................................
		if (bSendStatus)	{	CAN_SendStatusZRU();	bSendStatus = 0;	}	
		
		/*/..............................................................................................................................
		if (yes100ms)		MDR_PORTB->RXTX = (MDR_PORTB->RXTX & (~PORT_JTAG_Msk)) | 1<<10;					// PB10 = 1	Аппаратное отключение силовых линий ЗРУ 
		else	
			if (MDR_PORTB->RXTX & PB10)
										MDR_PORTB->RXTX = (MDR_PORTB->RXTX & (~(PORT_JTAG_Msk | PB10)));				// PB10 = 0
		*/
		//..............................................................................................................................
		//resetInternalWatchdog();																						// Перезапуск (сброс) внутреннего сторожевого таймера.
	}
}
/* END OF FILE Main.c */

