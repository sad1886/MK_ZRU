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
#include "math.h"

//--------------------------- Общие ---------------------------------------------------------------------------------------
volatile unsigned char mode;																	// Текущий режим работы контроллера
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
volatile unsigned char nBadAK_U[72]; 	//массив содержит информацию, отказавший ли АК или нет, 0 - норма, 1 - отказавший
volatile unsigned char nBadAK_DT[5];	//массив содержит информацию, отказавший ли АК или нет, 0 - норма, 1 - отказавший
volatile unsigned char AK_to_DT[5] = {54, 25, 17, 60, 50}; //таблица соответствия номеров АК (72 штуки) номерам ДД и ДТ (5 штук). 

//-------------------------------------------------------------------------------------------------------------------------
volatile float Pu[nUst] = {Pvuz1_def,Pvuz2_def,Pvir3_def};					// Уставки давления. Рвыр = Pu[nUst]
volatile float Tu[nUst]	= {Tvuz1_def,Tvuz2_def,Tvir3_def};					// Уставки температуры

volatile unsigned char iUst;																				// Индекс текущей уставки 0..nUst-1

volatile float	curW_zar,																						// Текущ заряд НВАБ		9
								W_raz,																							// Энергоёмк
								C_raz,																							// Ёмкость
								Po_ras,																							// Средн_P_разр
								Pvz_ras;																						// Средн_P_подзар,		13

volatile float	P = 0,																							// среднее значение ДД 
								dP = 0,																							// Разница максимального и минимального давления ДД
								T = 0,																							// среднее значение ДТ 
 								Uab = 10, Uab_old,																	// значение напряжения АБ от БЭ
								UsrAk = -0.7,																				// среднее значение напряжения АЭ
								dUak = 0,																						// Разница напряжений макс и мин значений из 72 АК, В, было[1] максимальное отклонение напряжения АЭ ΔUрез
								Umin_ak = -0.7,																			// минимальное значение напряжения АЭ
								Umax_ak = -0.7;																			// максимальное значение напряжения АЭ

volatile float	P_array[5], T_array[5], Uak_array[72];
volatile float	Pmax = 0, 																					// Максимальное давление
								Pmin = 0, 																					// Минимальное давление
								dT = 0,																							// Разница максимального и минимального давления ДТ
								Tmax = 0, 																					// Максимальная температура
								Tmin = 0;																						// Минимальная температура

//------------ для обмена и синхронизации МК-ов ---------------------------------------------------------------------------
unsigned char stat1[3]={0,0,0},
							stat2[3]={0,0,0},
							stat3[3]={0,0,0},
							stat4[3]={0,0,0},
							stat5[3]={0,0,0};

//текущий шаг алгоритма					
extern enum TestSteps StepAlgortmTest;							
extern enum ZarSteps StepAlgortmZar;
extern enum RazSteps StepAlgortmRazr; 
extern enum PodzarSteps StepAlgortmPodzar;							

volatile int DataOk;																								// Признак данные телеметрии АБ БЭ получены
unsigned char i;																										// 

//-------------------------------------------------------------------------------------------------------------------------
extern volatile unsigned char bNoWrkCAN;														// 0 - CAN работает, 1 - CAN не работает
							
unsigned char mk_be_osn[3]={0,0,0};	//связь МК ЗРУ с одноименным МК БЭ (0 - связь есть, 1 - связи нет). Основной канал
unsigned char mk_be_res[3]={0,0,0}; //связь МК ЗРУ с одноименным МК БЭ (0 - связь есть, 1 - связи нет). Резервный канал							

extern volatile unsigned char bRestData, vRestData;									// 1 - восстановить данные
extern volatile unsigned char bRestData_indiv;											// запрос на восстановление данных, индивидуальный

extern volatile unsigned char bSendStatus;													// послать байт состояния МУК ЗРУ

extern uint32_t bitNotZar, bitNotRaz;																// Состояние проводных линий
extern uint32_t Prev_bitNotZar, Prev_bitNotRaz;											// предыдущее состояние проводных линий, необходимо чтобы отслеживать было ли мигание
extern uint32_t ZaprZarProv, ZaprRazrProv; 													// состояние проводных запретов заряда, разряда. Формируется на основе "мигания" соответствующих проводных линий
extern uint32_t cntZarProv, cntRazrProv; //счетчики секунд отсутствия мигания проводных линий запретов

//--------------------------- ADC переменные ------------------------------------------------------------------------------
extern volatile int iadc;																						// Указатель на текущий канал измерения АЦП
extern volatile int iReadAdc;																				// Счётчик измерений текущего канала АЦП
extern volatile int NumReadAdc;																			// Число достоверных значений измерения АЦП текущего канала
extern volatile uint32_t cntReadADC;																// Счётчик циклов чтения АЦП

extern float aI_razr_dt[2], aI_zar_dt[2];														// Значения двух датчиков тока, передаваемые в телеметрию
extern float aI_razr, aI_razrOld, aI_zar;														// Значение тока, используемое в алгоритмах
extern float vU_zru, vU_zru_Old;																		// Uаб измеренное самим ЗРУ, а также предыдущее значение, необходимое для расчета W
extern float dTemp1_zru, dTemp2_zru;;																// датчики температуры

extern float cUsm[3][2];																						// (В) Измеренное напряжение средней точкой (смещение) для МУКов
extern float AUcc[3];																								// (В) Опорное напряжение АЦП МУК1, МУК2, МУК3
extern const float Uref[3];																					// (В) Прецизионное напряжение АЦП МУК1, МУК2, МУК3 на PD0
unsigned char OkResult;																							// Результат получен
extern volatile unsigned char OkDataADC;														// Значение канала АЦП готово
extern uint32_t summa;																							// Сумма значений
extern float Uadc;																									// Значения, прочитанные из АЦП
extern volatile uint32_t ADC1_REG_GO;																// #define ADC1_CFG_REG_GO  ((uint32_t)0x00000002)

//--------------------------- Преобразованные данные из БЭ ----------------------------------------------------------------
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

extern volatile union uBytes64 Reciev_CanDatch_All[nMUKBE][nFrameDatchCAN];			// Телеметрия датчиков всех трех МК
extern volatile union uBytes64 Reciev_CanAB_All[nMUKBE][nFrameABCAN];						// Телеметрия АБ всех трех МК

volatile unsigned char bReciev_CanErrors[nMUKBE];													// Флаги принятых фреймов телеметрии отказов БЭ
volatile unsigned char bReciev_CanDatch[nMUKBE][nFrameDatchCAN];					// Флаги принятых фреймов телеметрии датчиков
volatile unsigned char bReciev_CanAB[nMUKBE][nFrameABCAN];								// Флаги принятых фреймов телеметрии АБ

int nfRec_CanDatch1, nfRec_CanDatch2;																			// Маска принятых фреймов телеметрии датчиков 0x3ff
int nfRec_CanAK1, nfRec_CanAK2;																						// Маска принятых фреймов телеметрии АК 0xfffff

int nfRec_CanDatch1_All[nMUKBE], nfRec_CanDatch2_All[nMUKBE];							// Маска принятых фреймов телеметрии датчиков 0x3ff всех трех МК БЭ
int nfRec_CanAK1_All[nMUKBE], nfRec_CanAK2_All[nMUKBE];										// Маска принятых фреймов телеметрии АК 0xfffff всех трех МК БЭ

extern volatile unsigned char bRunCmdCAN, CurrentCmd, CurrentDlc;					// Флаг отправки команды по CAN
extern uint32_t ResultCAN;																								// Результат выполнения команд Вкл_РС, Откл_РС, Байт ошибок БЭ  по CAN

extern int cntSec_noCAN[6];																								// счётчики секунд неработоспособности CAN (1..6)
extern int cntSec_noMK_ZRU[3];																						// счётчики секунд отсутствия связи с другим МК ЗРУ

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
extern unsigned char PackRs10[lngPackRs10];												// Ответ на пакет 10

extern unsigned char * p_ParRs;
extern unsigned char * p_InPack;

extern volatile unsigned char BatchSize;													// Размер пакета в байтах

extern float z_p [nParams+5];																			// z – цена (вес) младшего разряда;
extern float x0_p[nParams+5];																			// x0 – сдвиг нуля
	

extern float z_p5 [lngUstavki_Curr-6];														// z – цена (вес) младшего разряда;
extern float x0_p5[lngUstavki_Curr-6];														// x0 – сдвиг нуля




int cnt_can=0, cntTimeOut=0;

extern unsigned char bReqBCU[2];																		// Флаг: поступил запрос (команда) от БЦУ

unsigned char bUstavkiBCU;

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
int sCount, sCountTVC, sCount_R, sCount_2h;												// Счётчик секунд общей паузы, Счётчик секунд для 2-х часов ожидания в ТВЦ (sCount_2h стала вспомогательной и используется в разных местах) 
int LimsCount, LimsCountTVC, LimsCount_R;													// Предельное (конечное) значение для счётчика секунд общей паузы

//........... T V C ...........
volatile unsigned char  StepAlgortm,
												bFlag,
												cntRazr, statTVC, 
												ETVC; //этап ТВЦ, который будем восстанавливать, получив команду ВКЛ ТЕСТ
int tstatTVC;																											// Текущее время от начала этапа 

float calc_dt = (float)dt5/(60*60); 															//дельта времени, необходимая для расчета W и C, может меняться в зависимости от паузы между измерениями, по умолчанию соответствует дельте в 5 секунд
float calc_dt1 = (float)dt1/(60*60);															// 1 сек
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
unsigned char mode_Zaryad = 0;  //если не 0, значит мы находимся в режиме заряда (ток заряда больше нуля) 

unsigned char bReadyWrk, cntReadyWrk;			// Разрешение на ответы по RS485
//-------------------------------------------------------------------------------------------------------------------------
// Подготовка мажоритированных данных для телеметрии
unsigned int MajorStatZRU(unsigned char * stat);

//-------------------------------------------------------------------------------------------------------------------------
//функция преобразует число с плавающей точкой в байт для последующей отправки по протоколу RS-485
unsigned char CreateByteFromParam(float fv, float x0, float z)
{	
	unsigned char b;	int tmp;
	if (fv >= x0)		
	{
		tmp = (int) ((fv-x0)/z + 0.5);
		if (tmp>255)	b = 255;	else	b = tmp;	
	}	
	else	b = 0;

	return b;
}	
//-------------------------------------------------------------------------------------------------------------------------
//функция находит в массиве нужный параметр, при этом номер фрейма (frame_number) и номер байта (byte_number) должны быть указаны в соответствии с протоколом обмена CAN 
//то есть индек в массивах начинается с 1, а не с 0. Вычитание происходит внутри функции.
float GetParamFromCANFrame(volatile union uBytes64 ArrayFrames[], unsigned int frame_number, unsigned int byte_number)
{
	union uBytesFloat16 tmp;
	
	tmp.b[0] = ArrayFrames[frame_number-1].b[byte_number-1];
	tmp.b[1] = ArrayFrames[frame_number-1].b[byte_number];
	
	return tmp.Fdata;
}
//-------------------------------------------------------------------------------------------------------------------------
//функция создает телеметрию 72АК для третьего МК ЗРУ
//возвращает номер выбранного МК БЭ (0 или 1). Если возвращает 2, значит связи с обоими МК1 и МК2 нет и необходимо искусственно пересчитать всех параметры АК
int CreateAKtelem()
{
	volatile float Uak_array_1[72], Uak_array_2[72], Uab1_sum, Uab2_sum; //телеметрия 72АК для МК1 и МК2 БЭ
//	volatile float Uab_be[6]; //значения шести датчиков Uаб всех трех МК БЭ, первые два - МК1, вторые дав - МК2, третьи два - МК3
	unsigned int ind, fr, i, true_ind;
	int connect[3];
	
	true_ind = 2; //пока не начали, считаем, что связи с другими МК БЭ нет
	
	//определяем, есть ли связь с МК БЭ
	connect[0] = ( (bNoWrkCAN & (3<<0) ) != (3<<0) ); //если хотя бы один из двух каналов работает
	connect[1] = ( (bNoWrkCAN & (3<<2) ) != (3<<2) ); //если хотя бы один из двух каналов работает
	connect[2] = ( (bNoWrkCAN & (3<<4) ) != (3<<4) ); //если хотя бы один из двух каналов работает
	
	if(connect[0] && connect[1]) //если есть связь и с МК1 и с МК2
	{
		//получаем телеметрию МК1
		ind = 0; fr = 2;
		Uak_array_1[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[0], fr, 5); //АК1
		Uak_array_1[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[0], fr, 7); //АК2
		for (fr = 3; fr <=19 ; fr++)
		{
			Uak_array_1[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[0], fr, 1);		
			Uak_array_1[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[0], fr, 3);	
			Uak_array_1[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[0], fr, 5);	
			Uak_array_1[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[0], fr, 7);	
		}
		Uak_array_1[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[0], 20, 1); //АК71
		Uak_array_1[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[0], 20, 3); //АК72	
		
		//получаем телеметрию МК2
		ind = 0; fr = 2;
		Uak_array_2[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[1], fr, 5); //АК1
		Uak_array_2[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[1], fr, 7); //АК2
		for (fr = 3; fr <=19 ; fr++)
		{
			Uak_array_2[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[1], fr, 1);		
			Uak_array_2[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[1], fr, 3);	
			Uak_array_2[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[1], fr, 5);	
			Uak_array_2[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[1], fr, 7);	
		}
		Uak_array_2[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[1], 20, 1); //АК71
		Uak_array_2[ind++] = GetParamFromCANFrame(Reciev_CanAB_All[1], 20, 3); //АК72			
		
		//находим сумму всех АК для каждого первых двух МК БЭ
		Uab1_sum = Uab2_sum = 0;
		for(i = 0; i < 72; i++)
		{
			Uab1_sum += Uak_array_1[i];
			Uab2_sum += Uak_array_2[i];
		}
		
		//до вызова этой функции мы уже ранее посчитали наиболее правдоподобное значение Uab
		if( fabs(Uab1_sum - Uab) < fabs(Uab2_sum - Uab) ) //если первый МК БЭ выдал сумму с меньшим отклонением
			true_ind = 0; //то считаем, что его телеметрию мы должны брать за основу
		else 
			true_ind = 1; //иначе за основу берем телеметрию второго МК БЭ			
	}
	else
	{
		if(connect[0])
			true_ind = 0;
		else if(connect[1])
			true_ind = 1;
		else 
			return 2; //если связи нет с обоими МК, то выходим с кодом 2
	}
	
	//заполняем массивы МК3 ЗРУ данными одного из двух МК, тем самым имитируя полученную телеметрию
	for(i = 0; i < nFrameABCAN; i++)
		Reciev_CanAB[i].data64 = Reciev_CanAB_All[true_ind][i].data64;
	
	return true_ind;
}
//-------------------------------------------------------------------------------------------------------------------------
//функция получает из КАНовских фреймов необходимые нам параметры
void GetDataFromCan() 
{	
	unsigned int i, cnt;
	unsigned int ind, fr;
	float sum;
	float Uab1[3], Uab2[3], Uab_sr12[3], Uab_sr_123, num_sr_123, Uab_delta[3], Uab_maxdelta, Uab_sum; 
	int connect[3], ind_of_maxdelta;
	int mk3_ind;
	
	//определяем, есть ли связь с МК БЭ
	connect[0] = ( (bNoWrkCAN & (3<<0) ) != (3<<0) ); //если хотя бы один из двух каналов работает
	connect[1] = ( (bNoWrkCAN & (3<<2) ) != (3<<2) ); //если хотя бы один из двух каналов работает
	connect[2] = ( (bNoWrkCAN & (3<<4) ) != (3<<4) ); //если хотя бы один из двух каналов работает
	
	{//Напряжение АБ		
		num_sr_123 = 0; Uab_sr_123 = 0; sum = 0;
		for(i = 0; i < 3; i++) //фиксируем три пары от каждого из трех МК БЭ
		{
			//контролируем, есть ли связь с этим МК			
			if(connect[i]) //если связь есть
			{
				Uab1[i] = GetParamFromCANFrame(Reciev_CanDatch_All[i], 8, 7); 
				Uab2[i] = GetParamFromCANFrame(Reciev_CanDatch_All[i], 9, 1);		
			
				Uab_sr12[i] = (Uab1[i] + Uab2[i])/2; //находим среднее между двумя Uаб одного МК БЭ
				
				sum += Uab_sr12[i]; //если связь с МК БЭ есть, то можем добавлять к расчетам его параметры
				num_sr_123++; //увеличиваем количество МК, участвующих в расчетах
			}
			else
			{
				Uab1[i] = Uab2[i] = Uab_sr12[i] = 0;				
			}
		}
		Uab_sr_123 = sum/num_sr_123; //находим среднее между всеми тремя МК БЭ (точнее, между всеми расчетными)
		
		if(num_sr_123 < 3) //если не было связи со всеми тремя МК
		{
			Uab = Uab_sr_123; //нет смысла искать отклонение от среднего. Если МК остался один или два, то мы уже нашли число, которое должны выдавать
		}
		else //находим среди трех МК максимально отличающийся, чтобы его исключить
		{
			Uab_maxdelta = 0; ind_of_maxdelta = 0;
			for(i = 0; i < 3; i++)
			{
				Uab_delta[i] = fabs(Uab_sr_123 - Uab_sr12[i]); //для каждого из трех БЭ находим отклонение
				if(Uab_delta[i] >= Uab_maxdelta) //если отклонение максимальное
				{
					Uab_maxdelta = Uab_delta[i];
					ind_of_maxdelta = i; //запоминаем номер этого БЭ
				}
			}

			Uab_sum = 0;
			for(i = 0; i < 3; i++)
			{
				if(i == ind_of_maxdelta) continue;
				Uab_sum += Uab_sr12[i];
			}
			Uab = Uab_sum/2;		
		}

	}//~Напряжение АБ		
	
	//пять давлений
	P_array[0] = GetParamFromCANFrame(Reciev_CanDatch, 3, 7);
	P_array[1] = GetParamFromCANFrame(Reciev_CanDatch, 4, 1);
	P_array[2] = GetParamFromCANFrame(Reciev_CanDatch, 4, 3);
	P_array[3] = GetParamFromCANFrame(Reciev_CanDatch, 4, 5);
	P_array[4] = GetParamFromCANFrame(Reciev_CanDatch, 4, 7);
	
	//пять температур
	T_array[0] = GetParamFromCANFrame(Reciev_CanDatch, 5, 1);
	T_array[1] = GetParamFromCANFrame(Reciev_CanDatch, 5, 3);
	T_array[2] = GetParamFromCANFrame(Reciev_CanDatch, 5, 5);
	T_array[3] = GetParamFromCANFrame(Reciev_CanDatch, 5, 7);
	T_array[4] = GetParamFromCANFrame(Reciev_CanDatch, 6, 1);	
	
	//состояние РС
	if (Reciev_CanDatch[9].b[2])	stat1[iMUK_ZRU] |= bPC;
	else													stat1[iMUK_ZRU] &= ~bPC;	
	
	//72 напряжения на АК
	mk3_ind = 0;
	if(nMUK_ZRU==nMUK3_ZRU) //если имеем дело с МК3, то прежде чем работать по штатному алгоритму, необходимо имитировать телеметри. 72АК
	{
		mk3_ind = CreateAKtelem();
	}
	if( (nMUK_ZRU==nMUK3_ZRU) && (mk3_ind == 2) ) //если имеем дело с МК3 ЗРУ и связи с первыми двумя МК БЭ нет
	{
		Uak_array[0] = Uab/72; //просто берем напряжение и делим его на 72
		for(i = 0; i < 72; i++)
			Uak_array[i] = Uak_array[0];
	}
	else //во всех остальных случаях работаем штатно
	{
		ind = 0; fr = 2;
		Uak_array[ind++] = GetParamFromCANFrame(Reciev_CanAB, fr, 5); //АК1
		Uak_array[ind++] = GetParamFromCANFrame(Reciev_CanAB, fr, 7);	//АК2
		for (fr = 3; fr <=19 ; fr++)
		{
			Uak_array[ind++] = GetParamFromCANFrame(Reciev_CanAB, fr, 1);		
			Uak_array[ind++] = GetParamFromCANFrame(Reciev_CanAB, fr, 3);	
			Uak_array[ind++] = GetParamFromCANFrame(Reciev_CanAB, fr, 5);	
			Uak_array[ind++] = GetParamFromCANFrame(Reciev_CanAB, fr, 7);	
		}
		Uak_array[ind++] = GetParamFromCANFrame(Reciev_CanAB, 20, 1); //АК71
		Uak_array[ind++] = GetParamFromCANFrame(Reciev_CanAB, 20, 3);	//АК72
	}
	
	
	//расчеты	
	//давление
	P = 0; Pmax = 0; Pmin = 0; dP = 0; 
	sum = 0; cnt = 0;
	
	for (i=0; i <= 4; i++) 
	{
		if(nBadAK_DT[i] == 0) //находим первый неотказавший измерительный АК
		{
			Pmax = P_array[i];
			Pmin = P_array[i];
		}	
	}	

	for (i=0; i <= 4; i++)
	{
		if(nBadAK_DT[i] != 0) //если АК отказавший
			continue; //игнорируем его
		
		if(P_array[i] > Pmax) 
			Pmax = P_array[i];
		if(P_array[i] < Pmin) 
			Pmin = P_array[i];
		
		sum += P_array[i];
		cnt++;
	}
	P = sum/cnt;
	dP = Pmax - Pmin;
	
	//температура
	T = 0; Tmax = 0; Tmin = 0; dT = 0;
	sum = 0; cnt = 0;

	for (i=0; i <= 4; i++) 
	{
		if(nBadAK_DT[i] == 0) //находим первый неотказавший измерительный АК
		{
			Tmax = T_array[i];
			Tmin = T_array[i];
		}	
	}		
	
	for (i=0; i <= 4; i++)
	{
		if(nBadAK_DT[i] != 0) //если АК отказавший
			continue; //игнорируем его
		
		if(T_array[i] > Tmax) 
			Tmax = T_array[i];
		if(T_array[i] < Tmin) 
			Tmin = T_array[i];
		
		sum += T_array[i];
		cnt++;
	}
	T = sum/cnt;
	dT = Tmax - Tmin;	
	
	//напряжение АК
	UsrAk = 0; Umax_ak = 0; Umin_ak = 0; dUak = 0;
	sum = 0; cnt = 0;
	
	for (i=0; i <= 71; i++)
	{
		if(nBadAK_U[i] == 0) //находим первый неотказавший АК
			{
				Umax_ak = Uak_array[i];
				Umin_ak = Uak_array[i];
			}			
	}
	
	for (i=0; i <= 71; i++)
	{
		if(nBadAK_U[i] != 0) //если АК отказавший
			continue; //игнорируем его
		
		if(Uak_array[i] > Umax_ak) 
			Umax_ak = Uak_array[i];
		if(Uak_array[i] < Umin_ak) 
			Umin_ak = Uak_array[i];
		
		sum += Uak_array[i];
		cnt++;
	}
	
	UsrAk = sum/cnt;
	dUak = Umax_ak - Umin_ak;
}

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
// Получение параметра из телеметрии БЭ
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
{	float Uab1, Uab2, mUab;	
	P   = GetParam(1, 0);																									// [0] среднее значение ДД 
	dP  = GetParam(1, 2)-GetParam(1, 3);																	// [1] разница между максимальным и минимальнымзначением ДД
	T   = GetParam(1, 4);																									// [4] среднее значение ДТ 
	
	//.....................................................................
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

	//.....................................................................
	UsrAk = GetParam(0, 0);																								// [0] среднее значение напряжения АЭ Н И Г Д Е  НЕ  И С П О Л Ь З У Е Т С Я !
	Umax_ak = GetParam(0, 2);
	Umin_ak = GetParam(0, 3);																							// [3] минимальное значение напряжения АЭ
	dUak = Umax_ak - Umin_ak;																							//	Разница	Umax_ak - Umin_ak
	
	
	
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
	C_raz += ((aI_razrOld + aI_razr)/2)*calc_dt;																// Расчёт	C = C + Iab*dt, 		
	W_raz += ((aI_razrOld + aI_razr)/2)*calc_dt*((Uab_old + Uab)/2);						// Расчёт W = W + Iab*dt*Uab
	Uab_old = Uab;	aI_razrOld = aI_razr;
}

//-------------Расчёт C W при потерея обоих линий CAN--------------------------------------------------------------------------------------------------
void Calculation_noCAN (void)
{
	C_raz += ((aI_razrOld + aI_razr)/2)*calc_dt;																// Расчёт	C = C + Iab*dt, 		dt=5сек
	W_raz += ((aI_razrOld + aI_razr)/2)*calc_dt*((vU_zru_Old + vU_zru)/2);			// Расчёт W = W + Iab*dt*Uab
	vU_zru_Old = vU_zru;		aI_razrOld = aI_razr;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Test_NVAB (void)														/* _Т_В_Ц___Н_В_А_Б_ */
{		
	int ii;

	//SetParamTest_NVAB();
	switch (StepAlgortm)																									// Переключатель состояний алгоритма ТВЦ
	{
	// .......... Инициализация ТВЦ .........................................................................................1
	case st_t_InitTest:

		statTVC = 1; tstatTVC =0; // Этап проведения ТВЦ. Время начала этапа.

	// ========== Отключение КОМП ...........................................................................................2
	case st_t_1_01:		
		
		pOtkl_KOMP();
		LimsCount = vsCount5;	sCount=0;		bPauza=1;													// Активация паузы 5 сек
		StepAlgortm = st_t_1_02;          	
	break;
	
	// .......... Ожидание отключения КОМП Заряда ...........................................................................3
	case st_t_1_02:		

		if (!bPauza) {
			StepAlgortm = st_t_1_03;	
		}
		break;
		
	// ========== Ожидание разряда АБ .......................................................................................4
	case st_t_1_03:
		
		if (!bPauza) {																											// Пауза 1 мин bPauza1m
			if	(((P <= 0.8*Pn) && (T <= Tn3))||															// (P<=0.8Pn && T<=Tn3)||
					 ((0.8*Pn <= P) && (P <= Pn) && (T <= Tn2))||									// (0.8Pn<=P<=Pn && T<=Tn2)||
					 ((Pn <= P) && (P <= Pv) && (T <= Tn1)))	
			{													 					 
				pVkl_Zapr_Razrayd();
				pVkl_Zapr_Zarayd();			
						for (ii=0; ii < 500; ii++)	tstatTVC =0;	//микропауза в 500мкс
				StepAlgortm = st_t_1_04;							 
		  }
			else	{																														
				LimsCount = vmCount1;		sCount=0;	bPauza=1;											// Сброс счётчика 1 мин, включение паузы 1 мин
				StepAlgortm = st_t_1_03;
			}
		}
		break;   
		
	// ========== Включение Заряда ..........................................................................................5
	case st_t_1_04:			
		
		pVkl_Test_Zarayd();	
		pOtkl_Zapr_Zarayd();
		stat4[iMUK_ZRU] &= ~br1;																					// запрет заряда = 0
		LimsCount = vsCount20;	sCount=0;		bPauza=1;												// Активация паузы 20 сек
		StepAlgortm = st_t_1_05;           
	break;
		
	// .......... Ожидание включения Тест Заряда ............................................................................6
	case st_t_1_05:	
		
		if (!bPauza) {
			if	(aI_zar > aIporog)	{	stat2[iMUK_ZRU] &= ~errNoVklZar;					// Собщение "Не включился Заряд АБ"=0
			//if	(aI_zar < 50)	{	stat2[iMUK_ZRU] &= ~errNoVklZar;							// Отладочная заглушка, Собщение "Не включился Заряд АБ"=0			
				StepAlgortm = st_t_1_06;
			}
			else	{									stat2[iMUK_ZRU] |= errNoVklZar;						// Собщение "Не включился Заряд АБ"=1
				StepAlgortm = st_t_1_04;
			}	
		}
		break;
		
	// ========== Заряжаем батарею ..........................................................................................7
	case st_t_1_06:		
		
		if (!bPauza) {
			if	((T >= Tn3)||																									// (T>Tn3)||
					 ((0.8*Pn <= P) && (P <= Pn) && (T >= Tn2))||									// (0.8Pn<=P<=Pn && T>=Tn2)||
					 ((Pn <= P) && (P <= Pv) && (T >= Tn1))||											// (Pn<=P<=Pv && T>=Tn1)
					 (P >= Pv) ||
//надо обсудить
//					 ((stat3[iMUK_ZRU2] & bZaprZar)&&															// включен запрет Заряда
//					 ( stat3[iMUK_ZRU3] & bZaprZar))||
					 ((stat4[iMUK_ZRU2] & br1)&&															// Заряд завершён
					 ( stat4[iMUK_ZRU3] & br1)) 	
					)
			{			
				if (P >= Pv) 
				{																								// "Превышение допустимой температуры НВАБ"
					stat3[iMUK_ZRU] |= errPrevDopustT;														// "Превышение допустимой температуры НВАБ"
					StepAlgortm = st_t_InitEnd_Alg_TVC;														// ******** Окончание по превышению температуры											
				}														// Для первого заряда StepNext = bOtkl_Zarayd. Для второго StepNext = bCmp_T_Tv_P_Pn
				else	
				{
					StepAlgortm = st_t_1_07;
				}														// Ожидание завершения заряда на других МК
			}	
			else	{																														// Продолжаем заряд
				LimsCount = vmCount1;		sCount=0;		bPauza=1;										// Включене паузы 1 мин
				StepAlgortm = st_t_1_06;									
			}
		}	
		break;

	// ========== Отключаем Заряд ...........................................................................................8
	case st_t_1_07:	
		
		stat4[iMUK_ZRU] |= br1;																							// Запрет заряда = 1
		pVkl_Zapr_Zarayd();																									// ВКЛ ЗАПР ЗАР
		pOtkl_Test_Zarayd();																								// ОТКЛ ТЕСТ ЗАРЯД ВКЛ ЗАПР ЗАР и т.д.
		LimsCount = vsCount20;	sCount=0;		bPauza=1;												// Активация паузы 20сек
		StepAlgortm = st_t_1_08;												

	break;

	case st_t_1_08:		
		
	  if ((!bPauza)&&
	   		((stat4[iMUK_ZRU2] & br1)||																	// включен запрет Заряда
		  	(stat4[iMUK_ZRU3] & br1))) 
		{
			if	(aI_zar > aIporog)		stat2[iMUK_ZRU] |= errNoOtklZar;					// Собщение "Не отключился Заряд АБ"=1
			//if	(aI_zar < 50)		stat2[iMUK_ZRU] |= errNoOtklZar;						// Отладочная заглушка, Собщение "Не отключился Заряд АБ"=1
			else								stat2[iMUK_ZRU] &= ~errNoOtklZar;							// Собщение "Не отключился Заряд АБ"=0
			StepAlgortm = st_t_2_01;																			// 
		}
		break;

	// ========== Начало этапа ...............................................................................		
	case st_t_2_01:			
		
		statTVC = 2;			tstatTVC =0;																		// Этап проведения ТВЦ
		C_raz = 0;		W_raz = 0;																					// На всякий случай сбрасываем рассчетные параметры		
		StepAlgortm = st_t_2_02;																			// 
	
	// ========== Включение тестового разряда ...............................................................................10
	case st_t_2_02:				
		pVkl_Test_Razrayd();
		pOtkl_Zapr_Razrayd();
		stat4[iMUK_ZRU] &= ~br2;																			// 
		time_Razr = 0; 																										// Начали заново процесс разряда
		sCount_2h = 0; 																										//подготавливаем переменную, с помощью которой будем ждать 20 секунд, прежде чем контролировать ток, переменная увеличивается раз в секунду
		LimsCount = vsCount5;	sCount=0;		bPauza=1;												// Активация паузы 5сек, каждые 5 секунд будем делать Calculation
		// Так как начался разряд, нужно уже начать Calculation, поэтому 
		calc_dt = calc_dt5; //дельта времени соответствует 5 секундам	
		Uab_old = Uab;	aI_razrOld = aI_razr; //фиксируем текущие U и I
		//
		StepAlgortm = st_t_2_03;																
		break;
	
	// .......... Ожидание включения разряда ................................................................................11
	case st_t_2_03:
		
		if (!bPauza) {
			Calculation();																										// После разрешения разряда и паузы нужно посчитать
			
			if (sCount_2h >= 20) //если прошло достаточно времени
			{
				if	(aI_razr > aIporog)	
				//if	(40 > aIkomp)																							// Отладочная заглушка
				{
					stat2[iMUK_ZRU] &= ~errNoVklRazr;																// Собщение "Не включился разряд"=0				
					StepAlgortm = st_t_2_04;	
				}
				else	{						
					stat2[iMUK_ZRU] |= errNoVklRazr;																// Собщение "Не включился разряд"=1
					StepAlgortm = st_t_2_02;																		// 
				}		

				LimsCount = vsCount1; 	sCount = 0;		bPauza = 1;												// vsCount5;Активация паузы 5 сек для расчёта W C
				calc_dt = calc_dt5; 																							//дельта времени соответствует 5 секундам					
			}
			else  //если же ток еще рано измерять
			{
				LimsCount = vsCount5;	sCount=0;		bPauza=1;							// Активация паузы 5сек
				calc_dt = calc_dt5;
				StepAlgortm = st_t_2_03; 																//зацикливаемся на Calculation и ожидание time_Razr >= 20
			}
		}			
		break;
	 
	// ========== Разряд АБ и вычисление C и W ..............................................................................12
	case st_t_2_04:		
		
		if (!bPauza) {
			if	(
					(T >= Traz)||																									// Контроль температуры
					(time_Razr >= tP_TVC)||																				// tP_TVC = 40*60 сек
					(Uab <= 72)||																									// напряжения АБ
					(Umin_ak <= 0.1)||
					((stat4[iMUK_ZRU2] & br2)&&																		// включен запрет Разряда
					( stat4[iMUK_ZRU3] & br2))
					)																															// включен запрет Разряда
			{			 
				if	(T >= Traz)	{	stat3[iMUK_ZRU] |= errPrevDopustT;}						// Превышение допустимой температуры
				pVkl_Zapr_Razrayd();
				pOtkl_Test_Razrayd();
				stat4[iMUK_ZRU] |= br2;
				LimsCount = vsCount20;	sCount = 0;		bPauza = 1;								// Активация паузы 20 сек
				StepAlgortm = st_t_2_05;																// Переход запрет разряда StepNext=bOtkl_Razrayd;
			}			 
			else	{																														
				Calculation();
				LimsCount = dt5;	sCount = 0;		bPauza = 1;											// Активация паузы 5 сек
				StepAlgortm = st_t_2_04;
			}	
		}	
		break;
		
	// ========== Отключаем разряд ..........................................................................................13
	case st_t_2_05:					

		if ((!bPauza)&&
					((stat4[iMUK_ZRU2] & br2)||															// включен запрет Разряда
					( stat4[iMUK_ZRU3] & br2)))
			{
				if (aI_razr > aIporog)	{																						// Сообщение "Не отключился разряд"=1
					//if (aI_razr > 5)	{																							// Отладочная заглушка
						stat2[iMUK_ZRU] |= errNoOtklRazr;															// Сообщение "Не отключился разряд"=1
						StepAlgortm = st_t_InitEnd_Alg_TVC;																// ******** Окончание ТВЦ
				}
				else	{
					stat2[iMUK_ZRU] &= ~errNoOtklRazr;						
					
					LimsCount = vsCount5;	sCount = 0;		bPauza = 1;									// Активация паузы 5сек
					
					stat4[iMUK_ZRU] &= ~br3;																			//
					
					StepAlgortm = st_t_2_06;
				}	
			}	
		break;
	 
	// ========== Проверка температуры АБ ...................................................................................14
	case st_t_2_06:
			
		if (!bPauza) {																										// Пауза 
			if	( (T <= 30)||																								// 
						(( stat4[iMUK_ZRU2] & br3 ) && ( stat4[iMUK_ZRU3] & br3 )) ) 		//если в двух других МК уже есть нужный флаг
			{								
				stat4[iMUK_ZRU] |= br3;
				
				stat3[iMUK_ZRU] &= ~errPrevDopustT;
				
				if (( stat4[iMUK_ZRU2] & br3 ) || ( stat4[iMUK_ZRU3] & br3 )) //если хотя бы водном из двух других МК есть флаг синхронизации
				{
					StepAlgortm = st_t_2_07;
				}
				else	{
					bPauza = 0;
					StepAlgortm = st_t_2_06;	
				}
		  }
			else	{	
				LimsCount = vsCount5;		sCount=0;		bPauza=1;										// Сброс счётчика 5 сек, включене паузы 5 сек
				StepAlgortm = st_t_2_06;	}
		}
		break;   
		
	// ========== Включение тестового разряда ...............................................................................10
	case st_t_2_07:		
		
		pVkl_Test_Razrayd();
		pOtkl_Zapr_Razrayd();
		stat4[iMUK_ZRU] &= ~br4;																			// 
		time_Razr = 0; 																										// Начали заново процесс разряда
		sCount_2h = 0; 																										//подготавливаем переменную, с помощью которой будем ждать 20 секунд, прежде чем контролировать ток, переменная увеличивается раз в секунду
		LimsCount = vsCount5;	sCount=0;		bPauza=1;												// Активация паузы 5сек, каждые 5 секунд будем делать Calculation
		// Так как начался разряд, нужно уже начать Calculation, поэтому 
		calc_dt = calc_dt5; //дельта времени соответствует 5 секундам	
		Uab_old = Uab;	aI_razrOld = aI_razr; //фиксируем текущие U и I
		//
		StepAlgortm = st_t_2_08;																
		break;
	
	// .......... Ожидание включения разряда ................................................................................11
	case st_t_2_08:
		
		if (!bPauza) {
			Calculation();																										// После разрешения разряда и паузы нужно посчитать
			
			if (sCount_2h >= 20) //если прошло достаточно времени
			{
				if	(aI_razr > aIporog)	
				//if	(40 > aIkomp)																							// Отладочная заглушка
				{
					stat2[iMUK_ZRU] &= ~errNoVklRazr;																// Собщение "Не включился разряд"=0				
				}
				else	{						
					stat2[iMUK_ZRU] |= errNoVklRazr;																// Собщение "Не включился разряд"=1
				}		
				StepAlgortm = st_t_2_09;
				
				LimsCount = vsCount1; 	sCount = 0;		bPauza = 1;									// Активация паузы 1 сек для расчёта W C
				calc_dt = calc_dt1; 																							//дельта времени соответствует 5 секундам					
			}
			else  //если же ток еще рано измерять
			{
				LimsCount = vsCount5;	sCount=0;		bPauza=1;							// Активация паузы 5сек
				calc_dt = calc_dt5;
				StepAlgortm = st_t_2_08; 																//зацикливаемся на Calculation и ожидание time_Razr >= 20
			}
		}			
		break;	
	
	// ========== Разряд АБ и вычисление C и W ..............................................................................12
	case st_t_2_09:	
		
		if (!bPauza) {
			if	(
					(T >= Traz)||																									// Контроль температуры
					(Uab <= 72)||																									// напряжения АБ
					(Umin_ak <= 0.1)||
					((stat4[iMUK_ZRU2] & br4)&&																		// включен запрет Разряда
					( stat4[iMUK_ZRU3] & br4))
					)																															
			{			 
				if	(T >= Traz)	{	stat3[iMUK_ZRU] |= errPrevDopustT;}						// Превышение допустимой температуры
				pVkl_Zapr_Razrayd();
				pOtkl_Test_Razrayd();
				stat4[iMUK_ZRU] |= br4;
				LimsCount = vsCount20;	sCount = 0;		bPauza = 1;								// Активация паузы 20 сек
				StepAlgortm = st_t_2_10;																// Переход запрет разряда StepNext=bOtkl_Razrayd;
			}			 
			else	{																														
				Calculation();
				LimsCount = vsCount1;	sCount = 0;		bPauza = 1;											// Активация паузы 1 сек
				StepAlgortm = st_t_2_09;
			}	
		}	
		break;	
	
	// ========== Отключаем разряд ..........................................................................................13
	case st_t_2_10:					

		if ((!bPauza)&&
					((stat4[iMUK_ZRU2] & br4)||															// включен запрет Разряда
					( stat4[iMUK_ZRU3] & br4)))
			{
				if (aI_razr > aIporog)	{																						// Сообщение "Не отключился разряд"=1
					//if (aI_razr > 5)	{																							// Отладочная заглушка
						stat2[iMUK_ZRU] |= errNoOtklRazr;															// Сообщение "Не отключился разряд"=1
						StepAlgortm = st_t_InitEnd_Alg_TVC;																// ******** Окончание ТВЦ
				}
				else	{
					stat2[iMUK_ZRU] &= ~errNoOtklRazr;										
					
					stat4[iMUK_ZRU] &= ~br5;																			//
					
					LimsCount = vsCount5;	sCount = 0;		bPauza = 1;									// Активация паузы 5сек
					
					StepAlgortm = st_t_3_01;
				}	
			}	
		break;	
			
	// ========== Начало этапа ...................................................................................14
	case st_t_3_01:
		
		if (!bPauza) {
			statTVC = 3;	tstatTVC =0;																		// Этап проведения ТВЦ
			StepAlgortm = st_t_3_02;																			//
		}
		break;
	
	// ========== Проверка температуры АБ ...................................................................................14
	case st_t_3_02:
		
		if (!bPauza) {																										// Пауза 
			if	( (T <= 30)||																								// 
						(( stat4[iMUK_ZRU2] & br5 ) && ( stat4[iMUK_ZRU3] & br5 )) ) 		//если в двух других МК уже есть нужный флаг
			{								
				stat4[iMUK_ZRU] |= br5;
				
				stat3[iMUK_ZRU] &= ~errPrevDopustT;
				
				if (( stat4[iMUK_ZRU2] & br5 ) || ( stat4[iMUK_ZRU3] & br5 )) //если хотя бы в одном из двух других МК есть флаг синхронизации
				{
					StepAlgortm = st_t_4_01;
				}
				else	{
					bPauza = 0;
					StepAlgortm = st_t_3_02;	
				}
		  }
			else	{	
				LimsCount = vsCount5;		sCount=0;		bPauza=1;										// Сброс счётчика 5 сек, включене паузы 5 сек
				StepAlgortm = st_t_3_02;	}
		}
		break;   			

	// ========== Начало этапа ...................................................................................14
	case st_t_4_01:
		
		statTVC = 4;	tstatTVC =0;																		// Этап проведения ТВЦ
		pVkl_RS(0);																												// Включаем РС
		LimsCount = vsCount20;	sCount = 0;		bPauza = 1;									// Активация паузы 20сек
		StepAlgortm = st_t_4_02;																			//		
		break;			

	// .......... Ожидание Вкл РС ...........................................................................................17
	case st_t_4_02:	
		
		if (!bPauza)
		{
			stat4[iMUK_ZRU] &= ~br6;
			if	(stat1[iMUK_ZRU] & bPC)	{	
				stat2[iMUK_ZRU] &= ~errNoVklRS;																	// Собщение "Не включается РС"=0
				sCount_2h=0;
				LimsCount = vsCount20;	sCount = 0;		bPauza = 1;								// Активация паузы 20 сек
				StepAlgortm = st_t_4_03;																		// Переход на счёт времени разряда напряжения до 0
			}	
			else	{																														// 
				stat2[iMUK_ZRU] |= errNoVklRS;																	// Собщение "Не включается РС"=1
				LimsCount = vmCount1;		sCount = 0;		bPauza = 1;																				// Активация паузы 1 мин
				StepAlgortm = st_t_5_01;																					// Переход на включение КОМП
			}		
		}
		break;
		
	// ========== Разряд АБ с РС не более 30 час ............................................................................18
	case st_t_4_03:				
																																				// При отладке ждём 10 сек и сбрасываем флаг bPC[iMUK_ZRU]
		if (!bPauza) {
			if ((sCount_2h >= tRazr)||																				// Если время разряда достигло предела, timeRazr = 108000сек (30*60*60)
//надо обсудить
//в старом алгоритме мы не анализируем приход команды, только телеметрию			
				(Umin_ak <= 0.3)||(!(stat1[iMUK_ZRU] & bPC))||									// или  АБ разряжена, или поступила команда ОТКЛ РС
				((stat4[iMUK_ZRU2] & br6)&&																	// включен запрет Разряда
				( stat4[iMUK_ZRU3] & br6))
				)
			{			
				stat4[iMUK_ZRU] |= br6;																			// На откл РС

				StepAlgortm = st_t_4_04;																					// Переход на отключение разряда РС
			}	
			else	{																														// Счёт времени
				LimsCount = vsCount5;	sCount = 0;		bPauza = 1;								// Активация паузы 5 сек
				StepAlgortm = st_t_4_03;
			}	
		}	
		break;
		
	// .......... Отключаем РС ..............................................................................................19
	case st_t_4_04:
		
		if (
				((stat4[iMUK_ZRU2] & br6)||																	// включен запрет Разряда
				( stat4[iMUK_ZRU3] & br6))
			 ) 
		{
			 pOtkl_RS(0);																											// Отключаем РС
			 LimsCount = vsCount20;	sCount = 0;		bPauza = 1;									// Активация паузы 20сек
			 StepAlgortm = st_t_4_05;																			// Переход на отключение разряда РС
		}	
		break;
		
	case st_t_4_05:							

		if (!bPauza) {
			if	(!stat1[iMUK_ZRU] & bPC)		stat2[iMUK_ZRU] |= errNoOtklRS;			// Собщение "Не отключается РС"
			LimsCount = vmCount1;		sCount = 0;		bPauza = 1;									// При отладке vmCount5 = 2 сек Активация паузы
			StepAlgortm = st_t_5_01;																						// Переход на включение КОМП
		}
		break;
		
	// ========== Включение КОМП заряда АБ  .................................................................................20
	case st_t_5_01:			
		
		if (!bPauza) {
			statTVC = 5;				tstatTVC =0;																	// Этап проведения ТВЦ
			pVkl_KOMP();
			pVkl_Test_Zarayd();
			pOtkl_Zapr_Zarayd();
			LimsCount = vsCount20;	sCount = 0;		bPauza = 1;									// Активация паузы 20сек
			StepAlgortm = st_t_5_02;																				// Переход на включение КОМП
		}
		break;
	
	// .......... Ожидание Вкл_КОМП .........................................................................................21
	case st_t_5_02:						

		if (!bPauza) {
			if (aI_zar > aIkomp)	{	
				stat2[iMUK_ZRU] |= errNoVklCompZar;				// aIkomp = 3; Сообщение "Не включился КОМП заряд"=1
				StepAlgortm = st_t_6_01;
			}
			else	{												
				sCount_2h=0;
				LimsCount = vsCount20;	 sCount = 0;	bPauza = 1;								// Включене паузы 20 сек
				StepAlgortm = st_t_5_03;
			}	
		}	
		break;
		
	// ========== Заряд АБ компенсационным током ............................................................................22
	case st_t_5_03:					
		
	if (!bPauza) {																												// При отладке vhCount2 = 20 сек
			if	((Uab > 80)&&(sCount_2h >= vmCount10))	
			{												// 
				StepAlgortm = st_t_6_01;																		// 
			}
			else	{																														// 
				LimsCount = vsCount20;	 sCount = 0;	bPauza = 1;	}							// Включене паузы 20 сек
				StepAlgortm = st_t_5_03;
			}
		break;

//	// ========== Вспомогательные шаги для восстановления 6-го этапа тестирования .......................................................................
//	//этот вспомогательный шаг выполняет команды, которые нужны, чтобы начать шестой этап, при восстановлении надо подать их заново
//	case bTest_Vosst_et6_command:			
//		if (!bPauza) {
//			pVkl_Test_Zarayd();		pOtkl_Zapr_Zarayd(); 			
//			LimsCount = vsCount20;	sCount = 0;		bPauza = 1;					// Активация паузы 20сек
//			StepAlgortm = bTest_Vosst_et6_checkI;									
//		}	
//		break;
//		
//	//этот вспомогательный шаг контролирует ток после подачи команд
//	case bTest_Vosst_et6_checkI:			
//		if (!bPauza) {
//			stat2[iMUK_ZRU] &= ~errNoVklCompZar;															// Сообщение "Не включился КОМП заряд"=0
//			if (aI_zar <= aIporog)		stat2[iMUK_ZRU] |= errNoOtklCompZar;			// Сообщение "Не отключился КОМП заряд"=1			
//			stat4[iMUK_ZRU] &= ~bready1; //обнуляем переменную синхронизации
//			StepAlgortm = bVkl_Tst_Zarayd_On2; //все вспомогательные шаги выполнены, можем возвращаться к штатному алгоритму					
//		}	
//		break;
			
	// ========== Включение КОМП заряда АБ  .................................................................................20
	case st_t_6_01:			
		
		if (!bPauza) {
			statTVC = 6;				tstatTVC =0;																	// Этап проведения ТВЦ
			pOtkl_KOMP();
			LimsCount = vsCount20;	 sCount = 0;	bPauza = 1;								// Включене паузы 20 сек
			StepAlgortm = st_t_6_02;																		// Откл КОМП
		}
		break;		
		
	// .......... Ожидание Откл_КОМП ........................................................................................23
	case st_t_6_02:						
		
		if (!bPauza) {
			stat2[iMUK_ZRU] &= ~errNoVklCompZar;															// Сообщение "Не включился КОМП заряд"=0
			if (aI_zar <= aIporog)		stat2[iMUK_ZRU] |= errNoOtklCompZar;			// Сообщение "Не отключился КОМП заряд"=1			
			stat4[iMUK_ZRU] &= ~br7; //обнуляем переменную синхронизации
			StepAlgortm = st_t_6_03;
		}	
		break;


		
	// ========== Заряжаем батарею максимальным током .......................................................................24
	case st_t_6_03:			

		if (!bPauza) {
			if	((T >= Tn3)||																									// (T>Tn3)||
					 ((0.8*Pn <= P) && (P <= Pn) && (T >= Tn2))||									// (0.8Pn<=P<=Pn && T>=Tn2)||
					 ((Pn <= P) && (P <= Pv) && (T >= Tn1))||											// (Pn<=P<=Pv && T>=Tn1)
					 (P >= Pv)||																									// (P>=Pv)
					 ((stat4[iMUK_ZRU2] & br7)&&															// 
					  (stat4[iMUK_ZRU3] & br7))
					)	
			{
				if (P < Pv)		stat3[iMUK_ZRU] |= errPrevDopustT;								// "Превышение допустимой температуры НВАБ"
				stat4[iMUK_ZRU] |= br7;
				LimsCount = vsCount5;		sCount=0;	bPauza=1;											// Для отладки vmCount1 = 2 сек	Сброс счётчика, включене паузы
				StepAlgortm = st_t_6_04;																				// Для первого заряда StepNext = bOtkl_Zarayd. Для второго StepNext = bCmp_T_Tv_P_Pn
			}	
			else	{																														// 
				LimsCount = vmCount1;		sCount=0;	bPauza=1;											// Сброс счётчика, включене паузы
				StepAlgortm = st_t_6_03;									
			}
		}	
		break;

	// .......... дожидаемся других МК ..............................................................................................19
	case st_t_6_04:
		
		if (
				((stat4[iMUK_ZRU2] & br7)||																	// 
				( stat4[iMUK_ZRU3] & br7))
			 ) 
		{
			 StepAlgortm = st_t_7_01;																			// 
		}	
		break;				
		
//	
//	// ========== Вспомогательные шаги для восстановления 7-го этапа тестирования .......................................................................
//	//этот вспомогательный шаг выполняет команды, которые нужны, чтобы начать седьмой этап, при восстановлении надо подать их заново
//	case bTest_Vosst_et7_command:			
//		if (!bPauza) {		
//			pVkl_KOMP();	
//			stat2[iMUK_ZRU] &= ~errNoOtklCompZar;															// Сообщение "Не отключился КОМП заряд"=0			
//			pVkl_Test_Zarayd();		pOtkl_Zapr_Zarayd(); 
//			LimsCount = vsCount20;	sCount = 0;		bPauza = 1;					// Активация паузы 20сек
//			StepAlgortm = bWaitVklKomp2;									
//		}	
//		break;
//		
	// ========== Включение КОМП заряда АБ  .................................................................................25
	case st_t_7_01:			
			
		statTVC = 7;					tstatTVC =0;																// Этап проведения ТВЦ
		pVkl_KOMP();
		stat2[iMUK_ZRU] &= ~errNoOtklCompZar;															// Сообщение "Не отключился КОМП заряд"=0
		LimsCount = vsCount20;	sCount = 0;		bPauza = 1;									// Активация паузы 20сек
		StepAlgortm = st_t_7_02;																			// Переход на проверку включения КОМП
		break;
	
	// .......... Ожидание Вкл_КОМП .........................................................................................26
	case st_t_7_02:						

		if (!bPauza)		{
			if (aI_zar > aIkomp)	{																									// 
					stat2[iMUK_ZRU] |= errNoVklCompZar;														// Сообщение "Не включился КОМП заряд"=1
					StepAlgortm = st_t_InitEnd_Alg_TVC;																// ******** Окончание ТВЦ
			}
			else	{									
					sCount_2h=0;	
					stat4[iMUK_ZRU] &= ~br8;
					LimsCount = vsCount10;	sCount = 0;		bPauza = 1;									// Активация паузы 
					StepAlgortm = st_t_7_03;
			}	
		}	
		break;
		
	// ========== Заряжаем батарею компенсационным током "КОМП" .............................................................27
	case st_t_7_03:			

		if (!bPauza) {
			if (((sCount_2h >= tVir)||(T >= Tk))||																// Для отладки tVir = 10 сек
					((stat4[iMUK_ZRU2] & br8)&&
					( stat4[iMUK_ZRU3] & br8))
				)
			{
				stat4[iMUK_ZRU] |= br8;
				if 	((stat4[iMUK_ZRU2] & br8)||																	// 
						( stat4[iMUK_ZRU3] & br8))
				{
					PvzRas = P;
					pVkl_Zapr_Zarayd();
					pOtkl_Test_Zarayd();																						//  
					pOtkl_KOMP();

					statTVC = 8;				tstatTVC = 0;																//  

					LimsCount = vsCount20;	sCount = 0;	bPauza = 1;									//  
					StepAlgortm = st_t_8_01;															//  
				}
			}
			else	{
				LimsCount = vsCount5;	sCount = 0;	bPauza = 1;			//  
				StepAlgortm = st_t_7_03;																			//  
			}	
		}
		break;
		
// ========== Начало этапа ...............................................................................		
	case st_t_8_01:			
		if (!bPauza) {		
			statTVC = 8;			tstatTVC =0;																		// Этап проведения ТВЦ
			C_raz = 0;		W_raz = 0;																					// На всякий случай сбрасываем рассчетные параметры		
			StepAlgortm = st_t_8_02;																			// 
		}
		break;
	
	// ========== Включение тестового разряда ...............................................................................10
	case st_t_8_02:				
		pVkl_Test_Razrayd();
		pOtkl_Zapr_Razrayd();
		stat5[iMUK_ZRU] &= ~br9;																			// 
		time_Razr = 0; 																										// Начали заново процесс разряда
		sCount_2h = 0; 																										//подготавливаем переменную, с помощью которой будем ждать 20 секунд, прежде чем контролировать ток, переменная увеличивается раз в секунду
		LimsCount = vsCount5;	sCount=0;		bPauza=1;												// Активация паузы 5сек, каждые 5 секунд будем делать Calculation
		// Так как начался разряд, нужно уже начать Calculation, поэтому 
		calc_dt = calc_dt5; //дельта времени соответствует 5 секундам	
		Uab_old = Uab;	aI_razrOld = aI_razr; //фиксируем текущие U и I
		//
		StepAlgortm = st_t_8_03;																
		break;

	// .......... Ожидание включения разряда ................................................................................11
	case st_t_8_03:
		
		if (!bPauza) {
			Calculation();																										// После разрешения разряда и паузы нужно посчитать
			
			if (sCount_2h >= 20) //если прошло достаточно времени
			{
				if	(aI_razr > aIporog)	
				//if	(40 > aIkomp)																							// Отладочная заглушка
				{
					stat2[iMUK_ZRU] &= ~errNoVklRazr;																// Собщение "Не включился разряд"=0					
				}
				else	{						
					stat2[iMUK_ZRU] |= errNoVklRazr;																// Собщение "Не включился разряд"=1
				}		

				StepAlgortm = st_t_8_04;
				LimsCount = vsCount5; 	sCount = 0;		bPauza = 1;												// Активация паузы  сек для расчёта W C
				calc_dt = calc_dt5; 																							//дельта времени соответствует 5 секундам					
			}
			else  //если же ток еще рано измерять
			{
				LimsCount = vsCount5;	sCount=0;		bPauza=1;							// Активация паузы 5сек
				calc_dt = calc_dt5;
				StepAlgortm = st_t_8_03; 																//зацикливаемся на Calculation и ожидание time_Razr >= 20
			}
		}			
		break;
	 
	// ========== Разряд АБ и вычисление C и W ..............................................................................12
	case st_t_8_04:		
		
		if (!bPauza) {
			if	(
					(T >= Traz)||																									// Контроль температуры
					(Uab <= 72)||																									// напряжения АБ
					(Umin_ak <= 0.1)||
					((stat5[iMUK_ZRU2] & br9)&&																		// включен запрет Разряда
					( stat5[iMUK_ZRU3] & br9))
					)																															// включен запрет Разряда
			{			 
				if	(T >= Traz)	{	stat3[iMUK_ZRU] |= errPrevDopustT;}						// Превышение допустимой температуры
				pVkl_Zapr_Razrayd();
				pOtkl_Test_Razrayd();
				stat5[iMUK_ZRU] |= br9;
				LimsCount = vsCount20;	sCount = 0;		bPauza = 1;								// Активация паузы 20 сек
				StepAlgortm = st_t_8_05;																// Переход запрет разряда StepNext=bOtkl_Razrayd;
			}			 
			else	{																														
				Calculation();
				LimsCount = dt5;	sCount = 0;		bPauza = 1;											// Активация паузы 5 сек
				StepAlgortm = st_t_8_04;
			}	
		}	
		break;
		
	// ========== Отключаем разряд ..........................................................................................13
	case st_t_8_05:					

		if ((!bPauza)&&
					((stat5[iMUK_ZRU2] & br9)||															// включен запрет Разряда
					( stat5[iMUK_ZRU3] & br9)))
			{
				if (aI_razr > aIporog)	{																						// Сообщение "Не отключился разряд"=1
						stat2[iMUK_ZRU] |= errNoOtklRazr;															// Сообщение "Не отключился разряд"=1
						StepAlgortm = st_t_InitEnd_Alg_TVC;																// ******** Окончание ТВЦ
				}
				else	{
					stat2[iMUK_ZRU] &= ~errNoOtklRazr;						
					
					LimsCount = vsCount5;	sCount = 0;		bPauza = 1;									// Активация паузы 5сек
					
					stat5[iMUK_ZRU] &= ~br10;																			//
					
					StepAlgortm = st_t_8_06;
				}	
			}	
		break;
	 
	// ========== Проверка температуры АБ ...................................................................................14
	case st_t_8_06:
			
		if (!bPauza) {																										// Пауза 
			if	( (T <= 30)||																								// 
						(( stat5[iMUK_ZRU2] & br10 ) && ( stat5[iMUK_ZRU3] & br10 )) ) 		//если в двух других МК уже есть нужный флаг
			{								
				stat5[iMUK_ZRU] |= br10;
				
				
				if (( stat5[iMUK_ZRU2] & br10 ) || ( stat5[iMUK_ZRU3] & br10 )) //если хотя бы водном из двух других МК есть флаг синхронизации
				{
					StepAlgortm = st_t_8_07;
				}
				else	{
					bPauza = 0;
					StepAlgortm = st_t_8_06;	
				}
		  }
			else	{	
				LimsCount = vsCount5;		sCount=0;		bPauza=1;										// Сброс счётчика 5 сек, включене паузы 5 сек
				StepAlgortm = st_t_8_06;	}
		}
		break;   
		
	// ========== Включение тестового разряда ...............................................................................10
	case st_t_8_07:		
		
		pVkl_Test_Razrayd();
		pOtkl_Zapr_Razrayd();
		stat5[iMUK_ZRU] &= ~br11;																			// 
		time_Razr = 0; 																										// Начали заново процесс разряда
		sCount_2h = 0; 																										//подготавливаем переменную, с помощью которой будем ждать 20 секунд, прежде чем контролировать ток, переменная увеличивается раз в секунду
		LimsCount = vsCount5;	sCount=0;		bPauza=1;												// Активация паузы 5сек, каждые 5 секунд будем делать Calculation
		// Так как начался разряд, нужно уже начать Calculation, поэтому 
		calc_dt = calc_dt5; //дельта времени соответствует 5 секундам	
		Uab_old = Uab;	aI_razrOld = aI_razr; //фиксируем текущие U и I
		//
		StepAlgortm = st_t_8_08;																
		break;
	
	// .......... Ожидание включения разряда ................................................................................11
	case st_t_8_08:
		
		if (!bPauza) {
			Calculation();																										// После разрешения разряда и паузы нужно посчитать
			
			if (sCount_2h >= 20) //если прошло достаточно времени
			{
				if	(aI_razr > aIporog)	
				//if	(40 > aIkomp)																							// Отладочная заглушка
				{
					stat2[iMUK_ZRU] &= ~errNoVklRazr;																// Собщение "Не включился разряд"=0				
				}
				else	{						
					stat2[iMUK_ZRU] |= errNoVklRazr;																// Собщение "Не включился разряд"=1
				}		
				StepAlgortm = st_t_8_09;
				
				LimsCount = vsCount1; 	sCount = 0;		bPauza = 1;									// Активация паузы 1 сек для расчёта W C
				calc_dt = calc_dt1; 																							//дельта времени соответствует 5 секундам					
			}
			else  //если же ток еще рано измерять
			{
				LimsCount = vsCount5;	sCount=0;		bPauza=1;							// Активация паузы 5сек
				calc_dt = calc_dt5;
				StepAlgortm = st_t_8_08; 																//зацикливаемся на Calculation и ожидание time_Razr >= 20
			}
		}			
		break;	
	
	// ========== Разряд АБ и вычисление C и W ..............................................................................12
	case st_t_8_09:	
		
		if (!bPauza) {
			if	(
					(T >= Traz)||																									// Контроль температуры
					(Uab <= 72)||																									// напряжения АБ
					(Umin_ak <= 0.1)||
					((stat5[iMUK_ZRU2] & br11)&&																		// включен запрет Разряда
					( stat5[iMUK_ZRU3] & br11))
					)																															
			{			 
				if	(T >= Traz)	{	stat3[iMUK_ZRU] |= errPrevDopustT;}						// Превышение допустимой температуры
				pVkl_Zapr_Razrayd();
				pOtkl_Test_Razrayd();
				P0_Ras = P;
				stat5[iMUK_ZRU] |= br11;
				LimsCount = vsCount20;	sCount = 0;		bPauza = 1;								// Активация паузы 20 сек
				StepAlgortm = st_t_8_10;																// Переход запрет разряда StepNext=bOtkl_Razrayd;
			}			 
			else	{																														
				Calculation();
				LimsCount = vsCount1;	sCount = 0;		bPauza = 1;											// Активация паузы 1 сек
				StepAlgortm = st_t_8_09;
			}	
		}	
		break;	
	
	// ========== Отключаем разряд ..........................................................................................13
	case st_t_8_10:					

		if ((!bPauza)&&
					((stat5[iMUK_ZRU2] & br11)||															// включен запрет Разряда
					( stat5[iMUK_ZRU3] & br11)))
			{
				if (aI_razr > aIporog)	{																						// Сообщение "Не отключился разряд"=1
					//if (aI_razr > 5)	{																							// Отладочная заглушка
						stat2[iMUK_ZRU] |= errNoOtklRazr;															// Сообщение "Не отключился разряд"=1
						StepAlgortm = st_t_InitEnd_Alg_TVC;																// ******** Окончание ТВЦ
				}
				else	{
					stat2[iMUK_ZRU] &= ~errNoOtklRazr;										
					
					stat5[iMUK_ZRU] &= ~br12;																			//
					
					LimsCount = vsCount5;	sCount = 0;		bPauza = 1;									// Активация паузы 5сек
					
					StepAlgortm = st_t_8_11;
				}	
			}	
		break;	
	
	// ========== Проверка температуры АБ ...................................................................................14
	case st_t_8_11:
		
		if (!bPauza) {																										// Пауза 
			if	( (T <= 30)||																								// 
						(( stat5[iMUK_ZRU2] & br12 ) && ( stat5[iMUK_ZRU3] & br12 )) ) 		//если в двух других МК уже есть нужный флаг
			{								
				stat5[iMUK_ZRU] |= br12;
				
				stat3[iMUK_ZRU] &= ~errPrevDopustT;
				
				if (( stat5[iMUK_ZRU2] & br12 ) || ( stat5[iMUK_ZRU3] & br12 )) //если хотя бы в одном из двух других МК есть флаг синхронизации
				{
					StepAlgortm = st_t_9_01;
				}
				else	{
					bPauza = 0;
					StepAlgortm = st_t_8_11;	
				}
		  }
			else	{	
				bPauza = 0;
				StepAlgortm = st_t_8_11;	}
		}
		break;   			
	
	// ========== Принудительный заряд до рабочей уставки P2 ................................................................28
	case st_t_9_01:
		stat5[iMUK_ZRU] &= ~br13;  /////////		
		statTVC = 9; tstatTVC =0;																						// Этап проведения ТВЦ
		pOtkl_KOMP();
		pVkl_Test_Zarayd();																									// 
		pOtkl_Zapr_Zarayd();
		iUst = 1;																														// N=2
		LimsCount = vsCount20;	sCount = 0;		bPauza = 1;										// Активация паузы 20сек
		StepAlgortm = st_t_9_02;																		// На ожидание Заряда
		break;
			
	// .......... Ожидание включения Тест Заряда ............................................................................29
	case st_t_9_02:		

		if (!bPauza) {
			//if	(5 > aIkomp)	{	stat2[iMUK_ZRU] &= ~errNoVklZar;					// Собщение "Не включился Заряд АБ"=0
			if	(aI_zar > aIporog)	{	stat2[iMUK_ZRU] &= ~errNoVklZar;					// Собщение "Не включился Заряд АБ"=0
				LimsCount = vsCount20;	sCount = 0;		bPauza = 1;								// Активация паузы 20сек
				StepAlgortm = st_t_9_03;
			}
			else	{									
				stat2[iMUK_ZRU] |= errNoVklZar;						// Собщение "Не включился Заряд АБ"=1
				StepAlgortm = st_t_InitEnd_Alg_TVC;
			}	
		}
		break;
		
	// ========== Заряжаем батарею ..........................................................................................30
	case st_t_9_03:			

		if (!bPauza) {
			if	((P >= Pvuz2)||((T >= T_max)&&(P>=0.8*Pvuz2))||
					 ((stat5[iMUK_ZRU2] & br13)&&															// включен запрет Разряда
					  (stat5[iMUK_ZRU3] & br13))
					)
			{			 
				pVkl_Zapr_Zarayd();																							// ВКЛ ЗАПР ЗАР
				pOtkl_Test_Zarayd();																						// ОТКЛ ТЕСТ ЗАРЯД ВКЛ ЗАПР ЗАР и т.д.
				stat5[iMUK_ZRU] |= br13;
				StepAlgortm = st_t_9_04;									
			}	
			else	{																														// 
				StepAlgortm = st_t_9_03;									
			}
			LimsCount = vsCount20;	sCount = 0;		bPauza = 1;									// Активация паузы 20сек
		}	
		break;
		
	// .......... Ожидание Отключения заряда ................................................................................31
	case st_t_9_04:			
	
		if (!bPauza) {
			if 	((stat5[iMUK_ZRU2] & br13)||															
					( stat5[iMUK_ZRU3] & br13))
			{
				if	(aI_zar > aIporog)	stat2[iMUK_ZRU] |= errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=1
				StepAlgortm = st_t_9_05;																					// 
			}
			else	{
				StepAlgortm = st_t_9_04;									
				LimsCount = vsCount5;	sCount = 0;		bPauza = 1;										// Активация паузы 5сек
			}	
		}
		break;
		
	// ========== Выбор уставки .............................................................................................32
	case st_t_9_05:

		if	((dP >= dP_u)||(dUak>=dU_u))	iUst=2;															// N=3
		else	{
			if	(T >= Tu[1])								iUst=0;															// N=1
			else														iUst=1;															// N=2
		}	
		StepAlgortm = st_t_InitEnd_Alg_TVC;
		break;

	// ========== Завершение алгоритма ......................................................................................33
	case st_t_InitEnd_Alg_TVC:	

		mode = Otkl_TEST;
		break;
	
	} //end of switch (StatusZarRazr)
}


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Podzarayd (void)														/*	_Подзаряд__Н_В_А_Б_ */
{	
	switch (StepAlgortmPodzar)																									// Обработчик состояний алгоритма заряда
	{
	// .......... Инициализация .............................................................................................
	case st_p_InitPodzar:				
		pVkl_Zapr_Zarayd();																								// Запрет заряда=1
		LimsCount = vsCount20;	sCount=0;		bPauza=1;												// Активация паузы 20 сек
		StepAlgortmPodzar = st_p_OtklPodzar_1;															// След шаг алгоритма Ожидание включения запрета заряда
		break;
	
	// .......... Проверка отключения тока заряда ........................................................................
	case st_p_OtklPodzar_1:
		if (!bPauza) {
			if	(aI_zar > aIporog) stat2[iMUK_ZRU] |= errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=1		
			StepAlgortmPodzar = st_p_Pnuz_1;
		}
		break;	
		
	// ......................................................................................................................
	case st_p_Pnuz_1:					
		if (P >= Pnuz)																											// 
			StepAlgortmPodzar = st_p_VklKomp;																						// Переход на включение компенсационного тока
		else 																															// P < Pнуз идём по алгортму
			StepAlgortmPodzar = st_p_Tnuz_1;
		break;

	// ......................................................................................................................
	case st_p_Tnuz_1:					
		if (T < TnuzAB)	{
			stat3[iMUK_ZRU] &= ~errPrevDopustT;																// "Превышение допустимой температуры при заряде" = 0
			StepAlgortmPodzar = st_p_Otkl_ZaprZarayd;															// Переход на отключение запрета заряда
		}
		else	{	
			stat3[iMUK_ZRU] |= errPrevDopustT;													// Сообщение "Превышение допустимой температуры при заряде"
			StepAlgortmPodzar = st_p_Tnuz_1;																// Ожидание остывания АБ
		}
		break;
		
	// .......... Отключаем запрет заряда ...................................................................................
	case st_p_Otkl_ZaprZarayd:								
		pOtkl_Zapr_Zarayd();																								// ОТКЛ ЗАПР ЗАРЯДА
		LimsCount = vsCount5;	sCount=0;		bPauza=1;													// Активация паузы 5 сек
		StepAlgortmPodzar = st_p_WaitOtkl_ZaprZar;
		break;
		
	// .......... Ожидание отключения запрета Заряда ........................................................................
	case st_p_WaitOtkl_ZaprZar:		
		if (!bPauza) {
			StepAlgortmPodzar = st_p_Tnuz_2;																						// След шаг алгоритма
		}
		break;

		// .......... Заряд током 7-10А .........................................................................................
	case st_p_Tnuz_2:					
			if (T >= TnuzAB)	{	
				stat3[iMUK_ZRU] |= errPrevDopustT;															// Сообщение "Превышение допустимой температуры при заряде"
				pVkl_Zapr_Zarayd ();																						// Запрет заряда=1
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20 сек
				StepAlgortmPodzar = st_p_OtklPodzar_2;													// След шаг алгоритма Ожидание включения запрета заряда
			}
			else	{
				stat3[iMUK_ZRU] &= ~errPrevDopustT;															// "Превышение допустимой температуры при заряде" = 0
				LimsCount = dt5;	sCount=0;		bPauza=1;													// Активация паузы 5сек
				StepAlgortmPodzar = st_p_Pnuz_2;																			// Переход на заряд
			}
		//}
		break;

	// .......... Заряд АБ током 7-10А до Pнуз...............................................................................
	case st_p_Pnuz_2:					
		if (!bPauza) {
			if (P >= Pnuz)	
				StepAlgortmPodzar = st_p_VklKomp;																					//	P >= Pнуз идём по алгортму
			else	
				StepAlgortmPodzar = st_p_Tnuz_2;																			//  Заряжаем АБ током 7-10А
		}
		break;			

	// .......... Проверка отключения тока заряда ........................................................................
	case st_p_OtklPodzar_2:
		if (!bPauza) {
			if	(aI_zar > aIporog) stat2[iMUK_ZRU] |= errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=1		
			StepAlgortmPodzar = st_Tnuz_minus2;
		}
		break;			

	// ......................................................................................................................
	case st_Tnuz_minus2:					
		if (!bPauza) {
			if (T < (TnuzAB-2))	{
				StepAlgortmPodzar = st_p_Otkl_ZaprZarayd;																	// След шаг алгоритма Ожидание включения запрета заряда
			}
			else	{
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20сек
				StepAlgortmPodzar = st_Tnuz_minus2;																		// Переход на отключение запрета заряда
			}
		}
		break;
		
	// .......... Вкл_КОМП ..................................................................................................
	case st_p_VklKomp:		
		pVkl_KOMP();																												// Вкл_КОМП
		pOtkl_Zapr_Zarayd();																								// ОТКЛ ЗАПР ЗАРЯДА
		LimsCount = vsCount20;	sCount=0;		bPauza=1;												// Активация паузы 20сек
		StepAlgortmPodzar = st_p_WaitVklKomp;																					// Ожидание остывания АБ
		break;

	// .......... Ожидание включения КОМП Заряда ............................................................................
	case st_p_WaitVklKomp:		
		if (!bPauza) {
			if	(aI_zar > aIkomp)	stat2[iMUK_ZRU] |= errNoVklCompZar;					// Собщение "Не включился КОМП Заряд АБ"=1
			StepAlgortmPodzar = st_p_VklKomp;																						// След шаг алгоритма
		}
		else	{
			stat2[iMUK_ZRU] &= ~errNoVklCompZar;
			LimsCount = dt5;	sCount=0;		bPauza=1;														// Активация паузы 5сек
			StepAlgortmPodzar = st_p_ZarydComp;																					// След шаг алгоритма
		}
		break;
		
	// .......... Заряд АБ компенсационным током..........................................................................
	case st_p_ZarydComp:							
		if (!bPauza) {
			if	((P >= PvuzCK)||(T >= (TvuzCK-1)))		{				
				if (T >= (TvuzCK-1))	{																					// T >= 25
					stat3[iMUK_ZRU] |= errPrevDopustT;														// Сообщение "Превышение допустимой температуры при заряде"
					pVkl_Zapr_Zarayd();																						// Вкл Запрет Заряда
					if	(aI_zar > 1) stat2[iMUK_ZRU] |= errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=1			
					bPauza = 0;
					StepAlgortmPodzar = st_p_Tvuz_minus2;
				}
				else {
					pVkl_Zapr_Zarayd();																						// Вкл Запрет Заряда
					pOtkl_KOMP();	
					LimsCount = vsCount5;	sCount=0;		bPauza=1;													// Активация паузы 20сек
					StepAlgortmPodzar = st_p_WaitOtklKomp;
				}
			}
			else	{
				LimsCount = vsCount5;	sCount=0;		bPauza=1;											// Активация паузы 5сек
				StepAlgortmPodzar = st_p_ZarydComp;																				// След шаг алгоритма
			}
		}
		break;
		
	// ......................................................................................................................
	case st_p_Tvuz_minus2:				
		if (!bPauza) {
			if (T >= (TvuzCK-2))	{
				stat3[iMUK_ZRU] &= ~errPrevDopustT;															// Сообщение "Превышение допустимой температуры при заряде"
				pOtkl_Zapr_Zarayd();																						// ОТКЛ ЗАПР ЗАРЯДА
				StepAlgortmPodzar = st_p_ZarydComp;																	// След шаг алгоритма Ожидание включения запрета заряда
			}
			else	{
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20сек
				StepAlgortmPodzar = st_p_Tvuz_minus2;														// зацикливаемся
			}
		}
		break;
	 
	// .......... Ожидание включения КОМП Заряда ............................................................................
	case st_p_WaitOtklKomp:		
		if (!bPauza) {
			if	(aI_zar > aIkomp_1A) stat2[iMUK_ZRU] |= errNoOtklCompZar;			// Собщение "Не отключился Заряд АБ"=1
			else stat2[iMUK_ZRU] &= ~errNoOtklCompZar;			// Собщение "Не отключился Заряд АБ"=0
			StepAlgortmPodzar = st_p_EndPodzaryda;																			// След шаг алгоритма
		}
		break;		
 
	// .......... Отключение Подзаряда .......................................................................................
	case st_p_EndPodzaryda:						
		stat1[iMUK_ZRU] &= ~bPodzaryad;																			// Команда "Подзаряд" снята
		mode = Otkl_Podzarayd;																							// 
	
 }	//end of case
}

//-------------------------------------------------------------------------------------------------------------------------
void Curr_W(void)	// Расчёт текущего уровня заряженности АБ
{	
/*
Расчет значения текущей энергоемкости АБ проводится по формуле:
			P_w = (P*293)/(273+T)
			W = ((P-P0)/(Pвз- P0))·Wтест
*/	
	float P_w = 0;
	
	if((DataOk))
	{
		P_w = (P*293)/(273 + T);
		curW_zar = ((P_w-P0)/(Pvz-P0)) * kWtst;	
	}
	else
		curW_zar = 0;
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
{		
	unsigned int tmp;
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

	PackRs1[4]  = CreateByteFromParam(aI_zar_dt[1], 0, 0.12);	// Ток заряда АБ, датчик 2
	PackRs1[5]  = CreateByteFromParam(aI_zar_dt[0], 0, 0.12);	// Ток заряда АБ, датчик 1
	PackRs1[6]  = CreateByteFromParam(aI_razr_dt[1], 0, 0.14);	// Ток разряда АБ, датчик 2
	PackRs1[7]  = CreateByteFromParam(aI_razr_dt[0], 0, 0.14);	// Ток разряда АБ, датчик 1
	PackRs1[8]  = CreateByteFromParam(vU_zru, 0, 0.48);	// Напряжение на АБ от ЗРУ
	PackRs1[9]  = CreateByteFromParam(curW_zar, -500, 17.7);	// Текущий уровень заряженности АБ
	PackRs1[10]  = CreateByteFromParam(dTemp2_zru, 0, 0.36);	// Температура корпуса ЗРУ, датчик 2
	PackRs1[11]  = CreateByteFromParam(dTemp1_zru, 0, 0.36);	// Температура корпуса ЗРУ, датчик 1

	tmp = MajorStatZRU(stat1);
	PackRs1[12] = tmp|((iUst+1)<<6);		
	PackRs1[13] = tmp>>8;
	
	tmp = MajorStatZRU(stat2);
	PackRs1[14] = tmp;		
	PackRs1[15] = tmp>>8;

	tmp = MajorStatZRU(stat3);
	PackRs1[16] = tmp;

	checksumCalc = Crc16(PackRs1, lngPackRs1-2);													// Вычисление контрольной суммы
	*(PackRs1+lngPackRs1-1) =  checksumCalc;	
	*(PackRs1+lngPackRs1-2) =  checksumCalc >> 8;													// Добавить контрольную сумму

}

//-------------------------------------------------------------------------------------------------------------------------
void InitPack1(void)	// Заполнение пакета телем. данными	
{	int i;

	for(i=4; i < lngPackRs1; i++)	{																					// nParams = 8
		PackRs1[i] = 0;
	}
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

	checksumCalc = Crc16(PackRs10, lngPackRs10-2);												// Выисление контрольной суммы
	*(PackRs10+lngPackRs10-1) =  checksumCalc;	
	*(PackRs10+lngPackRs10-2) =  checksumCalc >> 8;												// Добавить контрольную сумму
}

//-------------------------------------------------------------------------------------------------------------------------
void MakePack3(void)	// Заполнение пакета Краткой телем. данными	
{ 
// нужна ли эта корректировка?
//	if (stat1[iMUK_ZRU] & bPC)		  {	fV_AB[76].Fdata += 0.1;	}   //Коррекция напряжения 72-го АК АБ ДЛЯ БЭ	
	
	PackRs3[4]  = CreateByteFromParam(Uab, 10, 0.44);																					// Напряжение АБ, В
	PackRs3[5]  = CreateByteFromParam(P, 0, 0.26);																						// Текущее значение среднего давления в НВА, кгс/см2
	PackRs3[6]  = CreateByteFromParam(UsrAk , -1, 0.014);																				// Среднее значение напряжения по 72 АК, В
	PackRs3[7]  = CreateByteFromParam(Umax_ak, -1, 0.014);																			// Максимальное напряжение АК,В
	PackRs3[8]  = CreateByteFromParam(Umin_ak, -1, 0.014);																			// Минимальное напряжение АК, В
	PackRs3[9]  = CreateByteFromParam(dUak , 0, 0.014);																				// Разница напряжений макс и мин значений из 72 АК, В
	PackRs3[10] = CreateByteFromParam(dP, 0, 0.26);																					// Разница макс и мин давлений из 5-ти изм НВА, кгс/см2
	PackRs3[11] = CreateByteFromParam(T, -10, 0.28);																						// Текущее значение средней температуры НВАБ, С					
	PackRs3[12] = (mk_be_osn[0]<<5) |															// Состояние основного канала связи CAN МК1 ЗРУ с МК1 БЭ
								(mk_be_osn[1]<<4) |															// Состояние основного канала связи CAN МК2 ЗРУ с МК2 БЭ
								(mk_be_osn[2]<<3) |															// Состояние основного канала связи CAN МК3 ЗРУ с МК3 БЭ
								(mk_be_res[0]<<2) |															// Состояние резервного канала связи CAN МК1 ЗРУ с МК1 БЭ	
								(mk_be_res[1]<<1) |															// Состояние резервного канала связи CAN МК2 ЗРУ с МК2 БЭ	
								(mk_be_res[2]<<0);															// Состояние резервного канала связи CAN МК3 ЗРУ с МК3 БЭ		
	//5 давлений
	PackRs3[13] = CreateByteFromParam(P_array[0], 0, 0.26);
	PackRs3[14] = CreateByteFromParam(P_array[1], 0, 0.26);
	PackRs3[15] = CreateByteFromParam(P_array[2], 0, 0.26);
	PackRs3[16] = CreateByteFromParam(P_array[3], 0, 0.26);
	PackRs3[17] = CreateByteFromParam(P_array[4], 0, 0.26);
	//5 температур
	PackRs3[18] = CreateByteFromParam(T_array[0], -10, 0.28);
	PackRs3[19] = CreateByteFromParam(T_array[1], -10, 0.28);
	PackRs3[20] = CreateByteFromParam(T_array[2], -10, 0.28);
	PackRs3[21] = CreateByteFromParam(T_array[3], -10, 0.28);
	PackRs3[22] = CreateByteFromParam(T_array[4], -10, 0.28);
	//2 давления, 2 температуры
	PackRs3[23] = CreateByteFromParam(Pmax, 0, 0.26);
	PackRs3[24] = CreateByteFromParam(Pmin, 0, 0.26);
	PackRs3[25] = CreateByteFromParam(Tmax, -10, 0.28);
	PackRs3[26] = CreateByteFromParam(Tmin, -10, 0.28);

	checksumCalc = Crc16(PackRs3, lngPackRs3-2);													// Выисление контрольной суммы
	*(PackRs3+lngPackRs3-1) =  checksumCalc;	
	*(PackRs3+lngPackRs3-2) =  checksumCalc >> 8;													// Добавить контрольную сумму
}

//-------------------------------------------------------------------------------------------------------------------------
void ClearPack3(void)	// Очистить пакет Краткой телем.
{	int i;
	for(i= 4; i < lngPackRs3; i++)		PackRs3[i]  = 0;
	
	PackRs3[12] = (mk_be_osn[0]<<5) |															// Состояние основного канала связи CAN МК1 ЗРУ с МК1 БЭ
								(mk_be_osn[1]<<4) |															// Состояние основного канала связи CAN МК2 ЗРУ с МК2 БЭ
								(mk_be_osn[2]<<3) |															// Состояние основного канала связи CAN МК3 ЗРУ с МК3 БЭ
								(mk_be_res[0]<<2) |															// Состояние резервного канала связи CAN МК1 ЗРУ с МК1 БЭ	
								(mk_be_res[1]<<1) |															// Состояние резервного канала связи CAN МК2 ЗРУ с МК2 БЭ	
								(mk_be_res[2]<<0);															// Состояние резервного канала связи CAN МК3 ЗРУ с МК3 БЭ		

	checksumCalc = Crc16(PackRs3, lngPackRs3-2);													// Выисление контрольной суммы
	*(PackRs3+lngPackRs3-1) =  checksumCalc;	
	*(PackRs3+lngPackRs3-2) =  checksumCalc >> 8;													// Добавить контрольную сумму
}


//-------------------------------------------------------------------------------------------------------------------------
void MakePack4(void)	// Заполнение пакета Полной телем. данными
{		
	for(i=4; i < 76; i++)
	{
		PackRs4[i] = CreateByteFromParam(Uak_array[i-4], -1, 0.014);
	}
	
	PackRs4[76] = CreateByteFromParam(PvzRas, 38, 0.12);
	PackRs4[77] = CreateByteFromParam(P0_Ras, 0, 0.08);
	
	checksumCalc = Crc16(PackRs4, lngPackRs4-2);													// Выисление контрольной суммы
	*(PackRs4+lngPackRs4-1) =  checksumCalc;	
	*(PackRs4+lngPackRs4-2) =  checksumCalc >> 8;													// Добавить контрольную сумму
}

//-------------------------------------------------------------------------------------------------------------------------
void InitPack4(void)	// Заполнение пакета Полной телем. начальными данными
{	
	for(i=4; i < 76; i++)		{
		PackRs4[i] = 0;																											//	Напряжение 1..72-ого НВА НВАБ БЭ
	}

	checksumCalc = Crc16(PackRs4, lngPackRs4-2);													// Выисление контрольной суммы
	*(PackRs4+lngPackRs4-1) =  checksumCalc;	
	*(PackRs4+lngPackRs4-2) =  checksumCalc >> 8;													// Добавить контрольную сумму
}

//-------------------------------------------------------------------------------------------------------------------------
void ClearPack4(void)	// Заполнение пакета Полной телем. данными	
{	int i;
	for(i=4; i < lngPackRs4; i++)		{	PackRs4[i] = 0;	}										// Напряжение 1..72-ого НВА НВАБ БЭ и т.д.

	PackRs4[76] = CreateByteFromParam(PvzRas, 38, 0.12);
	PackRs4[77] = CreateByteFromParam(P0_Ras, 0, 0.08);
	
	checksumCalc = Crc16(PackRs4, lngPackRs4-2);													// Выисление контрольной суммы
	*(PackRs4+lngPackRs4-1) =  checksumCalc;	
	*(PackRs4+lngPackRs4-2) =  checksumCalc >> 8;													// Добавить контрольную сумму

}
//-------------------------------------------------------------------------------------------------------------------------
//функция пробегает по массиву, содержащему номера отказавших АК, и заполняет специальные служебные массивы, которые будут участвовать при расчете параметров в GetDataFromCan()
void BadAK_registration()
{
	unsigned char i = 0;	
		
	for(i = 0; i < 5; i++) //заполняем нулями массив, соответствующий 5 ДТ и ДД
		nBadAK_DT[i] = 0;
	for(i = 0; i < 72; i++) //заполняем нулями массив, соответствующий 72 АК
		nBadAK_U[i] = 0;

	for(i = 0; i < 5; i++) //пробегаем по всем номерам отказавших АК
	{
		if((nBadAk[i] <= 0) && (nBadAk[i] >72)) //проверяем, корректный ли номер, и не равен ли он 0
			continue; //если номер некорректный или равен нулю - просто игнорируем его и переходим к следующему элементу массива
		
		nBadAK_U[nBadAk[i]-1] = 1;  //фиксируем номер АК как отказавший, с учетом того, что индексация в массиве начинается с 0 (поэтому делаем -1)

		//проверяем, не является ли откзавший АК измерительным
		if (nBadAk[i] == 54) nBadAK_DT[0] = 1; 
		if (nBadAk[i] == 25) nBadAK_DT[1] = 1; 
		if (nBadAk[i] == 17) nBadAK_DT[2] = 1; 
		if (nBadAk[i] == 60) nBadAK_DT[3] = 1; 
		if (nBadAk[i] == 50) nBadAK_DT[4] = 1; 		
	}
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
	BadAK_registration();

	//if (cntBadAk)	{
		CurrentDlc = 5; CurrentCmd = CAN_NumBadAk;
		CAN_SendCmd(AdrMUK_ZRU, CurrentDlc, CurrentCmd);										// отправки пакета в БЭ
		bRunCmdCAN = 1;		bTimeOutCmd = 1;
//		CAN_SendCmd(AdrMUK2_ZRU, CurrentDlc, CurrentCmd);										// отправки пакета в БЭ
//		bRunCmdCAN = 1;		bTimeOutCmd = 1;
//		CAN_SendCmd(AdrMUK3_ZRU, CurrentDlc, CurrentCmd);										// отправки пакета в БЭ
//		bRunCmdCAN = 1;		bTimeOutCmd = 1;
	//}
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
//	PackRs7[5] = (int) ((W_raz - x0_p7[0])/z_p7[0]+0.5);									// Энергоемкость АБ при разряде, в том числе в ТВЦ
//	PackRs7[6] = (int) ((C_raz - x0_p7[1])/z_p7[1]+0.5);									// Емкость АБ при разряде,  в том числе в ТВЦ
//	PackRs7[7] = (tMin - x0_p7[2])/z_p7[2];																// Текущее время от начала этапа

	//временно для отладки
	tstatTVC = 18*60;
	W_raz = 300;
	C_raz = 13;
	//временно для отладки

	tMin = (float)(tstatTVC/60);																					// Текущее время от начала этапа в минутах
	PackRs7[4] = (bRestData<<7) | (bRestData_indiv<<6) | statTVC;					// Запрос на восстановление данных, ЗВД	1/0
	PackRs7[5] = CreateByteFromParam(W_raz, 0, 15.7);
	PackRs7[6] = CreateByteFromParam(C_raz, 0, 0.2);
	PackRs7[7] = CreateByteFromParam(tMin, 0, 17);
	
	checksumCalc = Crc16(PackRs7, lngPackRs7-2);													// Выисление контрольной суммы
	*(PackRs7+lngPackRs7-1) =  checksumCalc;	
	*(PackRs7+lngPackRs7-2) =  checksumCalc >> 8;													// Добавить контрольную сумму
}

//-------------------------------------------------------------------------------------------------------------------------
void TVC_restore(void)	// Восстановление ЭТВЦ
{		
	if((ETVC == 0)||(ETVC > 9)) //если приняли некорректное значение ЭТВЦ
		return; //просто выходим, даже не меняя режима работы
	
	if (ETVC)	{ 
		pOtkl_KOMP();																												// "ВКЛ КОМП"	«ВКЛ КОМП» = 1,
		pVkl_Zapr_Zarayd();																									// «ЗАПРЕТ ЗАРЯД» = 1, 
		pOtkl_Test_Zarayd();																								// «ОТКЛ ТЕСТ ЗАРЯД»,
		pVkl_Zapr_Razrayd();																								// «ЗАПРЕТ РАЗРЯД» = 1,
		pOtkl_Test_Razrayd();																								// «ОТКЛ ТЕСТ РАЗРЯД»,
		
		//сбрасываем все биты, участвующие в синхронизации
		stat4[iMUK_ZRU] = 0;	
		stat5[iMUK_ZRU] = 0;
		
		switch (ETVC)	{																									// 
//		case 	1:	StepAlgortm = bVkl_Tst_Zarayd;	
//							break;
//		case	2:	StepAlgortm = bVkl_Test_Razr;		
//							break;
//		case 	3:	
//							// Следующий этап выбирается исходя из необходимости: либо происходят наземные испытания, либо полет
//							#ifdef HOURS2 
//								StepAlgortm = bWait_2; //Наземные испытания, ждем 2 часа
//								sCount_2h = tstatTVC;			
//							#else					
//								StepAlgortm = bTst_T_NVAB; // Полет, идем на проверку температуры
//							#endif	
//							break;
//		case 	4:	StepAlgortm = bVkl_RS;		
//							stat4[iMUK_ZRU] |= bready4;
//							break;
//		case	5:	StepAlgortm = bVklKomp;				
//							break;
//		case	6:	StepAlgortm = bTest_Vosst_et6_command;		//идем на вспомогательный шаг, так как нужно сначала подать команды
//							break;
//		case	7:	StepAlgortm = bTest_Vosst_et7_command;		//идем на вспомогательный шаг, так как нужно сначала подать команды		
//							break;
//		case	8:	StepAlgortm = bVkl_Test_Razr;		
//							C_raz = 0;		W_raz = 0;			time_Razr = 0;	
//							break;
//		case	9:	StepAlgortm = bVkl_Tst_Zarayd9;
//							break;
		}
		mode = TEST; //теперь будем находится в режиме ТЕСТ
		stat1[iMUK_ZRU] &= ~bMain; //а не в основном
		stat1[iMUK_ZRU] |= bTest; 
		
		//сброс всех аварийных сообщений
		ResetAvars();
	}
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
	case st_InitZarayd:	
		
	// .......... Отключение КОМП ...........................................................................................
	case st_OtklKomp:		
		pOtkl_KOMP();
	
	// .......... Ожидание отключения КОМП Заряда ...........................................................................
	case st_WaitOtklKomp:		
		StepAlgortmZar = st_Tst_P_NVAB;
		bPauza=0;																														// Нет паузы
		break;
		
	// .......... Проверка давления АБ перед зарядом ........................................................................
	case st_Tst_P_NVAB:
		if (!bPauza) {
			if	(P <= 0.95*Pu[iUst])	{																				// Проверяем давление
				if ((iUst==2)&&(T >= Tu[2])&&(P >= 0.5*Pu[2]))	iUst=1;					// N=2
				pOtkl_Zapr_Zarayd ();																						// Запрет заряда=0		
				stat2[iMUK_ZRU] &= ~errNoOtklZar;																// Собщение "Не отключился заряд АБ"=0	
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20 сек
				StepAlgortmZar = st_Vkl_Zarayd;
			}	
			else	{																														// Если давление выше
				pVkl_Zapr_Zarayd ();																						// Запрет заряда=1
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20 сек
				StepAlgortmZar = st_OtklZar_1;
			}
		}
		break;

	// .......... Проверка отключения тока заряда ........................................................................
	case st_OtklZar_1:
		if (!bPauza) {
			if	(aI_zar > aIporog)	stat2[iMUK_ZRU] |= errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=1		
			StepAlgortmZar = st_Tst_P_NVAB;
		}
		break;

	// .......... Разрешен заряд НВАБ .......................................................................................
	case st_Vkl_Zarayd:															
		if (!bPauza) {
			if (aI_zar <= aIzard) 	
				stat3[iMUK_ZRU] &= ~errNoOgrTokZar;				// Собщение "Не ограничен ток заряда"=0, 
			else 
				stat3[iMUK_ZRU] |= errNoOgrTokZar;				// Собщение "Не ограничен ток заряда"=1
			StepAlgortmZar = st_Tst_T_NVAB_1;																		
		}
		break;

	// .......... Проверка превышения допустимой температуры ................................................................
	case st_Tst_T_NVAB_1:																										
		if	((mode_Zaryad)&&(T >= T_max)) 	{																
			if	(P <= 0.8*Pu[iUst])	{																					
				stat3[iMUK_ZRU] |= errPrevDopustT;															// "Превышение допустимой температуры НВАБ"=1
				pVkl_Zapr_Zarayd ();																						// Запрет заряда=1
				LimsCount = vsCount5;	sCount=0;		bPauza=1;											// Активация паузы 5 сек
				StepAlgortmZar = st_OtklZar_2;																		// След шаг алгоритма Ожидание включения запрета заряда
			}	
			else	{																														// Не включился заряд.
				StepAlgortmZar = st_Vkl_Zarayd_On;																// След шаг алгоритма
			}	
		}
		else	{																															// Не включился заряд.
				StepAlgortmZar = st_Vkl_Zarayd_On;																// След шаг алгоритма
		}
		break;

	// .......... Проверка отключения тока заряда ........................................................................
	case st_OtklZar_2:
		if (!bPauza) {
			if	(aI_zar > aIporog)	stat2[iMUK_ZRU] |= errNoOtklZar;					// Собщение "Не отключился Заряд АБ"=1
			LimsCount = vsCount20;	sCount=0;		bPauza=1;											// Активация паузы 20 сек			
			StepAlgortmZar = st_Tst_P_NVAB_2;
		}
		break;

	// .......... Проверка значения температуры НВАБ ........................................................................ 
	case st_Tst_P_NVAB_2:		
		if (!bPauza) {
			if	(T <= T_max-2)	{																							// Отключаем заряд. Вкл_КОМП = ОТКЛ ЗАРЯД
				pOtkl_Zapr_Zarayd ();																						// Запрет заряда=0
				stat3[iMUK_ZRU] &= ~errPrevDopustT;															// "Превышение допустимой температуры НВАБ"=0
				LimsCount = vsCount5;	sCount=0;		bPauza=1;											// Активация паузы 20сек
				StepAlgortmZar = st_Vkl_Zarayd_On;														// След шаг алгоритма Ожидание отключения запрета заряда
			}
			else {	
				StepAlgortmZar = st_Tst_P_NVAB_2;														    // зацикливаемся, пока температура не снизится
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20сек
			}
		}
		break;

	// .......... Заряжаем батарею ..........................................................................................
	case st_Vkl_Zarayd_On:
		if	(P >= Pu[iUst])	{																								
			pVkl_Zapr_Zarayd ();																							// Включить Запрет заряда=1
			LimsCount = vsCount20;	sCount=0;		bPauza=1;											// Активация паузы 20сек
			StepAlgortmZar =  st_OtklZar_3;
		}	
		else	{
			if (mode_Razryad)	
				StepAlgortmZar = st_InitZarayd;		// Если мы находимся в разряде, то начинаем алгоритм заряда заново
			else 
				StepAlgortmZar = st_Vkl_Zarayd;	// "Петля" - процесс заряда
		}
		break;

	// .......... Проверка отключения тока заряда ........................................................................
	case st_OtklZar_3:
		if (!bPauza) {
			if	(aI_zar > aIporog)	stat2[iMUK_ZRU] |= errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=1		
			StepAlgortmZar = st_ViborUst;
		}
		break;		
		
	// .......... Выбор уставок температуры и давления.......................................................................
	case st_ViborUst:
		if	((dP >= dP_u)||(dUak>=dU_u))	iUst=2;														// N=3
		else	{
			if	(T >= Tu[1])								iUst=0;														// N=1
			else														iUst=1;														// N=2
		}	
		StepAlgortmZar = st_Tst_P_NVAB;
		break;

	
	default:
		StepAlgortmZar = st_InitZarayd;																			// Переход на начало алгоритма
		break;

	} //end of switch (StepAlgortm)
	
}	// End of Zaryd_NVAB


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Zaryd_NVAB_noCAN (void)											/* _З_А_Р_Я_Д___Н_В_А_Б_ по проводным командам */
{
	switch (StepAlgortmZar)																								// Обработчик состояний алгоритма заряда
	{
	// .......... Инициализация алгоритма заряда ............................................................................
	case st_InitZarayd:
		bPauza = 0;
	// .......... Отключение КОМП ...........................................................................................
	case st_OtklKomp:
		if (!bPauza) {		
			pOtkl_KOMP();
		}
		else 
			break;
	
// .......... Ожидание отключения КОМП Заряда ...........................................................................
//	case bWaitOtklKomp:		

//		if (!bPauza) {
//			if	(aI_zar <= aIkomp)	stat2[iMUK_ZRU] |= errNoOtklCompZar;			// Собщение "Не отключился КОМП Заряд АБ"=1
//			StepAlgortmZar = bTst_P_NVAB;
//		}
//		break;
		
	// .......... Ожидание снятия запрета заряда ............................................................................
	case st_Tst_P_NVAB:
		if	(!ZaprZarProv)	{																								// Включение заряда
			pOtkl_Zapr_Zarayd ();																						// Запрет заряда=0
			LimsCount = vsCount20;	sCount=0;		bPauza=1;											// Активация паузы 5 сек
			StepAlgortmZar = st_OtklZar_1;
		}	
		else	{																														// Ожинание снятия Запрета Заряда
			LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20 сек
			StepAlgortmZar = st_OtklKomp;
		}
		break;

	// .......... Ожидание отключения запрета Заряда ........................................................................
	case st_OtklZar_1:		
		if (!bPauza) {
			StepAlgortmZar = st_Vkl_Zarayd;
		}
		break;
		
	// .......... Разрешен заряд НВАБ .......................................................................................
	case st_Vkl_Zarayd:																											// ОтклЗапЗар
		if (!bPauza) {
			if (aI_zar <= aIzard) stat3[iMUK_ZRU] &= ~errNoOgrTokZar;				// Собщение "Не ограничен ток заряда"=0, aIzard = 25
			else									stat3[iMUK_ZRU] |= errNoOgrTokZar;				// Собщение "Не ограничен ток заряда"=1
			StepAlgortmZar = st_Tst_T_NVAB_1;																			// След шаг алгоритма
		}
		break;

	// .......... Проверка запрета заряда ...................................................................................
	case st_Tst_T_NVAB_1:																											// ОтклЗапЗар
		if	(ZaprZarProv)	{																										// Отключаем заряд. Вкл_КОМП = ОТКЛ ЗАРЯД
				pVkl_Zapr_Zarayd ();																						// Запрет заряда=1
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20 сек
				StepAlgortmZar = st_OtklZar_2;															// След шаг алгоритма Ожидание включения запрета заряда
		}
		else	{																															// Не включился заряд.
			if (mode_Razryad)		StepAlgortmZar = st_InitZarayd;									// Выход из "Петли"
			else	{							StepAlgortmZar = st_Vkl_Zarayd;									// "Петля" - процесс заряда
				LimsCount = vsCount20;	sCount=0;		bPauza=1;										// Активация паузы 20 сек
			}
		}
		break;

	// .......... Ожидание включения запрета Заряда ...........................................................................
	case st_OtklZar_2:		
		if (!bPauza) {
			if	(aI_zar > aIporog)	stat2[iMUK_ZRU] |= errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=1
			else stat2[iMUK_ZRU] &= ~errNoOtklZar;						// Собщение "Не отключился Заряд АБ"=1
			StepAlgortmZar = st_ViborUst;																				// След шаг алгоритма Ожидание нормализации температуры
		}
		break;

	// .......... Выбор уставок температуры и давления.......................................................................
	case st_ViborUst:
		iUst=1;
		StepAlgortmZar = st_InitZarayd;
		break;

	default:
		StepAlgortmZar = st_InitZarayd;																			// Переход на начало алгоритма
		break;
	
	} //end of switch (StepAlgortm)
	
}	// End of Zaryd_NVAB


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Razryd_NVAB (void)													/* _Р_А_З_Р_Я_Д___Н_В_А_Б_ */
{	
	switch (StepAlgortmRazr)																							// Переключатель состояний алгоритма разряда
	{
	// .......... Инициализация разряда .....................................................................................
	case st_InitRazryad:
		pOtkl_Zapr_Razrayd();
		stat2[iMUK_ZRU] &= ~errNoOtklRazr; 																// Не отключился разряд АБ = 0
		LimsCount_R = vsCount20;	 sCount_R=0;	bPauza_R=1;								// Включене паузы 20 сек
		StepAlgortmRazr = st_Tst_I_Razryda;
		break;
	
	// .......... Проврка ограничения тока разряда ..........................................................................
	case st_Tst_I_Razryda:
	if (!bPauza_R) {	
			if (aI_razr > aIrazr_ogrn) 
				stat3[iMUK_ZRU] |= errNoOgrTokRazr;			// Собщение "Не ограничен ток разряда" = 1
			else											 
				stat3[iMUK_ZRU] &= ~errNoOgrTokRazr;		// Собщение "Не ограничен ток разряда" = 0
			LimsCount_R = dt1; 	 sCount_R=0;	bPauza_R=1;												
			calc_dt = calc_dt1; //дельта времени соответствует 1 секундам
			Uab_old = Uab;		aI_razrOld = aI_razr;
			StepAlgortmRazr = st_Tst_U_Razryda;																		// Переход на разряд с подсчётом C и W
		}
		break;
	
	// .......... Р_а_з_р_я_д АБ, контроль напряжения АБ ....................................................................
	case st_Tst_U_Razryda:
		if (!bPauza_R) {																										// Пауза 			
			Calculation();
			if	((Uab <= 72)||(Umin_ak <= 0.2))	{	
				StepAlgortmRazr = st_Otkl_Razrayd;
			}	
			else {																																	
				if	(T >= 50)	
	     			stat3[iMUK_ZRU] |=  errPrevDopustT;								// "Превышение допустимой температуры АБ" - процесс разряда
				else	{				
					if (mode_Razryad)	 stat3[iMUK_ZRU] &= ~errPrevDopustT;			//снимаем сообщение об аварии только если мы находились в режиме разряд				
				}
				LimsCount_R = dt1; 	 sCount_R=0;	bPauza_R=1;												
				calc_dt = calc_dt1; //дельта времени соответствует 1 секундам
				StepAlgortmRazr = st_Tst_U_Razryda; //зацикливаемся
			}	
		}	
		break;

	// .......... Отключаем разряд ..........................................................................................
	case st_Otkl_Razrayd:		
	  pVkl_Zapr_Razrayd();																								// Запрет РАЗРЯДА = 1
		LimsCount_R = vsCount20;	 sCount_R=0;	bPauza_R=1;									// Сброс счётчика , включене паузы 20 сек
		StepAlgortmRazr = st_OtklRaz_Inspect;
		break;		
		
	// .......... Проврка отключения тока разряда ..........................................................................
	case st_OtklRaz_Inspect:
		
		if (!bPauza_R) {
			if (aI_razr > aIporog) stat2[iMUK_ZRU] |= errNoOtklRazr;					//
			else									 stat2[iMUK_ZRU] &= ~errNoOtklRazr;
			StepAlgortmRazr = st_Tst_U_Razryda_end;															// Переход на ожидание подъёма напряжения АБ до 80В
			LimsCount_R = vsCount20;	 sCount_R=0;	bPauza_R=1;								// Сброс счётчика , включене паузы 20 сек
		}	
		break;		

	// .......... Проверка роста напряжения при запрете разряда ............................................................
	case st_Tst_U_Razryda_end:
		
		if (!bPauza_R) {
			if	(Uab >= 88)	{																									// Напряжение АБ достигло 88В
				StepAlgortmRazr = st_InitRazryad;																	// Переход на начало алгоритма
			}	
			else	{																														//
				StepAlgortmRazr = st_Otkl_Razrayd;																// Переход на запрт разряда
			}		
		}
		break;
	
	default:
		StepAlgortmRazr = st_InitRazryad;																			// Переход на начало алгоритма
		break;
		
	} //end of switch (StatusZarRazr)
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//-------------------------------------------------------------------------------------------------------------------------
void Razryd_NVAB_noCAN (void)													/* _Р_А_З_Р_Я_Д___Н_В_А_Б_ по проводным командам */
{

	switch (StepAlgortmRazr)																							// Переключатель состояний алгоритма разряда
	{

	// .......... Инициализация разряда .....................................................................................
	case st_InitRazryad:		
		StepAlgortmRazr = st_UabCheck;
		break;

	// .......... Контроль Uаб .....................................................................................
	case st_UabCheck:
			if	((vU_zru >= 88) && (!ZaprRazrProv))	{														// Напряжение АБ достигло 88В И нет запрета разряда ЗРП = 0	
				StepAlgortmRazr = st_WaitPause;																		// Начинаем ждать 20 секунд				
			}	
			else	{																														//
				StepAlgortmRazr = st_Otkl_Razrayd;																// Переход на запрет разряда
			}				
		break;	

			
	// .......... Задержка ............................................................
	case st_WaitPause:		
		if (!bPauza_R) {																							// Если дождались 
			pOtkl_Zapr_Razrayd();		
			StepAlgortmRazr = st_Tst_I_Razryda;												// Переход на отключение запрета Разряда
		}
		break;			
		
	// .......... Проврка ограничения тока разряда ..........................................................................
	case st_Tst_I_Razryda:		
		if (aI_razr > aIrazr_ogrn) {stat3[iMUK_ZRU] |= errNoOgrTokRazr;			// aIrazr_ogrn = 30 Собщение "Не ограничен ток разряда"
		}	
		else											 {stat3[iMUK_ZRU] &= ~errNoOgrTokRazr;
		}	
		LimsCount_R = dt5; 	 sCount_R=0;	bPauza_R=1;												/*Razr;*/
		calc_dt = calc_dt5; //дельта времени соответствует 5 секундам		
		vU_zru_Old = vU_zru;		aI_razrOld = aI_razr;
		StepAlgortmRazr = st_ZRPCheck;																		// Переход на контроль ЗРП с подсчётом C и W
		break;

	// .......... Контроль ЗРП ....................................................................
	case st_ZRPCheck:

		if (!bPauza_R) {																										// Пауза 5 сек		
			if	(ZaprRazrProv)	{																									//	Если есть запрет разряда ЗРП = 1
				StepAlgortmRazr = st_Otkl_Razrayd;
			}	
			else	{																														// ((P < 3)||(U <= 76))
				Calculation_noCAN();
				StepAlgortmRazr = st_WaitPause;														// По сути возвращаемся в начало алгоритма
			}	
		}	
		break;

	// .......... Отключаем разряд ..........................................................................................
	case st_Otkl_Razrayd:		
	  pVkl_Zapr_Razrayd();																								// Запрет РАЗРЯДА = 1
		LimsCount_R = vsCount20;	 sCount_R=0;	bPauza_R=1;									// Сброс счётчика , включене паузы 20 сек
		StepAlgortmRazr = st_OtklRaz_Inspect;
		break;

	// .......... Проврка ограничения тока разряда ..........................................................................
	case st_OtklRaz_Inspect:		
		if (!bPauza_R) {
			if (aI_razr > aIporog) {stat2[iMUK_ZRU] |= errNoOtklRazr;					// aIkomp = 2
			}	
			else									{stat2[iMUK_ZRU] &= ~errNoOtklRazr;
			}	
			StepAlgortmRazr = st_UabCheck;															// Переход на ожидание подъёма напряжения АБ до 80В
		}	
		break;
	
	default:
		StepAlgortmRazr = st_InitRazryad;																			// Переход на начало алгоритма
		break;

	} //end of switch (StatusZarRazr)
}

//-------------------------------------------------------------------------------------------------------------------------
void OneSecAdd (void)													/* Добавить секунду */
{	int i;
	AddSec = 0;		bOneSec = 1;
	
	//......................................................................
	if (cntReadyWrk < PauseReadyWrk) cntReadyWrk++;
	else	bReadyWrk = 1; 
	
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
	for (i=0;i<3;i++)	{																										// Установка/сброс флагов неработоспособности CAN узлов
		if(i != iMUK_ZRU) //если это не наш МК, а один из двух других
		{
			if (cntSec_noMK_ZRU[i] > 4)	//если прошло достаточно секунд ожидания связи, то считаем, что у этих МК ЗРУ нет связи с МК БЭ
			{
				cntSec_noMK_ZRU[i] = 0;	
				mk_be_osn[i] = 1; 
				mk_be_res[i] = 1;
			}	
			else	{cntSec_noMK_ZRU[i]++;	}	//увеличиваем счетчик секунд
		}
	}	
	
	//проводные запреты
	pNotCan(); //считали состояние проводных линий раз в секунду
	
	//заряд
	if(bitNotZar != Prev_bitNotZar) //если изменилось состояние по сравнению с предыдущим
	{
		cntZarProv = 0; //обнулили счетчик секунд
		ZaprZarProv = 0; //запрет заряда проводной = 0
	}
	Prev_bitNotZar = bitNotZar; //запомнили состояние
	
	if(cntZarProv > 10) //если долго не мигали
	{
		cntZarProv = 0; //обнуляем счетчик секунд
		ZaprZarProv = 1; //запрет заряда проводной = 1
	}
	
	//разряд
	if(bitNotRaz != Prev_bitNotRaz) //если изменилось состояние по сравнению с предыдущим
	{
		cntRazrProv = 0; //обнулили счетчик секунд
		ZaprRazrProv = 0; //запрет разряда проводной = 0
	}
	Prev_bitNotRaz = bitNotRaz; //запомнили состояние
	
	if(cntRazrProv > 10) //если долго не мигали
	{
		cntRazrProv = 0; //обнуляем счетчик секунд
		ZaprRazrProv = 1; //запрет разряда проводной = 1
	}	
	//~проводные запреты	
	
	
	//......................................................................
	sCount_2h++;
	
	//!!!нужно внимательно следить, чтобы bPauza_TVC и bPauza не были запущены одновременно, иначе sCount будет увеличиваться два раза
	//!!!да и LimsCount у них тоже общий. Либо нужно делать два независимых счетчика и два независимых порога для счетчиков
	//......................................................................
	if (bPauza_TVC)	{	sCountTVC++;
		if (sCountTVC >= LimsCountTVC)	{ sCountTVC = 0;	bPauza_TVC = 0;}	}		// Обслуживание паузы ТВЦ

	//......................................................................
	if (bPauza)	{	sCount++;
		if (sCount >= LimsCount)	{ sCount = 0;	bPauza = 0;}	}									// Обслуживание универсальной паузы

	//......................................................................
	if (bPauza_R)	{	sCount_R++;
		if (sCount_R >= LimsCount_R)	{ sCount_R = 0;	bPauza_R = 0;}	}					// Обслуживание паузы РАЗРЯДА
	
	//......................................................................
	mCountSecMain++;																													// Счётчик 1 мин
	if (mCountSecMain == 59)	{	mCountSecMain = 0;	mCount5Main++;						// Счётчик 5 мин в main
			time_Razr++;
	}
	//......................................................................
//	if (bPauza5)	{	sCount5++;
//		if (sCount5 >= 5)	{ sCount5=0;	bPauza5=0;}	}													// Обслуживание паузы 5 сек
	//......................................................................
	sTime.sec++;
  if (sTime.sec == 59)	{		sTime.sec = 0;																	// Прошло 60 сек
		if (sTime.min < 59)		{	sTime.min++;	}																	// mCount5Main++;}
		else	{									sTime.min = 0;	
			if (sTime.hour < 23)	{	sTime.hour++;	}
			else	{									sTime.hour = 0;		NewDay=1;										// Смена суток
				Date_Update();     																									// Increment the date
			}
		}	
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Инициализация переменных
void Var_init()
{	
	int i,j;
	
	iMUK_ZRU = nMUK_ZRU-1;
	switch (iMUK_ZRU)	{																										// 
	case 0:	iMUK_ZRU2=1; iMUK_ZRU3=2;	break;															// 
	case 1:	iMUK_ZRU2=0; iMUK_ZRU3=2;	break;															// 
	case 2:	iMUK_ZRU2=0; iMUK_ZRU3=1;	break;															// 
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
	stat5[iMUK_ZRU] = 0;


	//при начале работы программы контроллер выставляет запрос на восстановление данных, и индивидуальный и общий (ведь достаточно, чтобы хотя бы в одном МК появился этот запрос)
	bRestData = 1;																												// 1 - восстановить данные, 
	bRestData_indiv = 1;																									// индивидуальный запрос выставляется только здесь - задача показать, что именно этот МК сформировал запрос	
	if(bRestData_indiv)
		stat3[iMUK_ZRU] |= RestData; // Флаг на восстановление данных, обмениваемся индивидуальными значениями между МУКами
	else
		stat3[iMUK_ZRU] &= ~RestData;; // Флаг на восстановление данных, обмениваемся индивидуальными значениями между МУКами	

	ind_pack1 = 0;						ind_pack2 = 0;
	lngPack1 = Npack_Cmd-1;		lngPack2 = Npack_Cmd-1;											// Длина пакета RS485 максимальная

	bPauza = 0;
	bPauza20 = 0;																													// Флаг пауза 20 сек
	bOneSec = 0;																													// Флаг для чтения параметров АБ в БЭ через 1 сек
	
	EndTVC = 0; 																														// Флаг начала процесса окончания ТВЦ
	
	//заполняем нулями всю телеметрию, касающуюся БЭ
	for(i = 0; i < nMUKBE; i++) 
	{
		nfRec_CanDatch1_All[i] = 0;
		nfRec_CanDatch2_All[i] = 0;
		nfRec_CanAK1_All[i] = 0;
		nfRec_CanAK2_All[i] = 0;
		
		for(j = 0; j < nFrameDatchCAN; j++) 
			Reciev_CanDatch_All[i][j].data64 = 0;
		
		for(j = 0; j < nFrameABCAN; j++) 
			Reciev_CanAB_All[i][j].data64 = 0;
	}
	
	//параметры, измеряемые ЗРУ
	aI_razr_dt[0] = aI_razr_dt[1] = aI_zar_dt[0] = aI_zar_dt[1] = 0; //ток
	aI_razr = aI_razrOld = aI_zar = 0;
	vU_zru = vU_zru_Old = 0; //напряжение
	dTemp1_zru = dTemp2_zru = 0; //температура
	
	//проводные запреты
	cntZarProv = cntRazrProv = 0; //счетчики секунд проводных линий
	ZaprZarProv = ZaprRazrProv = 0; //при начале работы запретов нет
	Prev_bitNotZar = bitNotZar = 0;
	Prev_bitNotRaz = bitNotRaz = 0;
}	

//-------------------------------------------------------------------------------------------------------------------------
void ZRU_Init(void)
{
	uint32_t vkl;
	Init_Param();
	/*	ОТКЛ.АБ, ОТКЛ.СЭС, ВКЛ КОМП, ВКЛ ЗАПРЕТ ЗАРЯД, ВКЛ ЗАПРЕТ РАЗРЯД, ВЫКЛ ТЕСТ ЗАРЯД, ВЫКЛ ТЕСТ РАЗРЯД, ЗАПР ШИМ ЗРУ. №уст = 2 */
	
	vkl = ((1<<3)& MDR_PORTA->RXTX);	//считываем состояние РА3
	if(vkl)
		pVkl_Shim_ZRU(0);
	else
		pOtkl_Shim_ZRU(0);																										// «Вкл АБ» = 0, «Подкл СЭС» = 0, «Разр ШИМ ЗРУ» = 0 
	pVkl_KOMP();																													// "ВКЛ КОМП"	«ВКЛ КОМП» = 1,
	pVkl_Zapr_Zarayd();																										// «ЗАПРЕТ ЗАРЯД» = 1, 
	pVkl_Zapr_Razrayd();																									// «ЗАПРЕТ РАЗРЯД» = 1,
	pOtkl_Test_Zarayd();																									// «ОТКЛ ТЕСТ ЗАРЯД»,
	pOtkl_Test_Razrayd();																									// «ОТКЛ ТЕСТ РАЗРЯД»,
	pOtkl_RS(0);
	Wait(30);																															// Ожидание завершения коммутации силовый цепей

	Var_init();																														// Инициализация переменных

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
						
						switch (cmd)	{
						case gVkl_ZRU:				mode = Vkl_ZRU;  									 						// 0x1 Вкл_ЗРУ
											if (!tVkl_ZRU) tVkl_ZRU = 10;				break;								// 1 сек
						case gOtkl_ZRU:				mode = Otkl_ZRU;				break;			 					// 0x2 Откл_ЗРУ
						case gVkl_Test:	 																										// 0x3 Вкл_ТЕСТ. Первый шаг алгоритма ТВЦ
							if(DataOk)
							{
								mode = initTEST;									
								ETVC = (0xf0 & pack1[4]) >> 4;																// ЭТВЦ содержится в 5 байте принятого пакета команда
							}
							break;		
						case gOtkl_Test: 			mode = Otkl_TEST;				break;								// 0x4 Откл_ТЕСТ
						case gOtkl_RS:	 			pOtkl_RS(0);						break;								// 0x5 ОТКЛ РС 	отключать разрядные сопротивления в БЭ mode = Otkl_RS;
						case gVkl_Podzarayd:	if(DataOk)	mode = initPodzarayd;		break;		// 0x6 ВКЛ Подзаряд
						case gOtkl_Podzarayd:	mode = Otkl_Podzarayd;	break;								// 0x7 ОТКЛ Подзаряд
						case OZVD:						bRestData = 0; bRestData_indiv = 0; break;		// 0x8 «Отключение запроса на восстановление данных»
						}
												p_ParRs1 = PackRs2;	BatchSize1 = lngPackRs2;	break;
		
	case 	gStat_AB_short:	p_ParRs1 = PackRs3;	BatchSize1 = lngPackRs3;	break;		// Сост_АБ_Кратк_БЭ - краткая телеметрия БЭ

	case 	gStat_AB_Full:	p_ParRs1 = PackRs4;	BatchSize1 = lngPackRs4;	break;		// Сост_АБ_Полн_БЭ – полная телеметрия БЭ

	case	gUstavki_Curr:	bUstavkiBCU = 1;			p_InPack = pack1+4;								// Уставки_Текущ - управление БЭ
												p_ParRs1 = PackRs5;	BatchSize1 = lngPackRs5;	break;
		
	case	gUstavki_Tst:		p_ParRs1 = PackRs6;	BatchSize1 = lngPackRs6;	break;		// контроль параметров (уставок) алгоритмов ЗРУ

	case	gSaveData_to_BCU:
												p_ParRs1 = PackRs7;	BatchSize1 = lngPackRs7;	break;		// 0x7	запоминаемые для восстановления данные в БВС 
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

						switch (cmd)	{
						case gVkl_ZRU:				mode = Vkl_ZRU;  												 			// 0x1 Вкл_ЗРУ
											if (!tVkl_ZRU) tVkl_ZRU = 10;				break;								// 1 сек
						case gOtkl_ZRU:				mode = Otkl_ZRU;				break;			 					// 0x2 Откл_ЗРУ
						case gVkl_Test:	 																										// 0x3 Вкл_ТЕСТ. Первый шаг алгоритма ТВЦ
							if(DataOk)
							{
								mode = initTEST;									
								ETVC = (0xf0 & pack1[4]) >> 4;																// ЭТВЦ содержится в 5 байте принятого пакета команда
							}
							break;														
						case gOtkl_Test: 			mode = Otkl_TEST;				break;								// 0x4 Откл_ТЕСТ
						case gOtkl_RS:	 			pOtkl_RS(0);						break;								// 0x5 ОТКЛ РС 	отключать разрядные сопротивления в БЭ mode = Otkl_RS;
						case gVkl_Podzarayd:	if(DataOk)	mode = initPodzarayd;		break;		// 0x6 ВКЛ Подзаряд
						case gOtkl_Podzarayd:	mode = Otkl_Podzarayd;	break;								// 0x7 ОТКЛ Подзаряд
						case OZVD:						bRestData = 0; bRestData_indiv = 0; break;		// 0x8 «Отключение запроса на восстановление данных»
						}
												p_ParRs2 = PackRs2;	BatchSize2 = lngPackRs2;	break;
		
	case 	gStat_AB_short:	p_ParRs2 = PackRs3;	BatchSize2 = lngPackRs3;	break;		// Сост_АБ_Кратк_БЭ - краткая телеметрия БЭ

	case 	gStat_AB_Full:	p_ParRs2 = PackRs4;	BatchSize2 = lngPackRs4;	break;		// Сост_АБ_Полн_БЭ – полная телеметрия БЭ

	case	gUstavki_Curr:	bUstavkiBCU = 1;			p_InPack = pack2+4;								// Уставки_Текущ - управление БЭ
												p_ParRs2 = PackRs5;	BatchSize2 = lngPackRs5;	break;
		
	case	gUstavki_Tst:		p_ParRs2 = PackRs6;	BatchSize2 = lngPackRs6;	break;		// контроль параметров (уставок) алгоритмов ЗРУ

	case	gSaveData_to_BCU:
												p_ParRs2 = PackRs7;	BatchSize2 = lngPackRs7;	break;		// 0x7	запоминаемые для восстановления данные в БВС 
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
{	
	unsigned int res=0;
	unsigned char is, st0, st1, st2;
	unsigned char maska, bit_i, bit_m;	//bit_i - индивидуальное значение, bit_m - мажоритированное

	bit_m = 0;
	bit_i = 0;
		
	for (is=0;is<8;is++)	{
		maska = 1<<is;

		bit_i = (maska & stat[iMUK_ZRU])>>is;
		
		st0 = (maska & stat[0])>>is;
		st1 = (maska & stat[1])>>is;
		st2 = (maska & stat[2])>>is;

		if ( (st0==st1) && (st0==st2) )	{ //если мажоритар совпал
				bit_m = st0;
		}
		else	{
			if (st0==st1)	{		bit_m = st0;	}
			if (st0==st2)	{		bit_m = st0;	}
			if (st1==st2)	{		bit_m = st1;	}
		}		
		
		res |= (bit_m<<(is*2)) | (bit_i<<((is*2)+1));
	}
	return res;
}

/**************************************************************************************************************************
***************************************************************************************************************************/
int main(void)
{	
	int bZarRazr;//	unsigned int tmp;
	
	Clock_Init();																													// Инициализация тактового генератора
	Ports_Init();
//	Ports_Init_Tst(); 																									// Инициализация портов для отладочной платы
	ADC_Init();
	UART1_Init();	
	UART2_Init();	
 	CAN1_Init(); 
 	CAN2_Init();
	SysTickInit(100000);
#ifdef WATCH_DOG
	initInternalWatchdog();																								// Запуск сторожевого таймера
#endif	
	
	ZRU_Init();																														// 

	mode = Init_Run;																											// Начальрый режим
	cntReadyWrk = 0;																											// Счётчик задержки 3 секунды
	bPauza5 = 1;	bOneSec = 0;
	calc_dt = calc_dt5; 																									//по умолчанию дельта времени для расчета W и С будет соответствовать 5 секундам
	
	MakePack2_5_8_10();
	MakePack3();	InitPack4();
	InitPack1();	MakePack6();
	
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
				GetDataFromCan(); 																							// Забираем из пакетов нужную нам телеметрию
				MakePack3();	MakePack4();																			// if ((!bReqBCU[0])&&(!bReqBCU[1]))	{	MakePack3();	MakePack4(); }
				cnt_can=0;	updateD1=0;		updateD2=0;
				if (mode == CAN_not_working)	{mode = Init_Run;}
				DataOk = 1;
			}
			// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
			else {																														// Данные телеметрии НЕ получены
				if (cnt_can<4)	{	cnt_can++;	}
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
						StepAlgortmZar = st_InitZarayd;			
						StepAlgortmRazr = st_InitRazryad;
						mode = CAN_not_working;																				// Сообщение в БЦУ "Ошибка приёма телеметрии АБ"
					}
				}
			}
			// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .			
			MakePack6();
			MakePack7();																											//if ((!(MDR_PORTA->RXTX & 0x20))&&(!(MDR_PORTF->RXTX & 0x04)))	{	MakePack7(); }
		}									

		//.......................................................... Контроль обновления данных телеметрии.....................
		//МК ЗРУ больше не подтверждают прием пакетов телеметрии от МК БЭ, поэтому функции CAN_SendConf_1 закомментированы
		if (nfRec_CanDatch1 == okFrameDatch)	{															// Получены все фреймы
			add_nbuf = 0;
			//CAN_SendConf_1(1);																								// Подтверждение: Даные от МУК БЭ получены
			nfRec_CanDatch1 = 0;																							// Сброс битов получения фреймов
			updateD1 = 1;																											// Установить флаг "Обновление данных от МУК БЭ"
		}
		if (nfRec_CanAK1 == okFrameAK)	{																		// Получены все фреймы
			add_nbuf = 1;
			//CAN_SendConf_1(2);																								// Подтверждение: Даные от МУК БЭ получены
			nfRec_CanAK1 = 0;																									// Сброс битов получения фреймов
			updateD2 = 1;																											// Установить флаг "Обновление данных от МУК БЭ"
		}
		
		if (nfRec_CanDatch2 == okFrameDatch)	{															// Получены все фреймы
			add_nbuf = 0;
			//CAN_SendConf_2(1);																								// Подтверждение: Даные от МУК БЭ получены
			nfRec_CanDatch2 = 0;																							// Сброс битов получения фреймов
			updateD1 = 1;																											// Установить флаг "Обновление данных от МУК БЭ"
		}
		if (nfRec_CanAK2 == okFrameAK)	{																		// Получены все фреймы
			add_nbuf = 1;
			//CAN_SendConf_2(2);																								// Подтверждение: Даные от МУК БЭ получены
			nfRec_CanAK2 = 0;																									// Сброс битов получения фреймов
			updateD2 = 1;																											// Установить флаг "Обновление данных от МУК БЭ"
		}

		//определяем, есть ли связь у этого МК ЗРУ с одноименным МК БЭ
		switch(iMUK_ZRU+1)
		{
			case nMUK1_ZRU: //если это МК1 ЗРУ
				mk_be_osn[iMUK_ZRU] = ((bNoWrkCAN >> 0) & 0x1);	
				mk_be_res[iMUK_ZRU] = ((bNoWrkCAN >> 1) & 0x1);
				break;
			
			case nMUK2_ZRU: //если это МК2 ЗРУ
				mk_be_osn[iMUK_ZRU] = ((bNoWrkCAN >> 2) & 0x1);	
				mk_be_res[iMUK_ZRU] = ((bNoWrkCAN >> 3) & 0x1);
				break;			
			
			case nMUK3_ZRU: //если это МК3 ЗРУ
				mk_be_osn[iMUK_ZRU] = ((bNoWrkCAN >> 4) & 0x1);	
				mk_be_res[iMUK_ZRU] = ((bNoWrkCAN >> 5) & 0x1);
				break;		
			
			default:
				mk_be_osn[iMUK_ZRU] = 1;	
				mk_be_res[iMUK_ZRU] = 1;
				break;
		}
		
		//.....................................................................................................................
		switch (mode)																												// Обработчик состояний
		{
			case Init_Run:																										// Инициализация всех процессов при старте или сбоях
				Var_init();																											// Инициализация переменных
		
			case START:																												// Начальный запуск
				stat1[iMUK_ZRU] |= bMain;
				stat2[iMUK_ZRU] = 0;																						// Сброс флагов ошибок
				stat3[iMUK_ZRU] &= ~(errNoOgrTokRazr|errNoOgrTokZar|errPrevDopustT);  											
				StepAlgortmZar = st_InitZarayd;																		// StepAlgortmZar = bInitZarayd
				StepAlgortmRazr = st_InitRazryad;
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
					pVkl_Shim_ZRU(0);
				}	
				break;
			
			case Otkl_ZRU:																										// Отключить ЗРУ от АБ (отключить силовые ключи ЗРУ)
				pOtkl_Shim_ZRU(0);
				mode = Work;
				break;
			
			case initTEST:																										// Запуск подпрограммы ТВЦ АБ (определение ёмкости АБ) 
				TVC_restore(); //получив команду ВКЛ ТЕСТ, мы теперь всегда принимаем ЭТВЦ. Эта функция выбирает, с какого этапа начнется/продолжится тестирование	
				
				
			case TEST:																												// Запуск подпрограммы ТВЦ АБ (определение ёмкости АБ) 
				//if (!bPauza_TVC) //непонятно, зачем нужно
					Test_NVAB();
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
					LimsCountTVC = vsCount20;		sCountTVC=0;		bPauza_TVC=1; 					// Подготовавливаем паузу 20 секунд			
					
					// По принятии команды Откл.Тест или при выходе из алгоритма тестирования вызываем эти функции
					pVkl_Zapr_Zarayd();																							// «ЗАПРЕТ ЗАРЯД»  = 1
					pVkl_Zapr_Razrayd();																						// «ЗАПРЕТ РАЗРЯД» = 1
					pOtkl_Test_Zarayd();																						// "Откл_ТЕСТ ЗАР" 
					pOtkl_Test_Razrayd();																						// "Откл_ТЕСТ РАЗР" 		
					
					pOtkl_RS(0); 

					EndTVC = 1; //Устанавливаем флаг того, что процесс окончания ТВЦ запущен
				}
				break;

			case initPodzarayd:																								// Запуск подпрограммы Подзаряд АБ
				if (stat1[iMUK_ZRU] & bTest)				mode = TEST;
				else	{
					if (!(stat1[iMUK_ZRU] & bPodzaryad))	{						
						stat1[iMUK_ZRU] &= ~bMain;
						stat1[iMUK_ZRU] |= bPodzaryad;	StepAlgortmPodzar = st_p_InitPodzar;
					}
					ResetAvars(); //сбрасываем все аварийные сообщения
					mode = Vkl_Podzarayd;
				}
				break;
				
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
				mode = Init_Run;																								// 
				break;
			
			case RESTART:																											// Рестарт UART и переход в прерванный режим
				mode = Init_Run;																								// 
				break;
			
			case CAN_not_working:																							// Отказ CAN				
				bZarRazr = !bZarRazr;
				if (bZarRazr)		Zaryd_NVAB_noCAN();															//
				else						Razryd_NVAB_noCAN();														//
				break;
		} //end of switch (mode)
				
		//..............................................................................................................................
 		if (bReqBCU[0] && bReadyWrk)		{																		// Запрос от Uart1 БЦУ
			checksumCalc = Crc16(pack1, lngPack1-2);													// Выисление контрольной суммы
			checksumIn = pack1[lngPack1-2];																		// Принятая контрольная сумма 
			checksumIn = (checksumIn<<8) | pack1[lngPack1-1];									// Принятая контрольная сумма 
			if (checksumCalc==checksumIn)	{
				WrkCmd_1();																											// Отработать команду от Uart1 БЦУ и отправить пакет по RS-485 в БЦУ
			}
			bReqBCU[0] = 0;		lngPack1 = Npack_Cmd-1;													// Длина пакета RS485 максимальная
		}

 		if (bReqBCU[1] && bReadyWrk)		{	
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
		
		#ifdef WATCH_DOG
			resetInternalWatchdog();																						// Перезапуск (сброс) внутреннего сторожевого таймера.
		#endif			
	}
}
/* END OF FILE Main.c */

