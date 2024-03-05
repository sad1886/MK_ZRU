/**
  ******************************************************************************
  * @file    Work.h
  * @author  ЗАО Орбита Карташов Ю.Д.
  * @version V1.0.0
  * @date    03.07.2014
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------------------------------------------------*/
#ifndef WORK_H
#define WORK_H
/* Includes -------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//----------- Версия прошивки ----------------------------------------------------
#define Version 7.2.2

//************__В_Ы_Б_О_Р___НОМЕРА_ЗРУ_и_НОМЕРА__МУК__**********************************************************************

#define HW_ZRU01001
//#define HW_ZRU01002

//. . . . . . . . . . . . . . . . . . . . . .
#define nMUK_ZRU			nMUK1_ZRU							// МУК1 ЗРУ
//#define nMUK_ZRU			nMUK2_ZRU							// МУК2 ЗРУ
//#define nMUK_ZRU			nMUK3_ZRU							// МУК3 ЗРУ

//************Режим работы**********************************************************************
#define HOURS2 //если дефайн определен, то происходят испытания и в алгоритме ТВЦ в 3-ем этапе есть задержка 2 часа, иначе имеем штатную прошивку

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//----------- Адреса модулей в системе согласно Протоколу обмена RS485 ----------------------------------------------------
//	
#define Adr_ZRU1			4											// Адрес ЗРУ1
#define Adr_ZRU2			5											// Адрес ЗРУ2
#define Adr_ZRU3			6											// Адрес ЗРУ3
#define Adr_ZRU4			7											// Адрес ЗРУ4
#define Adr_ZRU5			8											// Адрес ЗРУ5
#define Adr_ZRU6			9											// Адрес ЗРУ6
#define Adr_ZRU7			10										// Адрес ЗРУ7
#define Adr_ZRU8			11										// Адрес ЗРУ8
#define Adr_ZRU9			12										// Адрес ЗРУ9
#define Adr_ZRU10			13										// Адрес ЗРУ10
#define Adr_ZRU11			14										// Адрес ЗРУ11
#define Adr_ZRU12			15										// Адрес ЗРУ12

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define nPrior_ZRU		0x00									// Приоритет МУК ЗРУ
#define nPrior_ZRU2		0x02									// Приоритет МУК ЗРУ

//--------------------------- Адреса МУКов в обмене ЗРУ с БЭ ---------------------------------------------------------------
//.........................................................................................
#define nMUK1_BE			0x01									// Номер МУК1 БЭ
#define nMUK2_BE			0x02									// Номер МУК2 БЭ
#define nMUK3_BE			0x03									// Номер МУК3 БЭ

#define AdrMUK1_BE		0x01									// Адрес МУК1 БЭ
#define AdrMUK2_BE		0x03									// Адрес МУК2 БЭ
#define AdrMUK3_BE		0x05									// Адрес МУК3 БЭ

#define nMUKs_BE			0x00									// Адрес МУКов БЭ. Используется для отправки пакетов по CAN для всех МУКов БЭ

//.........................................................................................
#define nMUK1_ZRU			0x01									// Номер МУК1 ЗРУ
#define nMUK2_ZRU			0x02									// Номер МУК2 ЗРУ
#define nMUK3_ZRU			0x03									// Номер МУК3 ЗРУ

																						// CAN1
#define AdrMUK1_ZRU		0x07									// Адрес МУК1 ЗРУ
#define AdrMUK2_ZRU		0x09									// Адрес МУК2 ЗРУ
#define AdrMUK3_ZRU		0x0B									// Адрес МУК3 ЗРУ

#define nMUKs_ZRU			0x0										// Общий номер для всех ЗРУ. Для поля шапки фрейма, которое не используется.


//. . . . . . . . . . . . . . . . . . . . . .
#define nMUK_BCU			1											// Адрес МУК БЦУ




//**************************************************************************************************************************
//. . . . . . . . . . . . . . . . . . . . . .
#if (nMUK_ZRU==nMUK1_ZRU)
#define AdrMUK_ZRU		AdrMUK1_ZRU						// Аадрес МУК1 ЗРУ для CAN

#elif	(nMUK_ZRU==nMUK2_ZRU)
#define AdrMUK_ZRU		AdrMUK2_ZRU						// Аадрес МУК2 ЗРУ для CAN

#elif	(nMUK_ZRU==nMUK3_ZRU)
#define AdrMUK_ZRU		AdrMUK3_ZRU						// Аадрес МУК3 ЗРУ для CAN

#endif

//. . . . . . . . . . . . . . . . . . . . . .
#define AdrCAN1_ZRU		AdrMUK_ZRU						// Аадрес этого МУКа ЗРУ для CAN
#define AdrCAN2_ZRU		AdrMUK_ZRU + 1				// Аадрес этого МУКа ЗРУ для CAN

//**************************************************************************************************************************
#ifdef HW_ZRU01001
#define Adr_RS_ZRU		Adr_ZRU1							// Адрес этого ЗРУ для RS485

#elif defined(HW_ZRU01002)
#define Adr_RS_ZRU		Adr_ZRU2							// Адрес этого ЗРУ для RS485

#else
#error "Invali HW_* macro!"
#endif

//**************************************************************************************************************************
#define nReadADC				900									// Количество чтений АЦП для одного канала
																						
#define Transmit_wait		200									// Максимальное число ожидания отправки байта по RS485
#define nTimeOut				5										// 5 раз повторять команду ЗРУ -> БЭ

// .....................................................................................
#define nLastChannel		7										// Номер последнего канала чтения АЦП в ЗРУ
#define nParams					8										// Число измеряемых АЦП параметров ЗРУ

#define nAllAE					72									// Число АЭ АБ

// .....................................................................................
#define PORT_JTAG_Msk		7
#define PB10						0x400

//--------------------------- Работа CAN-----------------------------------------------------------------------------------
// Коды пакетов BE --> ZRU
#define CAN_PI_Datch		0x01								// Отправить телеметрию ДД, ДТ 											*** МУК БЭ 
#define CAN_PI_AK				0x02								// Отправить телеметрию аккумуляторов 							*** МУК БЭ 
#define CAN_Pasport_Put	0x03								// Отправить Паспортные данные ДД, ДТ								*** МУК БЭ 
#define CAN_ResCmd			0x04								// Результат выполнения команды 										*** МУК БЭ 
#define CAN_StatErr			0x0C								// Ошибки аппаратуры и отказов ДД, ДТ, АЭ						*** МУК БЭ 

// Коды пакетов ZRU --> BE
#define CAN_Vkl_RS			0x05								// Включить разрядные сопротивления в БЭ 						*** АРК, КПА !
#define CAN_Otkl_RS			0x06								// Отключить разрядные сопротивления в БЭ 					*** АРК, КПА !
#define CAN_MSG_Ok			0x0B								// Подтверждение в получении пакета телеметрии 			*** АРК, МУК БЭ, КПА !
#define CAN_NumBadAk		0x0D								// Номера отказавших АЭ АБ													***	АРК, КПА 
#define CAN_CopyCmd			0x0F								// Продублировать команду БЦУ												***	АРК, КПА !

// Коды пакетов KPA --> BE
#define CAN_Step_On			0x07								// Включить шаговый режим 													*** КПА !
#define CAN_Step_Off		0x08								// Отключить шаговый режим 													*** КПА !
#define CAN_U_AK				0x09								// Выдать напряжение пары АК 												*** КПА !
#define CAN_SetPeriod		0x0A								// Установить период выдачи телеметрии датчиков			***	АРК, КПА !
#define CAN_Pasport_Get	0x0E								// Запрос на паспортные данные ДД, ДТ								*** КПА 

//-------------------------------------------------------------------------------------------------------------------------
#define nMUKBE					3										// Число МУКов БЭ
#define nMUKZRU					3										// Число МУКов ЗРУ

#define okFrameDatch		0x3ff								// Маска фреймов в пакете телеметрии датчиков (получены все фреймы)
#define okFrameAK				0xfffff							// Маска фреймов в пакете телеметрии АК (получены все фреймы)

#define nFrameDatchCAN	10									// Число фреймов в пакете телеметрии датчиков
#define nFrameABCAN			20									// Число фреймов в пакете телеметрии АК

#define nfVdatch				35									// Число значений типа float в пакете телеметрии датчиков
#define niVdatch				7										// Число значений типа int в пакете телеметрии датчиков
#define nfV_AB					77									// Число значений типа float в пакете телеметрии АБ
#define niV_AB					3										// Число значений типа int в пакете телеметрии АБ

//--------------------------- Time определения ----------------------------------------------------------------------------
#define PauseReadyWrk		2										// Пауза 3 секунды для выхода МК на рабочий режим (не отвечаем на запросы по RS485)
#define vhCount12				43200								// 12 часов в сек (12*60*60)
#define vhCount5				18000								// 5 часов в сек (5*60*60)
#define vhCount2				7200								// 2 часа в сек (2*60*60)
#define vhCount1				3600								// 1 час в сек (1*60*60)
#define vmCount10				600									// 10 мин для паузы (10*60)
#define vmCount5				300									// 5 мин для паузы (5*60)
#define vmCount2				120									// 2 мин для паузы (2*60)
#define vmCount1				60									// 5 мин для паузы (5*60)
#define vsCount5				5										// 5 сек для расчёта C и W
#define vsCount10				10									// 15 сек для расчёта C и W
#define vsCount15				15									// 15 сек для расчёта C и W
#define vsCount20				20									// 20 сек для задержки повтора 3 раза алгоритма заряда
#define vsCount40				40									// 40 сек для задержки

#define dt1							1										// Дельта времени 5 сек при разряде
#define dt5							5										// Дельта времени 5 сек при разряде
#define dt20						20									// Дельта времени 20 сек при ТВЦ
#define timeRazr 				108000							// timeRazr = 108000сек (30*60*60)
#define timeZarKomp			172800							// timeRazr = 48час

// Для О Т Л А Д К И
//#define vsCount5				5										// 5 сек для расчёта C и W
#define vsCount2				2										// 2 сек
//#define vmCount5				2										// 2 сек
//#define vmCount1				2										// 2 сек
//#define vhCount2				2										// 2 сек
//**************************************************************************************************************************
//--------------------------- Шаги состояния процесса ТВЦ -----------------------------------------------------------------

#define bInit_TVC						0x01
#define	bOtklKomp						0x02
#define	bWaitOtklKomp				0x03
#define	bWaitRazryd					0x04
#define	bVkl_Tst_Zarayd			0x05
#define	bWaitVkl_Tst_Zarayd	0x06
#define	bVkl_Tst_Zarayd_On	0x07
#define	bWeitMKs						0x27
#define	bVkl_ZaprZarayd			0x08
#define	bWaitOtkl_Zarayd		0x09
#define	bVkl_Test_Razr			0x0A
#define	bWaitVkl_Test_Razr	0x0B
#define	bTst_T_Razryda			0x0C
#define	bWaitOtkl_Razrayd		0x0D	
#define	bTst_T_NVAB					0x0E
#define	bRepeatRazr					0x0F

#define	bWait_2							0x11
#define	bVkl_RS							0xA4	
#define	bWaitVkl_RS					0x12	
#define	bCountTimeRazr			0x13
#define	bOtkl_RS						0xA3
#define	bWaitOtkl_RS				0x14
#define	bVklKomp						0x15
#define	bWaitVklKomp				0x16
#define	bZarydComp					0x17
#define	bWaitOtklKomp2			0x18
#define	bVkl_Tst_Zarayd_On2	0x19
#define	bVklKomp2						0x1A
#define	bWaitVklKomp2				0x1B
#define	bZarydComp2					0x1C
#define	bWaitVkl_Test_Razr8	0xA2

#define	bTest_Vosst_et6_command		0xB0						// Вспомогательный шаг для восстановления 6-го этапа тестирования, выполнение команд
#define	bTest_Vosst_et6_checkI		0xB1						// Вспомогательный шаг для восстановления 6-го этапа тестирования, контроль тока после выполнения команд

#define	bTest_Vosst_et7_command		0xC0						// Вспомогательный шаг для восстановления 7-го этапа тестирования, выполнение команд
#define	bTest_Endtemp       0xB2   ////////////	               // Вспомогательный шаг для синхронизации выхода из 1-го этапа
#define	bVkl_Tst_Zarayd9		0x1E						// Включение заряда на 9-ом этапе ТВЦ
#define	bWaitVkl_Tst_Zar9		0x1F						// Включение заряда на 9-ом этапе ТВЦ
#define	bVkl_Tst_Zar_On9		0x20
#define	bWaitOtkl_Tst_Zar9	0x21						// Отключение заряда на 9-ом этапе ТВЦ
#define bInitEnd_Alg_TVC		0x4A
#define bEnd_Alg_TVC				0xA1						// Конец алгоритма ТВЦ

//--------------------------- Шаги состояния процессов заряд-разряд -------------------------------------------------------
enum ZarSteps {st_InitZarayd = 100, st_OtklKomp, st_WaitOtklKomp, st_Tst_P_NVAB, st_OtklZar_1, st_Vkl_Zarayd, st_Tst_T_NVAB_1, st_OtklZar_2, st_Tst_P_NVAB_2, st_Vkl_Zarayd_On, st_OtklZar_3, st_ViborUst};

//--------------------------- Шаги состояния процесса разряд --------------------------------------------------------------
enum RazSteps {st_InitRazryad = 200, st_Tst_I_Razryda, st_Tst_U_Razryda, st_Otkl_Razrayd, st_OtklRaz_Inspect, st_Tst_U_Razryda_end, st_UabCheck, st_WaitPause, st_ZRPCheck};

//--------------------------- Шаги состояния процесса подзаряд -------------------------------------------------------------
enum PodzarSteps {st_p_InitPodzar = 300, st_p_OtklPodzar_1, st_p_Pnuz_1, st_p_VklKomp, st_p_Tnuz_1, st_p_Otkl_ZaprZarayd, st_p_WaitOtkl_ZaprZar, st_p_Tnuz_2, st_p_Pnuz_2,
									st_p_OtklPodzar_2, st_Tnuz_minus2, st_p_WaitVklKomp, st_p_ZarydComp, st_p_Tvuz_minus2, st_p_WaitOtklKomp, st_p_EndPodzaryda};


//--------------------------- Режимы работы -------------------------------------------------------------------------------
#define Init_Run						1								// Инициализация всех процессов при старте или сбоях Uart (Ошибки принятия данных по UART)
#define START								2								// Начальный запуск и переход в режим опроса каналов АЦП, выход из состояния диагностики
#define Vkl_ZRU							3
#define Otkl_ZRU						4
#define TEST								7								// Тест
#define Otkl_TEST						8								// Отключить Тест
#define ADC_ERR							16							// Ошибка АЦП (появление флага OVERWRITE), повторный запуск опроса текущей пары каналов

#define RESTART							32							// Рестарт UART в случае ошибки
#define Otkl_RS							33							// Отключить разрядное сопротивления
#define CAN_not_working			34							// Нет связи по CAN (отказ)
#define Vkl_Podzarayd				35							// Включить подзаряд АБ
#define Otkl_Podzarayd			36							// Отключить подзаряд АБ
#define Work								37							// Штатная работа: ЗАРЯД, РАЗЯД одновременно работают
#define initTEST						38							// Тест
#define initPodzarayd				39

//**************************************************************************************************************************

//..........................Напряжение................................................................................................
#define Uminrazr						71.5						// Для алгоритма РАЗРЯД
#define UAKminrazr					0.1							// Для алгоритма РАЗРЯД
#define UakminPC						0.3							// При разряде на РС
#define deltaRazr						0.07						// Для алгоритма РАЗРЯД

//..........................Уставки ЗРУ.................................................................................................
#define	nUst					3											// Число уставок

#define	dU_u_def 			0.1										// Максимально допустимый разброс давления ΔДу
#define	dP_u_def 			6.0										// Максимально допустимый разброс напряжений ΔUр 
#define	T_max_def 		32										// Предел доп темп АБ на заряде
#define	Pvuz1_def 		38										// Верхний уровень заряженности НВАБ
#define	Pvuz2_def 		42										// Верхний уровень заряженности НВАБ
#define	Tvuz1_def 		30										// Предельно допустимая температура НВАБ при уровне заряженности Рвуз1
#define	Tvuz2_def 		30										// Предельно допустимая температура НВАБ при уровне заряженности Рвуз8
#define	Pvir3_def 		45										// Верхний уровень заряженности для восстановления НВАБ
#define	Tvir3_def 		25										// Предельно допустимая температура АБi при уровне заряженности РВЫР3, ТВЫР3
#define	Pnuz_def 			42										// Верхний уровень заряженности НВАБ
#define	PvuzCK_def 		62										// РВУЗ СК		11	Предел ур заряж АБ на СК при заряде током не более (1,5±0,5) А		ufix8,z = 0.06 кгс/см2, x0 = 50 кгс/см2
#define	TnuzAB_def 		25										// ТНУЗАБ			12	Предел доп темп АБ в начале подзаряда на СК		ufix8,z = 0.08 0C, x0 = 5 0C
#define	TvuzCK_def 		25										// ТВУЗ СК		13	Предел доп темп АБ  при заряде током не более (1,5±0,5) А до ур заряж РВУЗСК		ufix8,z = 0.04 0C, x0 = 15 0C
#define	Pn_def 				38										// Давление нижнего уровня заряженности НВАБ при ТВЦ,  (40-50)
#define	Pv_def 				42										// Давление верхнего уровня заряженности НВАБ при ТВЦ, (46-54)
#define	Tn3_def 			40										// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности Р<0,8Pн,  (42-45)
#define	Tn2_def 			30										// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности 0,8Pн<Р<Pн, (28-32)
#define	Tn1_def 			20										// Предельно допустимая температура НВАБ в ТВЦ при уровне заряженности Pн<Р<Pв, (18-22)
#define	Tk_def 				24										// Tv Предельно допустимая температура НВАБ при включении компенсационного подзаряда (22-28)
#define	tVir_def 			48*60*60							// Время выравнивания заряженности НВАБ при ТВЦ, 48*60 = 2880 мин = 172800 сек
#define	Pvz_def 			54										// Среднее давление НВА при ТВЦ после подзаряда(48 часов), кгс/см2
#define	Traz_def 			55										// Предельно допустимая температура в ТВЦ НВАБ при разряде (42-45)
#define	P0_def 				3											// Среднее давление НВА при прекращении  тестового разряда НВАБ (НАБ (76+-1В)
#define	tRazr_def 		30*60*60							// Время разряда на разрядное сопротивление, 30 час = 1800 мин = 108000 сек (10-50)
#define	kWtst_def 		2900									// Коэфф. для расчёта текущего значения энергоёмкости
#define	tP_TVC_def 		40										// от 10,0 до 70,0 мин	40 мин	Время принудительного разряда в ТВЦ,ТВЦ	аналоговый, 

//#define	Pvuz_ck_def 	62										// Верхний уровень заряженности НВАБ на СК при заряде током не более 2А
//#define	Tnuz_ab_def 	25										// Предельно допустимая температура НВАБ в начале подзаряда на СК
//#define	Tvuz_ck_def 	25										// Предельно допустимая температура НВАБ

// ............................ Токи в алгоритмах ......................................
#define	fNul								0.8							// Ток погрешности (пульсации) нуля

//Токи алгоритма Заряда/Разряда
#define	aIkomp							3								// Ток компенсационного заряда
#define	aIkomp_1A						1								// Ток, встречается в алгоритме подзаряда
#define	aIporog							2								// Предназначена для контролея наличия/отсутсвия тока 
#define	aIzard							18							// Ток заряда
#define	aIogrn							27							// Ток максимально допустимый
#define	aIrazr_ogrn					30							// Ток максимально допустимый

#define	tWaitCmd						65							// Ожидание выполнения команды

//**************************************************************************************************************************
/*
Мажоритированные данные, двухбитные, формируемые в каждом из трех МК ЗРУ
РС							 										1 / 0	В АБi включены РС.
ЗРУ							 										1 / 0	Состояние силового канала i-го ЗРУ.
Подзаряд				 										1 / 0	Включен режим «Подзаряд на СК» i-го ЗРУ.

ТВЦ							 										1 / 0	Включен режим «ТВЦ» i-го ЗРУ.
Основной режим	 										1 / 0	Включен режим «Основной» i-го ЗРУ.
Заряд 				 	 										1 / 0	Разрешен заряд  i-го ЗРУ.
Разряд				 	 										1 / 0	Разрешен разряд i-го ЗРУ.
Не подключились РС									1 / 0	Не включились РС в АБi после выдачи из ЗРУi команды на их включение.
Не отключились РС										1 / 0	Не отключились РС в АБi после выдачи из ЗРУi команды на их отключение.
Не отключился разряд АБ							1 / 0	Не отключился разряд АБi после формирования ЗРУi УВ на запрет разряда.
Не отключился заряд АБ							1 / 0	Не отключился заряд АБi после формирования ЗРУi УВ на запрет заряда.

Не включился КОМП подзаряд					1 / 0	Не включился компенсационный подзаряд АБi согласно алгоритму.
Не отключился КОМП подзаряд					1 / 0	Не отключился компенсационный подзаряд АБi согласно алгоритму.
Не включился разряд АБ в ТВЦ				1 / 0	Не включился разряд АБi согласно алгоритму.
Не включился заряд АБ в ТВЦ					1 / 0	Не включился заряд АБi согласно алгоритму.
резерв
Не ограничивается ток разряда			 	1 / 0	Не ограничивается ток разряда АБi (более 30 А).
Не ограничивается ток заряда			 	1 / 0	Не ограничивается ток заряда АБi (более 25 А).
Превышена допустимая температура АБ	1 / 0	Превышение допустимой температуры АБi.

*/

// ............................. Статус и сообщения ЗРУ .....................................................
//unsigned char stat1[3];
// байт 1 ========================================================================
// номер уставки						1..3						//6,7 бит

#define bIlast						 	0x08						//																		1 / 0	В 
#define bPC								 	0x04						//РС																	1 / 0	В АБi включены РС.
#define pwrZRU							0x02						//ЗРУ																	1 / 0	Состояние силового канала i-го ЗРУ.
#define bPodzaryad				 	0x01						//Подзаряд														1 / 0	Включен режим «Подзаряд на СК» i-го ЗРУ.
#define bTest								0x80						//ТВЦ																	1 / 0	Включен режим «ТВЦ» i-го ЗРУ.
#define bMain	 							0x40						//Основной режим											1 / 0	Включен режим «Основной» i-го ЗРУ.
#define bZaryad 					 	0x20						//Заряд																1 / 0	Разрешен заряд  i-го ЗРУ.
#define bRazryad					 	0x10						//Разряд															1 / 0	Разрешен разряд i-го ЗРУ.

//unsigned char stat2[3];
// байт 2 ========================================================================
#define errNoVklRS					0x08						//Не подключились РС						ТВЦ												1 / 0	Не включились РС в АБi после выдачи из ЗРУi команды на их включение.
#define errNoOtklRS					0x04						//Не отключились РС							ТВЦ												1 / 0	Не отключились РС в АБi после выдачи из ЗРУi команды на их отключение.
#define errNoOtklRazr				0x02						//Не отключился разряд АБ				ТВЦ, Разряд, РазрядПр			1 / 0	Не отключился разряд АБi после формирования ЗРУi УВ на запрет разряда.
#define errNoOtklZar				0x01						//Не отключился заряд АБ				ТВЦ, Заряд, ЗарядПр				1 / 0	Не отключился заряд АБi после формирования ЗРУi УВ на запрет заряда.
#define errNoVklCompZar			0x80						//Не включился КОМП подзаряд		ТВЦ												1 / 0	Не включился компенсационный подзаряд АБi согласно алгоритму.
#define errNoOtklCompZar		0x40						//Не отключился КОМП подзаряд		ТВЦ, Подзаряд на СК				1 / 0	Не отключился компенсационный подзаряд АБi согласно алгоритму.
#define errNoVklRazr				0x20						//Не включился разряд АБ в ТВЦ	ТВЦ												1 / 0	Не включился разряд АБi согласно алгоритму.
#define errNoVklZar					0x10						//Не включился заряд АБ в ТВЦ		ТВЦ												1 / 0	Не включился заряд АБi согласно алгоритму.

//unsigned char stat3[3];
// байт 3 ========================================================================
#define bready							0x80						//выход из условия в 4 этапе
#define bZaprZar						0x40						//Запрет Заряда
#define bZaprRazr						0x20						//Запрет Разряда
#define vklZRU							0x10						//Включить ЗРУ команда
#define RestData						0x08						//Восстановить данные
#define errNoOgrTokRazr			0x04						//Не ограничивается ток разряда			 	1 / 0	Не ограничивается ток разряда АБi (более 30 А).
#define errNoOgrTokZar			0x02						//Не ограничивается ток заряда			 	1 / 0	Не ограничивается ток заряда АБi (более 25 А).
#define errPrevDopustT			0x01						//Превышена допустимая температура АБ	1 / 0	Превышение допустимой температуры АБi.

//unsigned char stat4[3];
// байт 4 ========================================================================
//весь байт необходим для уникальных синхронизаций в ТВЦ
#define bready8							0x80						//
#define bready7							0x40						//прекращение начального заряда
#define bready6							0x20						//выход из условия 4-ого разряда
#define bready5							0x10						//выход из условия 3-ого разряда
#define bready4							0x08						//выход из условия 2-ого разряда
#define bready3							0x04						//выход из условия 1-ого разряда
#define bready2							0x02						//выход из условия в 7 этапе
#define bready1							0x01						//выход из условия в 6 этапе

#define tRestData						60							// 1 мин время ожидания прихода повторного флага на восстановление данных
#define ctimeGetcmd					2								// Время прихода (в сек) команды БЦУ в ЗРУ для 3-х МУКов, чтобы собрался мажоритар

#define errNoDataTelem			0x8000					// "Нет данных телеметрии АБ"

// .............................Ошибки рабочих параметров, принятых от БЭ данных
#define errNoDataP					0x10000					// "Нет значения давления"
#define errNoDatadP					0x20000					// "Нет значения максимальное отклонение ДД"
#define errNoDataT					0x40000					// "Нет значения среднее значение ДТ "
#define errNoDataUsrAk			0x80000					// "Нет значения UsrAk"
#define errNoDatadUak			 0x100000					// "Нет значения dU"
#define errNoDataUAB			 0x200000					// "Нет значения UAB"
#define errNoDataUmin_ak	 0x400000					// "Нет значения Umin_ak"

#define errNoDataUsr				0x80000					// "Нет значения Usr"
#define errNoDatadU					0x100000				// "Нет значения dU"
#define errNoDatadUAB				0x200000				// "Нет значения UAB"

// .............................Ошибка аппаратуры модуля БЭ
/* ResultCAN:
	0x10000		1	- Опорное напряжение ПСД ниже допустимого значения. Uop < 2.5*0.95
	0x20000		2	- Общий ПСД вне допустимого значения (>0.1)
	0x40000		3	- Число вышедших из строя датчиков температуры превысило лимит
	0x80000		4	- Число вышедших из строя датчиков давления превысило лимит
	0x100000	5	- Опорное напряжение МИБ вне допуска, Uop < 2*0.95
	0x200000	6	- В МИБ напряжение на общем вне допуска (>0.1)
	0x400000	7	- Число вышедших из строя АЭ превысило лимит
	0x800000		- 
	*/
#define waitOEon		5													// Ожидание переключения OE после приёма на передачу
#define waitOEoff		75												// Ожидание завершения передачи

//-------------------------------------------------------------------------------------------------------------------------
union uBytesFloat16 {
	unsigned char b[2];
	__fp16 Fdata;
};

union uBytes {
	unsigned char b[4];
	uint32_t data32;
};

union uBytes64 {
	unsigned char b[8];
	uint64_t data64;
};

union uBytesFloat {
	unsigned char b[4];
	float data32;
};

//--------------------------------------------------------------------------------------------------------------------------
float abs_f (float vf);
void Var_init(void);												// Инициализация переменных
void EnableIRQ_ADC_CAN_UART (void);

//--------------------------------------------------------------------------------------------------------------------------
void pVkl_Shim_ZRU (int bWait);					void pOtkl_Shim_ZRU (int bWait);				// 1 
void pVkl_KOMP (void);									void pOtkl_KOMP (void);									// 2
void pVkl_Zapr_Zarayd (void);						void pOtkl_Zapr_Zarayd (void);					// 3
void pVkl_Zapr_Razrayd (void);					void pOtkl_Zapr_Razrayd (void);					// 4
void pVkl_Test_Zarayd (void);						void pOtkl_Test_Zarayd (void);					// 5
void pVkl_Test_Razrayd (void);					void pOtkl_Test_Razrayd (void);					// 6
void pVkl_RS (int bWait);								void pOtkl_RS (int bWait);							// 7
//void pVkl_AB_ZRU (void);								void pOtkl_AB_ZRU (void);								// 0
//void pVkl_SES_ZRU (int bWait);					void pOtkl_SES_ZRU (int bWait);					// 8

//--------------------------------------------------------------------------------------------------------------------------
void pNotCan (void);																					// При отказе 2-х CAN от БЭ
void CAN_SendCmd(unsigned char adr_MUK_ZRU, unsigned char dlc, unsigned char cmd);
void CAN_SendStatusZRU(void);
void CAN_SendConf_1(unsigned char confcmd);
void CAN_SendConf_2(unsigned char confcmd);
void CAN_SendBadNumAk(unsigned char adr_MUK_Z, unsigned char n_MUK_BE, unsigned char cmd);

void CAN2_TstMSG(int cod, unsigned char lenData, volatile unsigned char * volatile pnt);
//void CAN_SendTstData(unsigned char n_MUK_Z, unsigned char n_MUK_BE, unsigned char cmd);
//void CAN1_MakeMSG_pack(int cod, unsigned char lenData, volatile unsigned char * volatile pnt);
//void CAN2_MakeMSG_pack(int cod, unsigned char lenData, volatile unsigned char * volatile pnt);

//--------------------------------------------------------------------------------------------------------------------------
void UART_SendByte(unsigned char byte);			// Функция отправки байта по UART

//--------------------------------------------------------------------------------------------------------------------------
void ADC_Start (int ch);
void ADC_GO (int nadc);											// Зауск процесса преобразования
//void ReadADC (void);												// Основная процедура программы
void PutParamADC (void);										// С 23.01.20
void MakePack1(void);												// С 14.07.20 Заполнение пакета телем. ЗРУ данными	
void MakePack6(void);

#endif /* __WORK_H */

/* END OF FILE Work.h */

