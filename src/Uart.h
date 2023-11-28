/* Define to prevent recursive inclusion -------------------------------------------------------------------------------*/
#ifndef UART_H
#define UART_H

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>

// Команды, поступающие из БЦУ по интерфейсу RS485:
/*
«ЗПД»																«Запрос полных данных» – данных АК (таблица 5) 
«ЗУР»																«Запрос условий работы» – АБ (таблица 6)
«ЗТУ»																«Запрос о состоянии текущих уставок ЗРУ» (таблица 10)
«ОЗВД»															«Отключение запроса на восстановление данных»  ЗВД=0
*/

//-------------------------------------------------------------------------------------------------------------
// Пакеты, поступающие из БЦУ по интерфейсу RS485:
#define gStat_ZRU						0x1														// телеметрия ЗРУ
#define gCmd_for_ZRU				0x2														// команда для ЗРУ
#define gStat_AB_short			0x3														// Сост_АБ_Кратк_БЭ - краткая телеметрия БЭ
#define gStat_AB_Full				0x4														// Сост_АБ_Полн_БЭ – полная телеметрия БЭ
#define gUstavki_Curr				0x5														// Уставки_Текущ - управление БЭ
#define gUstavki_Tst				0x6														// контроль параметров (уставок) алгоритмов ЗРУ
#define gSaveData_to_BCU		0x7														// запоминаемые для восстановления данные в БВС 
#define gRestData_from_BCU	0x8														// данные для восстановления из БВС 
#define gParamZRU						0x9														// частные параметры ЗРУ 
#define gTstLine						0xFF													// проверка связи

// Длины пакетов в байтах, поступающих из БЦУ по интерфейсу RS485:
#define lngStat_ZRU					6															// 
#define lngCmd_ZRU					7															// 
#define lngStat_AB_short		6															// 
#define lngStat_AB_Full			6															// 
#define lngUstavki_Curr			36														// 
#define lngUstavki_Tst			6															// 
#define lngSaveData_BCU			6															// 
#define lngRestData_BCU			10														// 
#define lngParamZRU					6															// 
#define lngTstLine					6															// 

// Параметр команды gCmd_for_ZRU - код команды для ЗРУ
#define gVkl_ZRU						1														// ВКЛ ЗРУ 			выполнять подключение ЗРУ-120 к НВАБ и силовым шинам БСШ-В
#define gOtkl_ZRU						2														// ОТКЛ ЗРУ 		выполнять отключение ЗРУ-120 от НВАБ и силовых шин БСШ-В
#define gVkl_Test						3														// ВКЛ Тест
#define gOtkl_Test					4														// ОТКЛ Тест
#define gOtkl_RS						5														// ОТКЛ РС 			отключать разрядные сопротивления в БЭ
#define gVkl_Podzarayd			6														// ВКЛ Подзаряд
#define gOtkl_Podzarayd			7														// ОТКЛ Подзаряд
#define OZVD								8														// «Отключение запроса на восстановление данных» – снятие запроса восст данных (установить ЗВД=0) 

//-------------------------------------------------------------------------------------------------------------
// Размеры пакетов, отправляемые ЗРУ по RS485 
#define lngPackRs1					19														// телеметрия ЗРУ
#define lngPackRs2					6															// Ответ на пакет 2
#define lngPackRs3					29														// краткая телеметрия БЭ
#define lngPackRs4					80														// полная телеметрия БЭ
#define lngPackRs5					6															// Ответ на пакет 5
#define lngPackRs6					36														// контроль уставок ЗРУ 
#define lngPackRs7					10														// запоминаемые для восстановления данные в БВС 
#define lngPackRs8					6															// Ответ на пакет 8
#define lngPackRs9					8															// частные параметры ЗРУ 
#define lngPackRs10					6															// Ответ на пакет 10

//-------------------------------------------------------------------------------------------------------------
// Другие определения
#define i14tel							lngPackRs1-5									// индекс 14-го байта тедеметрии ЗРУ
#define i15tel							lngPackRs1-4									// индекс 15-го байта тедеметрии ЗРУ
#define i16tel							lngPackRs1-3									// индекс 16-го байта тедеметрии ЗРУ

#define START_BYTE					0xAA													// Первый байт пакета
	
#define Npackage						20														// Приёмный кольцевой буфер данных, принятых по UART
#define Npack_Cmd						60														// Размерность пакета команды БЦУ (по самому длинному) 40*12

// Макросы для работы с UART	по RS-485
#define RS_TRANSMIT1		MDR_PORTA->RXTX |=  0x20;					// Режим передатчика Uart1 - OE=1
#define RS_RECEIVE1 		MDR_PORTA->RXTX &= ~0x20;					// Режим приёмника  Uart1 - OE=0

#define RS_TRANSMIT2		MDR_PORTF->RXTX |=  0x04;					// Режим передатчика Uart2 - OE=1
#define RS_RECEIVE2 		MDR_PORTF->RXTX &= ~0x04;					// Режим приёмника  Uart2 - OE=0

#define TRANSMIT_WAIT1	while (!byte_sent1) {}						// Ожидание отправки байта
#define TRANSMIT_WAIT2	while (!byte_sent2) {}						// Ожидание отправки байта

//-------------------------------------------------------------------------------------------------------------
// Функции	
void SendMsgBCU(void);																		// Отправка сообщения в БЦУ по RS485 
void GetCmd(void);																				// Поиск команды, полученной по по RS485 от БЦУ
void GetCmd_2(int nUart, unsigned char * package, unsigned char * pack);
//void RunCmd (unsigned char * p_InPack, int tip);																			// Отработка команды, полученной по по RS485 от БЦУ

unsigned short Crc16(unsigned char * pcBlock, unsigned short len);
void TransmRS_485_(unsigned char * pcBlock, unsigned short len);

#endif /* __UART_H */
