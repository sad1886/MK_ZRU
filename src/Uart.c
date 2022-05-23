
#include <stdlib.h>	
#include "MDR32F9x.h"
#include "init.h"																									// Файл с описанием процедур инициализации и аппаратных настроек
#include "Can.h"
#include "Work.h"
#include "Uart.h"

extern int iMUK_ZRU;

//--------------------------- RS485 переменные ----------------------------------------------------------------------------
//UART1 . . . . . . . . . . . . . . . . . . . . . . . . . . .
unsigned char pack1[Npack_Cmd];																			// Буфер приёма пакетов
volatile int ind_pack1;																							// Индекс приёмного буфера
volatile int lngPack1;																							// Длина принятого пакета

volatile int BatchSize1;																						// Размер передаваемого пакета в байтах
volatile int ind_mas_trans1;																				// Индекс передаваемого пакета
unsigned char * p_ParRs1;
volatile unsigned short checksumCalc1, checksumIn1;

//UART2 . . . . . . . . . . . . . . . . . . . . . . . . . . .
unsigned char pack2[Npack_Cmd];																			// Буфер приёма пакетов
volatile int ind_pack2;																							// Индекс приёмного буфера
volatile int lngPack2;																							// Длина принятого пакета

volatile int BatchSize2;																						// Размер пакета в байтах
volatile int ind_mas_trans2;
unsigned char * p_ParRs2;
volatile unsigned short checksumCalc2, checksumIn2;

//.......................................................................................................................
unsigned char bReqBCU[2];																						// Флаг: поступил запрос (команда) от БЦУ
volatile unsigned char cmd_ZRU;																			// cmd_curr, 

unsigned short checksumCalc, checksumIn;

unsigned char * p_InPack;
unsigned char * p_ParRs;

float U_ADC[nParams];																								// Массив измеренных напряжений АЦП параметров ЗРУ
//																										1			2			1		  2			4				  6		 7		номера каналов АЦП
float Vals_ZRU[nParams] = {0,0,0,0,0,0,0,0};		// {Iзар1 Iзар2 Iраз1 Iраз2  UАБ1 	0	 Tмп1 Tмп2}	Массив реальных значений измеренных параметров ЗРУ
float Vals_ZRUold[nParams];											// {Iзар1 Iзар2 Iраз1 Iраз2  UАБ1 	0	 Tмп1 Tмп2}	Массив реальных значений измеренных параметров ЗРУ

//float aI_razr, aI_razrOld, aI_zar;

//float tVals_ZRU[11] = {21, 22, 60, 135, 90, 1, 4000, 4500, 50, 8, 60};						// Массив тестовых значений измеренных параметров ЗРУ

// *******************************************************************************************************************************************************************
// ........ пакет 1 ..............................................................................................
// Ток Зар2, Ток Зар1, Ток Разр2, Ток Разр1, U_АБ2, U_АБ1, ТМП2, ТМП1,		1 -8
// Текущ заряд НВАБ, Энергоёмк, Ёмкость, Средн_P_разр, Средн_P_подзар,		9 -13
// Сост ЗРУ14, Сост ЗРУ16, Сост ЗРУ16.																		14-15

unsigned char PackRs1[lngPackRs1] = {START_BYTE, Adr_RS_ZRU, nMUK_BCU, 1,														// Шапка пакета
																		0, 0, 0, 0, 0, 0, 0, 0,																					// Данные, ind - 4...14	Ток Зар2, Ток Зар1, Ток Разр2, Ток Разр1, U_АБ2, U_АБ1, ТМП2, ТМП1,
																		0, 0,	0, 0, 0,																									// Данные, ind - 4...14	Текущ заряд НВАБ, Энергоёмк, Ёмкость, Средн_P_разр, Средн_P_подзар,		9 -13
																		0, 0};																													// Контрольная сумма
float z_p [nParams] 	= {0.12, 0.12,0.14, 0.14, 0.48, 17.7, 0.36, 0.36};														// z – цена (вес) младшего разряда;
float x0_p[nParams] 	= {0,    0,    0,    0,    0,  	-500, 	 0,    0};															// x0 – сдвиг нуля

int indsData_p[nParams] = {5,4,7,6,8,9,11,10};																											// Номера индексов в блоке данных пакета телеметрии протокола RS485
	
// ........ пакет 2 ..............................................................................................
unsigned char PackRs2[lngPackRs2] = {START_BYTE, Adr_RS_ZRU, nMUK_BCU, 2, 0, 0};										// Шапка пакета Контрольная сумма

// ........ пакет 3 ..............................................................................................
unsigned char PackRs3[lngPackRs3] = {START_BYTE, Adr_RS_ZRU, nMUK_BCU, 3,														// Шапка пакета
																		0, 0, 0, 0, 0, 0, 0, 0, 0,
																		0, 0};																													// Контрольная сумма

float z_p3[lngPackRs3-7] 	= {0.44, 0.28, 0.013, 0.013, 0.013, 0.013, 0.28, 0.28};										// z – цена (вес) младшего разряда;
float x0_p3[lngPackRs3-7] = { 10,   0,   -0.7,  -0.7,  -0.7,  	0,   	0,    -10};										// x0 – сдвиг нуля

// ........ пакет 4 ..............................................................................................
unsigned char PackRs4[lngPackRs4] = {START_BYTE, Adr_RS_ZRU, nMUK_BCU, 4,														// Шапка пакета
																		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 		// Данные, ind - 4...
																		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 		// 
																		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 		// 
																		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 		// 
																		0, 0, 0, 0	};																									// Контрольная сумма
float z_p4[3]  = {0.013, 0.28, 0.28};																																// z – цена (вес) младшего разряда;
float x0_p4[3] = {-0.7, 	 0,   -10}; 																															// x0 – сдвиг нуля

// ........ пакет 5 ..............................................................................................
unsigned char PackRs5[lngPackRs5] = {START_BYTE, Adr_RS_ZRU, nMUK_BCU, 5, 0, 0};										// Шапка пакета Контрольная сумма

float z_p5[lngUstavki_Curr-6] 	= {0.005, 0.034, 0.2,   0.12,  0.12,  0.12, 0.08, 0.08,  0.06,			// z – цена (вес) младшего разряда;
																	 0.06,  0.08,  0.06,  0.18,  0.22,  0.04, 0.03, 0.05, 0.024, 17, 	
																	 0.12,	0.08,  0.08,  12,		 15.7,  0.31};
float x0_p5[lngUstavki_Curr-6]  = {0,  		3.5 ,  0, 		30,    30,	  0,     40,   10,		35,				// x0 – сдвиг нуля
																	 50,    5,   	 15,    5,     5, 	  35, 	 25,   18,   	22,		1,
																	 38,   	40,    0,     1,  	 0, 	  1};

// ........ пакет 6 ..............................................................................................
unsigned char PackRs6[lngPackRs6] = {START_BYTE, Adr_RS_ZRU, nMUK_BCU, 6,														// Шапка пакета
																		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,																		// Данные, ind - 4...
																		0, 0, 0, 0,	0, 0, 0, 0, 0, 0,
																		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,																		//
																		0, 0};																													// Контрольная сумма

// ........ пакет 7 ..............................................................................................
unsigned char PackRs7[lngPackRs7] = {START_BYTE, Adr_RS_ZRU, nMUK_BCU, 7,														// Шапка пакета Контрольная сумма
																		 0, 0, 0, 0, 0, 0};		
float z_p7[3]  = {15.7, 0.2, 17};																																// z – цена (вес) младшего разряда;
float x0_p7[3] = {0, 	  0,   0}; 																																		// x0 – сдвиг нуля

// ........ пакет 8 ..............................................................................................
unsigned char PackRs8[lngPackRs8] = {START_BYTE, Adr_RS_ZRU, nMUK_BCU, 8,	0, 0};										// Шапка пакета Контрольная сумма
																		 		
// ........ пакет 9 ..............................................................................................
unsigned char PackRs9[lngPackRs9] = {START_BYTE, Adr_RS_ZRU, nMUK_BCU, 9,														// Шапка пакета Контрольная сумма
																		 0, 0, 0, 0};		
float z_p9[2]  = {0.12, 0.08};																																			// z – цена (вес) младшего разряда;
float x0_p9[2] = {38,		0}; 																																				// x0 – сдвиг нуля

// ........ пакет 10 ..............................................................................................
unsigned char PackRs10[lngPackRs10] = {START_BYTE, Adr_RS_ZRU, nMUK_BCU, 255,	0, 0};								// Шапка пакета Контрольная сумма
																		 		

// ...............................................................................................................
/*
  Name  : CRC-16 CCITT
  Poly  : 0x1021    x^16 + x^12 + x^5 + 1
  Init  : 0xFFFF
  Revert: false
  XorOut: 0x0000
  Check : 0x29B1 ("123456789")
  MaxLen: 4095 байт (32767 бит) - обнаружение одинарных, двойных, тройных и всех нечетных ошибок
*/
const unsigned short Crc16Table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Вычисление контрольной суммы
unsigned short Crc16(unsigned char * pcBlock, unsigned short len)
{
  unsigned short crc = 0xFFFF;

  while (len--)
    crc = (crc << 8) ^ Crc16Table[(crc >> 8) ^ *pcBlock++];

  return crc;
}

//end of file
