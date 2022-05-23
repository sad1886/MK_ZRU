/*------------------------ CAN -----------------------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion -------------------------------------------------------------------------------*/
#ifndef CAN_H
#define CAN_H

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "core_cm3.h"
#include "system_MDR32F9x.h"
#include <stdlib.h>	

typedef struct
{
  uint32_t BUF_ID; 							// ID сообщения буфера [28..18] SID, [17..0] EID
  uint32_t BUF_DLC;							// DLC сообщения буфера .12 IDE, .11 SSR, .10 R0, .9 R1, .8 RTR, [3..0] DLC
  uint32_t BUF_DATAL; 					// Данные [3]...[0] сообщения буфера 
  uint32_t BUF_DATAH; 					// Данные [7]...[4] сообщения буфера 
} CAN_T_BUF;

#define CAN_SID	18							// смещение SID в CAN_BUF[x].BUF_ID

// разряды в CAN_BUF[x].BUF_DLC
#define	CAN_IDE	12
#define	CAN_SSR	11
#define	CAN_R0	10
#define	CAN_R1	 9
#define	CAN_RTR	 8

typedef struct
{
  uint32_t BUF_MASK; 						// Маска для приема сообщения CAN 
  uint32_t BUF_FILTER; 					// Фильтр для приема сообщени CAN 
} CAN_T_MASK;

typedef struct
{
  uint32_t CAN_CONTROL;					// Регистр управление контроллером CAN
  uint32_t CAN_STATUS;					// Регистр состояния контроллера CAN 
  uint32_t CAN_BITTMNG;					// Регистр задания скорости работы
  uint32_t CAN_HOLE0;						// Дыра0
  uint32_t CAN_INT_EN;					// Регистр разрешения прерываний контроллера
  uint32_t CAN_HOLE1;						// Дыра1
  uint32_t CAN_HOLE2;						// Дыра2
  uint32_t CAN_OVER;						// Регистр границы счетчика ошибок
  uint32_t CAN_RXID;						// Регистр принятого ID сообщения
  uint32_t CAN_RXDLC;						// Регистр принятого DLC сообщения
  uint32_t CAN_RXDATAL; 				// Регистр принятых данных
  uint32_t CAN_RXDATAH;					// Регистр принятых данных
  uint32_t CAN_TXID;						// Регистр передаваемого ID сообщения
  uint32_t CAN_TXDLC;						// Регистр передаваемого DLC сообщения
  uint32_t CAN_DATAL; 					// Регистр передаваемых данных
  uint32_t CAN_DATAH; 					// Регистр передаваемых данных
  uint32_t CAN_BUF_CON[32];			// Регистр управления буферами 1..32
  uint32_t CAN_INT_RX;					// Флаги разрешения прерываний от приемных буферов
  uint32_t CAN_RX;							// Флаги RX_FULL от приемных буферов
  uint32_t CAN_INT_TX;					// Флаги разрешения прерываний от передающих буферов
  uint32_t CAN_TX; 							// Флаги ~TX_REQ от передающих буферов
  uint32_t CAN_HOLE[76];
  CAN_T_BUF	CAN_BUF[32];				// ID, DLC, DATA[8] буферов 1..32
  uint32_t CAN_HOLE3[64];				// дыра
  CAN_T_MASK CAN_MASK[32];			// маска и фильтр буферов 1..32
} CAN_TypeDef;

#define CAN1_BASE	((uint32_t)0x40000000)
#define CAN2_BASE	((uint32_t)0x40008000)

#define CAN1	((CAN_TypeDef *) CAN1_BASE)
#define CAN2	((CAN_TypeDef *) CAN2_BASE)

// разряды в регистре CAN_CONTROL
#define	CAN_EN	0								// = 1 разрешить CAN
#define	CAN_ROM	1								// = 1 только на прием
#define	CAN_STM	2								// = 1 self-test mode
#define	CAN_SAP	3								// = 1/0 подтверждать прием собств/только чужих пакетов
#define	CAN_ROP	4								// = 1/0 прием собств/только чужих пакетов

// разряды в регистре CAN_STATUS
#define	CAN_RX_READY	0
#define	CAN_TX_READY	1

// разряды в регистре CAN_BUF_CON[xx]
#define	CAN_OVER_WR	7						// = 1 была перезапись принятого сообщения
#define	CAN_RX_FULL	6						// = 1 сообщение принято
#define	CAN_TX_REQ	5						// = 0 нет запроса на отправку или отправлено, = 1 уст. запрос на отправку
#define	CAN_PRIOR0	4						// = 1 повысить приоритет отправки
#define	CAN_RTR_EN	3						// = 1 отвечать при приеме RTR
#define	CAN_OVER_EN	2						// = 1 разрешить перезапись принятых
#define	CAN_RX_ON		1						// = 0/1 режим передача/прием
#define	CAN_BUF_EN	0						// = 1 включить буфер

// разряды вкл тактирования CAN в регистре RSTCLK->PER_CLOCK
#define	CAN1_PER_CLK	0
#define	CAN2_PER_CLK	1

// разряды в регистре RSTCLK->CAN_CLOCK
#define	CAN1_CLC_EN		24
#define	CAN2_CLC_EN		25

// разряды в регистре CAN_INT_EN
#define	ERR_OVER_INT	4					// = 1 разрешение прерывания по превышению TEC или REC допустимого значения в ERROR_MAX
#define	ERR_INT_EN		3					// = 1 разрешение прерывания по возникновению ошибки
#define	TX_INT_EN			2					// = 1 разрешение прерывания по возможности передачи
#define	RX_INT_EN			1					// = 1 разрешение прерывания по приему сообщений
#define	GLB_INT_EN		0					// = 1 общее разрешение прерывания блока CAN

// разряды разрешения прерываний CAN в SETENA
#define	CAN2_IRQ_EN		1
#define	CAN1_IRQ_EN		0

#define nbuf_RX				0					// РќРѕРјРµСЂ Р±СѓС„РµСЂР° РїСЂРёС‘РјР° CAN
#define nbuf_TX				29				// РќРѕРјРµСЂ Р±СѓС„РµСЂР° РїРµСЂРµРґР°С‡Рё CAN
#define maxOVER	 	 		255				// РњР°РєСЃРёРјР°Р»СЊРЅРѕРµ С‡РёСЃР»Рѕ РѕС€РёР±РѕРє

void CAN1_DeInit(void);
void CAN2_DeInit(void);
void CAN1_Init(void);
void CAN2_Init(void);

#endif /* __CAN_H */
