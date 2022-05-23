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
  uint32_t BUF_ID; 							// ID ��������� ������ [28..18] SID, [17..0] EID
  uint32_t BUF_DLC;							// DLC ��������� ������ .12 IDE, .11 SSR, .10 R0, .9 R1, .8 RTR, [3..0] DLC
  uint32_t BUF_DATAL; 					// ������ [3]...[0] ��������� ������ 
  uint32_t BUF_DATAH; 					// ������ [7]...[4] ��������� ������ 
} CAN_T_BUF;

#define CAN_SID	18							// �������� SID � CAN_BUF[x].BUF_ID

// ������� � CAN_BUF[x].BUF_DLC
#define	CAN_IDE	12
#define	CAN_SSR	11
#define	CAN_R0	10
#define	CAN_R1	 9
#define	CAN_RTR	 8

typedef struct
{
  uint32_t BUF_MASK; 						// ����� ��� ������ ��������� CAN 
  uint32_t BUF_FILTER; 					// ������ ��� ������ �������� CAN 
} CAN_T_MASK;

typedef struct
{
  uint32_t CAN_CONTROL;					// ������� ���������� ������������ CAN
  uint32_t CAN_STATUS;					// ������� ��������� ����������� CAN 
  uint32_t CAN_BITTMNG;					// ������� ������� �������� ������
  uint32_t CAN_HOLE0;						// ����0
  uint32_t CAN_INT_EN;					// ������� ���������� ���������� �����������
  uint32_t CAN_HOLE1;						// ����1
  uint32_t CAN_HOLE2;						// ����2
  uint32_t CAN_OVER;						// ������� ������� �������� ������
  uint32_t CAN_RXID;						// ������� ��������� ID ���������
  uint32_t CAN_RXDLC;						// ������� ��������� DLC ���������
  uint32_t CAN_RXDATAL; 				// ������� �������� ������
  uint32_t CAN_RXDATAH;					// ������� �������� ������
  uint32_t CAN_TXID;						// ������� ������������� ID ���������
  uint32_t CAN_TXDLC;						// ������� ������������� DLC ���������
  uint32_t CAN_DATAL; 					// ������� ������������ ������
  uint32_t CAN_DATAH; 					// ������� ������������ ������
  uint32_t CAN_BUF_CON[32];			// ������� ���������� �������� 1..32
  uint32_t CAN_INT_RX;					// ����� ���������� ���������� �� �������� �������
  uint32_t CAN_RX;							// ����� RX_FULL �� �������� �������
  uint32_t CAN_INT_TX;					// ����� ���������� ���������� �� ���������� �������
  uint32_t CAN_TX; 							// ����� ~TX_REQ �� ���������� �������
  uint32_t CAN_HOLE[76];
  CAN_T_BUF	CAN_BUF[32];				// ID, DLC, DATA[8] ������� 1..32
  uint32_t CAN_HOLE3[64];				// ����
  CAN_T_MASK CAN_MASK[32];			// ����� � ������ ������� 1..32
} CAN_TypeDef;

#define CAN1_BASE	((uint32_t)0x40000000)
#define CAN2_BASE	((uint32_t)0x40008000)

#define CAN1	((CAN_TypeDef *) CAN1_BASE)
#define CAN2	((CAN_TypeDef *) CAN2_BASE)

// ������� � �������� CAN_CONTROL
#define	CAN_EN	0								// = 1 ��������� CAN
#define	CAN_ROM	1								// = 1 ������ �� �����
#define	CAN_STM	2								// = 1 self-test mode
#define	CAN_SAP	3								// = 1/0 ������������ ����� ������/������ ����� �������
#define	CAN_ROP	4								// = 1/0 ����� ������/������ ����� �������

// ������� � �������� CAN_STATUS
#define	CAN_RX_READY	0
#define	CAN_TX_READY	1

// ������� � �������� CAN_BUF_CON[xx]
#define	CAN_OVER_WR	7						// = 1 ���� ���������� ��������� ���������
#define	CAN_RX_FULL	6						// = 1 ��������� �������
#define	CAN_TX_REQ	5						// = 0 ��� ������� �� �������� ��� ����������, = 1 ���. ������ �� ��������
#define	CAN_PRIOR0	4						// = 1 �������� ��������� ��������
#define	CAN_RTR_EN	3						// = 1 �������� ��� ������ RTR
#define	CAN_OVER_EN	2						// = 1 ��������� ���������� ��������
#define	CAN_RX_ON		1						// = 0/1 ����� ��������/�����
#define	CAN_BUF_EN	0						// = 1 �������� �����

// ������� ��� ������������ CAN � �������� RSTCLK->PER_CLOCK
#define	CAN1_PER_CLK	0
#define	CAN2_PER_CLK	1

// ������� � �������� RSTCLK->CAN_CLOCK
#define	CAN1_CLC_EN		24
#define	CAN2_CLC_EN		25

// ������� � �������� CAN_INT_EN
#define	ERR_OVER_INT	4					// = 1 ���������� ���������� �� ���������� TEC ��� REC ����������� �������� � ERROR_MAX
#define	ERR_INT_EN		3					// = 1 ���������� ���������� �� ������������� ������
#define	TX_INT_EN			2					// = 1 ���������� ���������� �� ����������� ��������
#define	RX_INT_EN			1					// = 1 ���������� ���������� �� ������ ���������
#define	GLB_INT_EN		0					// = 1 ����� ���������� ���������� ����� CAN

// ������� ���������� ���������� CAN � SETENA
#define	CAN2_IRQ_EN		1
#define	CAN1_IRQ_EN		0

#define nbuf_RX				0					// Номер буфера приёма CAN
#define nbuf_TX				29				// Номер буфера передачи CAN
#define maxOVER	 	 		255				// Максимальное число ошибок

void CAN1_DeInit(void);
void CAN2_DeInit(void);
void CAN1_Init(void);
void CAN2_Init(void);

#endif /* __CAN_H */
