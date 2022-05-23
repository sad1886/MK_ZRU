/*------------------------ CAN ----------------------------------------*/

#include "MDR32F9x.h"

#define _CAN_
#include "Can.h"

//==========================================================================
//void CAN1_DeInit(void)
////==========================================================================
//{  uint32_t i;
//	
//	MDR_CAN1->STATUS = 0;		  MDR_CAN1->BITTMNG = 0;	  MDR_CAN1->INT_EN 	= 0;	  MDR_CAN1->OVER 	 = 0;
//	MDR_CAN1->INT_RX = 0; 		MDR_CAN1->INT_TX 	= 0;	  MDR_CAN1->CONTROL = 0;
//	
//  for (i = 0; i < 32; i++)  {    MDR_CAN1->BUF_CON[i] = 0;  }
//}

////==========================================================================
//void CAN2_DeInit(void)
////==========================================================================
//{  uint32_t i;
//	
//	for(i = 0; i < 10000; i++) { MDR_CAN2->STATUS = 0; }

//	for (i = 0; i < 31; i++)  {    MDR_CAN2->BUF_CON[i] = 0;  }
//	// Очистим буфера
//	for(i = 0; i < 31; i++)	{
//		MDR_CAN2->CAN_BUF[i].ID 				=0;
//		MDR_CAN2->CAN_BUF[i].DATAL 			=0;		MDR_CAN2->CAN_BUF[i].DATAH 				 =0;
//		MDR_CAN2->CAN_BUF_FILTER[i].MASK=0;		MDR_CAN2->CAN_BUF_FILTER[i].FILTER =0;
//	}
//	
//  MDR_CAN2->STATUS = 0;		  MDR_CAN2->BITTMNG = 0;	  MDR_CAN2->INT_EN  = 0;	  MDR_CAN2->OVER = 0;
//  MDR_CAN2->INT_RX = 0;		  MDR_CAN2->INT_TX 	= 0;		MDR_CAN2->CONTROL = 0;
//	
//	MDR_RST_CLK->CAN_CLOCK &= ~(1 << CAN2_CLC_EN);
//	MDR_RST_CLK->PER_CLOCK &= ~(1 << CAN2_PER_CLK);
//}

//==========================================================================
void CAN1_Init(void)
//==========================================================================
{
	MDR_RST_CLK->PER_CLOCK |= (1 << CAN1_PER_CLK);																				// 
	MDR_RST_CLK->CAN_CLOCK |= (1 << CAN1_CLC_EN);																					// 
	//         			       	SB				 SJW				 SEG2				SEG1				PSEG			BRP
	//MDR_CAN1->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 9;		// CAN1 speed 1000 kbit/s	от 01.07.16
	//MDR_CAN1->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 19;		// CAN1 speed 500 kbit/s	от 01.07.16
	//................ 80Мг ..........................................................
	//MDR_CAN1->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 39;		// CAN1 speed 250 kbit/s	от 28.12.17

	//................ 24Мг ..........................................................
	MDR_CAN1->BITTMNG	 =	(0 << 27) | (0 << 25) | (3 << 22) | (7 << 19) | (2 << 16) | 5;	// CAN1 speed 250 kbit/s	РѕС‚ 24.10.18 24Mg
	
	MDR_CAN1->OVER = maxOVER;																															// 
	MDR_CAN1->BUF_CON[nbuf_RX] = (1<< CAN_BUF_EN) | (1<< CAN_RX_ON);											// 
	MDR_CAN1->INT_EN = (1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
	MDR_CAN1->INT_RX = 1<<nbuf_RX;																												// 
	MDR_CAN1->CAN_BUF_FILTER[nbuf_RX].MASK = 0;																						// 0x7FF << 18;
	MDR_CAN1->CAN_BUF_FILTER[nbuf_RX].FILTER = 0;																					// ((nbuf_RX+1) << 18);
	MDR_CAN1->CONTROL = (1 << CAN_EN);																										// 
}

//==========================================================================
void CAN2_Init(void)
//==========================================================================
{	
	MDR_RST_CLK->PER_CLOCK |= (1 << CAN2_PER_CLK);																				// 
	MDR_RST_CLK->CAN_CLOCK |= (1 << CAN2_CLC_EN);																					// 
	//         			       	SB				 SJW				 SEG2				SEG1				PSEG			BRP
	//MDR_CAN2->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 9;		// CAN2 speed 1000 kbit/s	от 01.07.16
	//MDR_CAN2->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 19;		// CAN2 speed 500 kbit/s	от 01.07.16
	//................ 80Мг ..........................................................
	//MDR_CAN2->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 39;		// CAN2 speed 250 kbit/s	от 28.12.17

	//................ 24Мг ..........................................................
	MDR_CAN2->BITTMNG	 =	(0 << 27) | (0 << 25) | (3 << 22) | (7 << 19) | (2 << 16) | 5;	// CAN2 speed 250 kbit/s	РѕС‚ 24.10.18 24Mg

	MDR_CAN2->OVER = maxOVER;																															// 
	MDR_CAN2->BUF_CON[nbuf_RX] = (1<< CAN_BUF_EN) | (1<< CAN_RX_ON);											// 
	MDR_CAN2->INT_EN = (1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
	MDR_CAN2->INT_RX = 1<<nbuf_RX;																												// 
	MDR_CAN2->CAN_BUF_FILTER[nbuf_RX].MASK = 0;																						// 0x7FF << 18;
	MDR_CAN2->CAN_BUF_FILTER[nbuf_RX].FILTER = 0;																					// ((nbuf_RX+1) << 18);
	MDR_CAN2->CONTROL = (1 << CAN_EN);																										// 
}

