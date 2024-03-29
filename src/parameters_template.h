/**********************************************************************
*																																			*
*		Файл параметров для МУК ЗРУ зав. № ________		*
*		Является шаблоном с универсальными начальными коэффициентами 																																*
***********************************************************************/

#ifndef PARAMETERS_ZRU01001_H
#define PARAMETERS_ZRU01001_H

// .............................................................................................................................
// 										//ДТ1					//ДТ2
float cUsm[3][2] =  {1.41586590, 1.42268384, 		//МК1		//(В) Измеренное напряжение средней точкой (смещение) для МУКов					
										 1.41502085, 1.42268384,		//МК2
										 1.40575933, 1.41517997			//МК3		
										};
										
// ............................ Датчики токов, напряжения и температуры ЗРУ.....................
						// 										//ДТ1		//ДТ2
volatile float KoefZar [3][2] = {39.312, 39.452,	//МК1	 												// Коэффициенты масштабирования для Тока Заряда;
																 39.312, 39.452,	//МК2	
																 39.312, 39.452		//МК3
																};
volatile float KoefRazr[3][2] = {37.893, 37.966,	//МК1 												// Коэффициенты масштабирования для Тока Разряда;
																 37.893, 37.966,	//МК2	
																 37.893, 37.966		//МК3
																};

//																		//МК1					//МК2					//МК3
volatile float Koef_k_Uab_zru[3] = {81.63110168, 81.85477775, 82.39049012}; //коэффициент Uаб_зру
volatile float Koef_b_Uab_zru[3] = {0.424771822, 0.199492301, 0.021287108}; //смещение Uаб_зру 

volatile float Koef_k_dtemp1[3] = {357.4708858, 340.2298951, 335.481519}; //коэффициент Темп1
volatile float Koef_b_dtemp1[3] = {-270.199492301, -270.0808839, -266.0672971}; //смещение Темп1 

volatile float Koef_k_dtemp2[3] = {357.4708858, 340.2298951, 335.481519}; //коэффициент Темп2
volatile float Koef_b_dtemp2[3] = {-270.199492301, -270.0808839, -266.0672971}; //смещение Темп2 

//volatile float KoefIzarT[3][2] 	= {0, 		0, 		 0,
//																	 0, 		0, 		 0}; 																// Коэффициенты масштабирования для Тока Заряда;
//volatile float KoefIrazT[3][2] 	= {0, 		0, 		 0,
//																	 0.289, 0.289, 0.289}; 														// Коэффициенты масштабирования для Тока Разряда;

//volatile float KoefIzarABT[3] = {0.07, 0.07, 0.07}; 																// Коэффициенты масштабирования для Тока Заряда;
//volatile float KoefIrazABT[3] = {0.08, 0.08, 0.08}; 																// Коэффициенты масштабирования для Тока Разряда;


//**************************************************************************************************
			float	AUcc[3]	= {3.054, 3.042, 3.056};		// (В) Опорное напряжение АЦП МУК1, МУК2, МУК3
const float	Uref[3]	= {2.984, 2.967, 2.979};		// (В) Прецизионное напряжение АЦП МУК1, МУК2, МУК3 на PD0

																 
#endif
// end of parameters.h
