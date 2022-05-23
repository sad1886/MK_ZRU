/**
  ******************************************************************************
  * @file    parameters.h
  * @author  ЗАО Орбита Карташов Ю.Д.
  * @version V1.0.0
  * @date    03.07.2014
  ******************************************************************************
  */
#ifndef PARAMETERS_H
#define	PARAMETERS_H
/*
	########################## Настраиваемые параметры при отладке модуля ##########################
*/
#include "Work.h"


#ifdef HW_ZRU01001
#include "parameters_ZRU01001.h"

#elif defined(HW_ZRU01002)
#include "parameters_ZRU01002.h"

#else
#error "Invali HW_* macro!"
#endif

//#ifdef HW_E3001001
//#include "parameters_E3001001.h"
//#elif defined(HW_E3001002)
//#include "parameters_E3001002.h"
//#elif defined(HW_T3001001)
//#include "parameters_T3001001.h"
//#elif defined(HW_ZRU01002)
//#include "parameters_ZRU01002.h"
//#elif defined(HW_ZRU01001)
//#include "parameters_ZRU01001.h"
//#else
//#error "Invali HW_* macro!"
//#endif

#endif
