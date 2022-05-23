#ifndef __CONST_H
#define __CONST_H

#include "config.h"

//период ШИМ
#ifdef	RT
	#define TPWM  3200	//25 кГц при тактовой 80МГц
#endif
#ifdef	ZRU
	#define TPWM  1600	//25 кГц при тактовой 80МГц
#endif


#define PWM_1_3   MDR_TIMER3->CCR1	//PWM1 и PWM3, основной СМ HIN и HIN-R
#define PWM_2_4   MDR_TIMER2->CCR1	//PWM2 и PWM4, основной СМ LIN и LIN-R
#define PWM_1_3_R MDR_TIMER2->CCR2	//PWM1-R и PWM3-R, дополнительный СМ HIN и HIN-R
#define PWM_2_4_R MDR_TIMER2->CCR3	//PWM2-R и PWM4-R, дополнительный СМ LIN и LIN-R


// //коэф. дробные до наброса
// #define KP_RT    5.0/10.0
// #define KI_RT    8.0/1000.0
//коэф. дробные
#define KP_RT    30.0/10.0
#define KI_RT    200.0/1000.0
// #define KP_RT    0.0/10.0
// #define KI_RT    1.0/10000.0

//вадим
// #define KP_ZRU_U   1.0		//разряд
// #define KI_ZRU_U   57.0/1000.0	//разряд
// #define KP_ZRU_I   176.0/100.0		//разряд
// #define KI_ZRU_I   18.0/100.0	//разряд
////вадим	мои
//#define KP_ZRU_U   1.0		//разряд
//#define KI_ZRU_U   57.0/1000.0	//разряд
//#define KP_ZRU_I   70.0/100.0		//разряд
//#define KI_ZRU_I   18.0/100.0	//разряд
// //мои 3 кГц хороший разряд
// #define KP_ZRU_U   8.0/10.0		//разряд
// #define KI_ZRU_U   150.0/1000.0	//разряд
// #define KP_ZRU_I   70.0/100.0		//разряд
// #define KI_ZRU_I   18.0/100.0	//разряд
//мои 3 кГц 
#define KP_ZRU_U   6.0/10.0		//разряд
#define KI_ZRU_U   30.0/1000.0	//разряд
#define KP_ZRU_I   210.0/100.0		//разряд
#define KI_ZRU_I   6.0/100.0	//разряд
// //мои 2 кГц 
// #define KP_ZRU_U   6.0/10.0		//разряд
// #define KI_ZRU_U   50.0/1000.0	//разряд
// #define KP_ZRU_I   70.0/100.0		//разряд
// #define KI_ZRU_I   18.0/100.0	//разряд
//3 кГц до переезда
// #define KP_ZRU_U   6.0/10.0		//разряд
// #define KI_ZRU_U   30.0/1000.0	//разряд
// #define KP_ZRU_I   70.0/100.0		//разряд
// #define KI_ZRU_I   18.0/100.0	//разряд

#define KP_SN_U   4		//разряд
#define KI_SN_U   400000	//разряд
#define KP_SN_I   4		//разряд
#define KI_SN_I   200000	//разряд	 работало с плохим током
////#define KP_SN_U   7		//разряд
////#define KI_SN_U   200000	//разряд
////#define KP_SN_I   4		//разряд
////#define KI_SN_I   35000	//разр


#define ZONA 40	//в кодах АЦП
#define UOP_ZRU 1887	//1.5В при опорном АЦП 3.3В - соответствует 28.0
#define UOP_SN	(1887-1*ZONA)
#define UOP_RT1 (1887-2*ZONA)	//1.5В - соответствует 28.0
#define UOP_RT2 (1887-3*ZONA)	//1.5В - соответствует 28.0
#define UOP_RT3 (1887-4*ZONA)	//1.5В - соответствует 28.0
#define UMAX 2390	//1.9В - напряжение, выше которого мы не должны подниматься(с учетом инверсии)
#define UMIN 1683	//1.3В - напряжение, выше которого мы не должны подниматься(с учетом инверсии)

//1861 - старое значение зоны ЗРУ


#define iMax 500000	//в коде АЦП
#define iMin -500000	//в коде АЦП

//Токи
#define I_ZRU_RAZ 557	//30 А
//#define I_ZRU_RAZ 272	//15 А
//#define I_ZRU_RAZ 363	//20 А
#define I_ZRU_ZAR I_ZRU_20	//20 А
#define I_ZRU_20 371	//20 А
#define I_ZRU_10 186	//10 А
#define I_ZRU_5 93	//5 А
#define I_ZRU_2_5 46	//2.5 А
#define I_ZRU_1 19	//1 А

#define I_KZ_ZRU 1510	//28.8 А
//#define I_0 2330 //ток равен нулю на отладочной	  1837.5 1837.4
#define I_0 1793 //ток равен нулю на новой плате	  1460,0 при опоре 3310,0

//#define U_UTOCH ((2970*4095)/3300) //если бы питание было 3.3 ровно, то именно такой код мы должны были бы получать, измеряя прецизионное напряжение 3.0, доведенное до АЦП через делитель 0.99 (2970)
#define U_UTOCH 3685.5 //если бы питание было 3.3 ровно, то именно такой код мы должны были бы получать, измеряя прецизионное напряжение 3.0, доведенное до АЦП через делитель 0.99 (2970)

#endif /* __CONST_H */
