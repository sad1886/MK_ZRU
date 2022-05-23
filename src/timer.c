#include "MDR32F9x.h"
#include "const.h"

void Timer_Init(){
	
	//TIMER3: ШИМ	
	MDR_RST_CLK->PER_CLOCK |= 1<<16;
	MDR_RST_CLK->TIM_CLOCK |= 1 << 26;	//подали тактовую частоту на Таймер3
	
  MDR_TIMER3->CNTRL     = 0x00000000;
  MDR_TIMER3->CNT       = 0x00000000;
	MDR_TIMER3->PSG       = 0x00000000;
	MDR_TIMER3->ARR       = TPWM; 

	MDR_TIMER3->CH1_CNTRL = 0x00000C00;//канал работает в режиме "ШИМ", переключение REF при CNT=CCR
	MDR_TIMER3->CH1_CNTRL1= 0x00000009;//сигнал REF идет на выход канал 1
	MDR_TIMER3->CCR1= 0 ;
         
  MDR_TIMER3->STATUS = 0x00000000;//сбросили флаги всех прерываний
  MDR_TIMER3->IE |= 1<<1; //1 - разрешили прерывание по CNT = ARR
  NVIC_EnableIRQ(Timer3_IRQn); // сигналы прерывания от Таймера №3
	
// 	MDR_TIMER3->CNTRL    |= 0x00000001;//разрешение работы таймера 
	//TIMER2: ШИМ
	MDR_RST_CLK->PER_CLOCK |= 1<<15;	
	MDR_RST_CLK->TIM_CLOCK |= 1 << 25;	//подали тактовую частоту на Таймер3
	
  MDR_TIMER2->CNTRL     = 0x00000000;
  MDR_TIMER2->CNT       = 0x00000000;
	MDR_TIMER2->PSG       = 0x00000000;
	MDR_TIMER2->ARR       = TPWM; 

	MDR_TIMER2->CH1_CNTRL = 0x00000C00;//канал работает в режиме "ШИМ", переключение REF при CNT=CCR
	MDR_TIMER2->CH1_CNTRL1= 0x00000009;//сигнал REF идет на выход канал 1
	MDR_TIMER2->CCR1= 0 ;
	
	MDR_TIMER2->CH2_CNTRL = 0x00000C00;//канал работает в режиме "ШИМ", переключение REF при CNT=CCR
	MDR_TIMER2->CH2_CNTRL1= 0x00000009;//сигнал REF идет на выход канал 1
	MDR_TIMER2->CCR2= 0 ;
	
	MDR_TIMER2->CH3_CNTRL = 0x00000C00;//канал работает в режиме "ШИМ", переключение REF при CNT=CCR
	MDR_TIMER2->CH3_CNTRL1= 0x00000009;//сигнал REF идет на выход канал 1
	MDR_TIMER2->CCR3= 0 ;	
         
//   MDR_TIMER2->STATUS = 0x00000000;//сбросили флаги всех прерываний
//   MDR_TIMER2->IE |= 1<<1; //1 - разрешили прерывание по CNT = ARR
//   NVIC_EnableIRQ(Timer2_IRQn); // сигналы прерывания от Таймера №3
	
	MDR_TIMER2->CNTRL    |= 0x00000001;//разрешение работы таймера 
	MDR_TIMER3->CNTRL    |= 0x00000001;//разрешение работы таймера 
}

void __irq Timer3_IRQHandler (){
	uint32_t temp;	
	
	MDR_TIMER3->STATUS = 0x00; //сбрасываем флаг
	
//мигаем PB5 и PB7
 	temp = MDR_PORTB->RXTX;
 	temp ^= REG_EN;	//инвертируем пятый и седьмой бит - ЗРУ1
	temp &= ~0x1F;	//чтобы не сбивалась отладка по JTAG-A, обнуляем первые пять бит
	MDR_PORTB->RXTX = temp;
}

//--- System Timer initialization ---
void SysTickInit(uint32_t delay) //задержка в микросекундах
{
	SysTick->LOAD = delay*80;	// (HCLK = 80MHz)
	SysTick->CTRL = 1<<2|1<<1|1;	//Enabel SysTick
	while((SysTick->CTRL&0x10000)!=0x10000);
	SysTick->CTRL = 0;	//выключили системный таймер
}

void SysTick_Handler(void)
{

}
