

#define	DMA_PER_CLK	5																			// разряд вкл тактирования в регистре RSTCLK->PER_CLOCK

#define CFG_master_enable  0    													// Маска разрешает работу контроллера.

#define Uart1_TX	0

	//Параметры структуры. 
#define dst_src       (0<<30)                 //Источник - байт
#define dst_size      (0<<28)                 //Принимаем по 16 бит. (Приемник и передатчик должны иметь одинаковые размерности).
#define src_inc       (0<<26)                 //Источник смещается на 8 бит после каждой передачи. 
#define src_size      (0<<24)                 //Отправляем по 16 бит.
#define dst_prot_ctrl                         //Здесь настраивается различного рода защита (буферизация, привилегированный режим, )
#define R_power       (0<<14)                 //Арбитраж (приостановка передачи до внешнего сигнала, разрешающего ее продолжение) после каждой передачи. 
#define n_minus_1     (99<<4)                 //100  передач DMA. 
#define next_useburst (0<<3)                  //Так и не удалось понять, что это...
#define cycle_ctrl    (1<<0)                  //Обычный режим.
//Настраиваем структуру.
#define ST_DMA_DAC_STRYKT dst_src|src_inc|src_size|dst_size|R_power|n_minus_1|next_useburst|cycle_ctrl


struct DAC_ST
{
	uint32_t Destination_end_pointer;                                     //Указатель конца данных приемника.
	uint32_t Source_end_pointer;                                          //Указатель конца данных источника
	uint32_t channel_cfg;                                                 //Конфигурация канала.
	uint32_t NULL;                                                        //Пустая ячейка. 
}

void InitDMA (void)
{
	MDR_RST_CLK->PER_CLOCK |= (1<<DMA_PER_CLK);							// Подача тактирования
	
	DMA->CFG = (1<<CFG_master_enable);                 			// Разрешаем работу DMA
	
	DMA->CHNL_ENABLE_SET = (1<<Uart1_TX);                 	// Разрешаем работу канала DMA
	
	DMA->CHNL_REQ_MASK_CLR = (1<<Uart1_TX);         		  	// Разрешаем установку запросов на выполнение циклов DMA, по dma_sreq[] и dma_req[]
	
	DMA->CHNL_PRIORITY_SET = 1<<Uart1_TX;             	    // Высокий приоритет. 
	
	
}
