

#define	DMA_PER_CLK	5																			// ������ ��� ������������ � �������� RSTCLK->PER_CLOCK

#define CFG_master_enable  0    													// ����� ��������� ������ �����������.

#define Uart1_TX	0

	//��������� ���������. 
#define dst_src       (0<<30)                 //�������� - ����
#define dst_size      (0<<28)                 //��������� �� 16 ���. (�������� � ���������� ������ ����� ���������� �����������).
#define src_inc       (0<<26)                 //�������� ��������� �� 8 ��� ����� ������ ��������. 
#define src_size      (0<<24)                 //���������� �� 16 ���.
#define dst_prot_ctrl                         //����� ������������� ���������� ���� ������ (�����������, ����������������� �����, )
#define R_power       (0<<14)                 //�������� (������������ �������� �� �������� �������, ������������ �� �����������) ����� ������ ��������. 
#define n_minus_1     (99<<4)                 //100  ������� DMA. 
#define next_useburst (0<<3)                  //��� � �� ������� ������, ��� ���...
#define cycle_ctrl    (1<<0)                  //������� �����.
//����������� ���������.
#define ST_DMA_DAC_STRYKT dst_src|src_inc|src_size|dst_size|R_power|n_minus_1|next_useburst|cycle_ctrl


struct DAC_ST
{
	uint32_t Destination_end_pointer;                                     //��������� ����� ������ ���������.
	uint32_t Source_end_pointer;                                          //��������� ����� ������ ���������
	uint32_t channel_cfg;                                                 //������������ ������.
	uint32_t NULL;                                                        //������ ������. 
}

void InitDMA (void)
{
	MDR_RST_CLK->PER_CLOCK |= (1<<DMA_PER_CLK);							// ������ ������������
	
	DMA->CFG = (1<<CFG_master_enable);                 			// ��������� ������ DMA
	
	DMA->CHNL_ENABLE_SET = (1<<Uart1_TX);                 	// ��������� ������ ������ DMA
	
	DMA->CHNL_REQ_MASK_CLR = (1<<Uart1_TX);         		  	// ��������� ��������� �������� �� ���������� ������ DMA, �� dma_sreq[] � dma_req[]
	
	DMA->CHNL_PRIORITY_SET = 1<<Uart1_TX;             	    // ������� ���������. 
	
	
}
