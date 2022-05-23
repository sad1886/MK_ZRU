/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_config.h"

#define ASSERT_INFO_FILE_ID FILEID__MDR32F9X_PORT_C

void PORT_DeInit(MDR_PORT_TypeDef* PORTx)
{
  /* Check the parameters */
  assert_param(IS_PORT_ALL_PERIPH(PORTx));

  PORTx->ANALOG = 0;
  PORTx->PULL = 0;
  PORTx->OE = 0;
  PORTx->RXTX = 0;
  PORTx->FUNC = 0;
  PORTx->PD = 0;
  PORTx->PWR = 0;
  PORTx->GFEN = 0;
}

void PORT_Init(MDR_PORT_TypeDef* PORTx, const PORT_InitTypeDef* PORT_InitStruct)
{
  uint32_t tmpreg_OE;
  uint32_t tmpreg_FUNC;
  uint32_t tmpreg_ANALOG;
  uint32_t tmpreg_PULL;
  uint32_t tmpreg_PD;
  uint32_t tmpreg_PWR;
  uint32_t tmpreg_GFEN;
  uint32_t portpin, pos, mask_s, mask_l, mask_d;

  /* Check the parameters */
  assert_param(IS_PORT_ALL_PERIPH(PORTx));
  assert_param(IS_PORT_PIN(PORT_InitStruct->PORT_Pin));
  assert_param(IS_PORT_OE(PORT_InitStruct->PORT_OE));
  assert_param(IS_PORT_PULL_UP(PORT_InitStruct->PORT_PULL_UP));
  assert_param(IS_PORT_PULL_DOWN(PORT_InitStruct->PORT_PULL_DOWN));
  assert_param(IS_PORT_PD_SHM(PORT_InitStruct->PORT_PD_SHM));
  assert_param(IS_PORT_PD(PORT_InitStruct->PORT_PD));
  assert_param(IS_PORT_GFEN(PORT_InitStruct->PORT_GFEN));
  assert_param(IS_PORT_FUNC(PORT_InitStruct->PORT_FUNC));
  assert_param(IS_PORT_SPEED(PORT_InitStruct->PORT_SPEED));
  assert_param(IS_PORT_MODE(PORT_InitStruct->PORT_MODE));

  /* Get current PORT register values */
  tmpreg_OE     = PORTx->OE;
  tmpreg_FUNC   = PORTx->FUNC;
  tmpreg_ANALOG = PORTx->ANALOG;
  tmpreg_PULL   = PORTx->PULL;
  tmpreg_PD     = PORTx->PD;
  tmpreg_PWR    = PORTx->PWR;
  tmpreg_GFEN   = PORTx->GFEN;

  /* Form new values */
  pos = 0;
  mask_s = 0x0001;
  mask_l = 0x00000003;
  mask_d = 0x00010001;
  for (portpin = PORT_InitStruct->PORT_Pin; portpin; portpin >>= 1)
  {
    if (portpin & 0x1)
    {
      tmpreg_OE     = (tmpreg_OE     & ~mask_s) | (PORT_InitStruct->PORT_OE        <<  pos);
      tmpreg_FUNC   = (tmpreg_FUNC   & ~mask_l) | (PORT_InitStruct->PORT_FUNC      << (pos*2));
      tmpreg_ANALOG = (tmpreg_ANALOG & ~mask_s) | (PORT_InitStruct->PORT_MODE      <<  pos);
      tmpreg_PULL   = (tmpreg_PULL   & ~mask_d) | (PORT_InitStruct->PORT_PULL_UP   << (pos + 16))
                                                | (PORT_InitStruct->PORT_PULL_DOWN <<  pos);
      tmpreg_PD     = (tmpreg_PD     & ~mask_d) | (PORT_InitStruct->PORT_PD_SHM    << (pos + 16))
                                                | (PORT_InitStruct->PORT_PD        <<  pos);
      tmpreg_PWR    = (tmpreg_PWR    & ~mask_l) | (PORT_InitStruct->PORT_SPEED     << (pos*2));
      tmpreg_GFEN   = (tmpreg_GFEN   & ~mask_s) | (PORT_InitStruct->PORT_GFEN      <<  pos);
    }
    mask_s <<= 1;
    mask_l <<= 2;
    mask_d <<= 1;
    pos++;
  }

  /* Configure PORT registers with new values */
  PORTx->OE     = tmpreg_OE & (~JTAG_PINS(PORTx));
  PORTx->FUNC   = tmpreg_FUNC & (~JTAG_PINS(PORTx));
  PORTx->ANALOG = tmpreg_ANALOG & (~JTAG_PINS(PORTx));
  PORTx->PULL   = tmpreg_PULL & (~JTAG_PINS(PORTx));
  PORTx->PD     = tmpreg_PD & (~JTAG_PINS(PORTx));
  PORTx->PWR    = tmpreg_PWR & (~JTAG_PINS(PORTx));
  PORTx->GFEN   = tmpreg_GFEN & (~JTAG_PINS(PORTx));
}

void PORT_StructInit(PORT_InitTypeDef* PORT_InitStruct)
{
  /* Reset PORT initialization structure parameters values */
  PORT_InitStruct->PORT_Pin        = PORT_Pin_All;
  PORT_InitStruct->PORT_OE         = PORT_OE_IN;
  PORT_InitStruct->PORT_PULL_UP    = PORT_PULL_UP_OFF;
  PORT_InitStruct->PORT_PULL_DOWN  = PORT_PULL_DOWN_OFF;
  PORT_InitStruct->PORT_PD_SHM     = PORT_PD_SHM_OFF;
  PORT_InitStruct->PORT_PD         = PORT_PD_DRIVER;
  PORT_InitStruct->PORT_GFEN       = PORT_GFEN_OFF;
  PORT_InitStruct->PORT_FUNC       = PORT_FUNC_PORT;
  PORT_InitStruct->PORT_SPEED      = PORT_OUTPUT_OFF;
  PORT_InitStruct->PORT_MODE       = PORT_MODE_ANALOG;
}

uint8_t PORT_ReadInputDataBit(MDR_PORT_TypeDef* PORTx, uint32_t PORT_Pin)
{
  uint8_t bitstatus;

  /* Check the parameters */
  assert_param(IS_PORT_ALL_PERIPH(PORTx));
  assert_param(IS_GET_PORT_PIN(PORT_Pin));

  if ((PORTx->RXTX & PORT_Pin) != Bit_RESET)
  {
    bitstatus = (uint8_t)Bit_SET;
  }
  else
  {
    bitstatus = (uint8_t)Bit_RESET;
  }
  return bitstatus;
}

uint32_t PORT_ReadInputData(MDR_PORT_TypeDef* PORTx)
{
  /* Check the parameters */
  assert_param(IS_PORT_ALL_PERIPH(PORTx));

  return (PORTx->RXTX);
}

void PORT_SetBits(MDR_PORT_TypeDef* PORTx, uint32_t PORT_Pin)
{
  /* Check the parameters */
  assert_param(IS_PORT_ALL_PERIPH(PORTx));
  assert_param(IS_PORT_PIN(PORT_Pin));
  assert_param(IS_NOT_JTAG_PIN(PORTx, PORT_Pin));

  PORTx->RXTX = PORT_Pin | (PORTx->RXTX & (~JTAG_PINS(PORTx)));
}

void PORT_ResetBits(MDR_PORT_TypeDef* PORTx, uint32_t PORT_Pin)
{
  /* Check the parameters */
  assert_param(IS_PORT_ALL_PERIPH(PORTx));
  assert_param(IS_PORT_PIN(PORT_Pin));
  assert_param(IS_NOT_JTAG_PIN(PORTx, PORT_Pin));

  PORTx->RXTX &= ~(PORT_Pin | JTAG_PINS(PORTx));
}

void PORT_WriteBit(MDR_PORT_TypeDef* PORTx, uint32_t PORT_Pin, BitAction BitVal)
{
  uint32_t portdata;
  /* Check the parameters */
  assert_param(IS_PORT_ALL_PERIPH(PORTx));
  assert_param(IS_GET_PORT_PIN(PORT_Pin));
  assert_param(IS_PORT_BIT_ACTION(BitVal));
  assert_param(IS_NOT_JTAG_PIN(PORTx, PORT_Pin));

  portdata = PORTx->RXTX & (~JTAG_PINS(PORTx));
  if (BitVal != Bit_RESET)
  {
    PORTx->RXTX = portdata | PORT_Pin;
  }
  else
  {
    PORTx->RXTX = portdata & (~PORT_Pin);
  }
}

void PORT_Write(MDR_PORT_TypeDef* PORTx, uint32_t PortVal)
{
  /* Check the parameters */
  assert_param(IS_PORT_ALL_PERIPH(PORTx));
  assert_param(IS_NOT_JTAG_PIN(PORTx, PortVal));

  PORTx->RXTX = PortVal & (~JTAG_PINS(PORTx));
}
