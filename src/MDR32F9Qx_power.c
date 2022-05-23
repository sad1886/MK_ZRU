/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_power.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_config.h"

/* POWER registers bit address in the alias region */
#define PERIPH_BASE                 0x40000000
#define PERIPH_BB_BASE              0x42000000
#define POWER_OFFSET                (MDR_POWER_BASE - PERIPH_BASE)
#define BKP_OFFSET                  (MDR_BKP_BASE - PERIPH_BASE)

#define SFR_OFFSET(TP, MOD, SFR)    ((uint32_t)&((TP*)MOD)->SFR)
#define BB_ADDR(TP, MOD, SFR, BIT)  (PERIPH_BB_BASE + SFR_OFFSET(TP, MOD, SFR)*32 + BIT*4)
#define POWER_BB(SFR, BIT)          BB_ADDR(MDR_POWER_TypeDef, POWER_OFFSET, SFR, BIT)
#define BKP_BB(SFR, BIT)            BB_ADDR(MDR_BKP_TypeDef, BKP_OFFSET, SFR, BIT)

#define POWER_PVDEN_BB              POWER_BB(PVDCS, POWER_PVDCS_PVDEN_Pos)

#define BKP_FPOR_BB                 BKP_BB(REG_0E, BKP_REG_0E_FPOR_Pos)
#define BKP_STANDBY_BB              BKP_BB(REG_0F, BKP_REG_0F_STANDBY_Pos)

/* --------------------- POWER registers bit mask ------------------------ */

/* BKP_REG0E register bit mask */
#define DUcc_Mask         ((uint32_t)(BKP_REG_0E_LOW_Msk | BKP_REG_0E_SELECTRI_Msk))
#define DUccTrim_Mask     ((uint32_t)BKP_REG_0E_TRIM_Msk)

void POWER_DeInit(void)
{
  MDR_POWER -> PVDCS = (uint32_t) 0x00000000;
}

void POWER_DUccMode(uint32_t DUccMode)
{
  uint32_t tmpreg;
  /* Check the parameters */
  assert_param(IS_POWER_DUCC_MODE(DUccMode));
  /* Clear POWER_REG0E[5:0] bits */
  tmpreg  = MDR_BKP -> REG_0E & (uint32_t) (~DUcc_Mask);
  /* Set POWER_REG0E[5:0] bits according to DUcc mode */
  tmpreg |= DUcc_Mask & DUccMode;
  MDR_BKP -> REG_0E = tmpreg;
}

void POWER_DUccTrim(uint32_t DUccTrim)
{
  uint32_t tmpreg;
  /* Check the parameters */
  assert_param(IS_POWER_DUCC_TRIM(DUccTrim));
  /* Clear POWER_REG0E[5:0] bits */
  tmpreg  = MDR_BKP -> REG_0E & (uint32_t) (~DUccTrim_Mask);
  /* Set POWER_REG0E[5:0] bits according to DUcc mode */
  tmpreg |= DUccTrim_Mask & DUccTrim;
  MDR_BKP -> REG_0E = tmpreg;
}

void POWER_DUccStandby(void)
{
  *(__IO uint32_t *) BKP_STANDBY_BB = (uint32_t) 0x01;
}

void POWER_PVDlevelConfig(uint32_t POWER_PVDlevel)
{
  uint32_t tmpreg;
  /* Check the parameters */
  assert_param(IS_POWER_PVD_LEVEL(POWER_PVDlevel));
  tmpreg = MDR_POWER->PVDCS;
  /* Clear PLS[5:3] bits */
  tmpreg &= (uint32_t) ~POWER_PVDCS_PLS_Msk;
  /* Set PLS[5:3] bits according to POWER_PVDlevel value */
  tmpreg |= (POWER_PVDlevel & POWER_PVDCS_PLS_Msk);
  /* Store the new value */
  MDR_POWER->PVDCS = tmpreg;
}

void POWER_PVBDlevelConfig(uint32_t POWER_PVBDlevel)
{
  uint32_t tmpreg;
  /* Check the parameters */
  assert_param(IS_POWER_PVBD_LEVEL(POWER_PVBDlevel));
  tmpreg = MDR_POWER->PVDCS;
  /* Clear PBLS[2:1] bits */
  tmpreg &= (uint32_t) ~POWER_PVDCS_PBLS_Msk;
  /* Set PBLS[2:1] bits according to POWER_PVBDlevel value */
  tmpreg |= (POWER_PVBDlevel & POWER_PVDCS_PBLS_Msk);
  /* Store the new value */
  MDR_POWER->PVDCS = tmpreg;
}

void POWER_SetFlagPOR(void)
{
  *(__IO uint32_t *) BKP_FPOR_BB = (uint32_t) 0x01;
}

ErrorStatus POWER_FlagPORstatus(void)
{
  ErrorStatus state = ERROR;
  if (*(__IO uint32_t *) BKP_FPOR_BB == 0)
  {
    state = SUCCESS;
  }
  return state;
}

void POWER_PVDenable(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  *(__IO uint32_t *) POWER_PVDEN_BB = (uint32_t)NewState;
}

FlagStatus POWER_GetFlagStatus(uint32_t POWER_FLAG)
{
  FlagStatus status;
  /* Check the parameters */
  assert_param(IS_POWER_FLAG(POWER_FLAG));

  if ((MDR_POWER->PVDCS & POWER_FLAG) != (uint32_t)RESET)
  {
    status = SET;
  }
  else
  {
    status = RESET;
  }
  /* Return the flag status */
  return status;
}

void POWER_ClearFlag(uint32_t POWER_FLAG)
{
  /* Check the parameters */
  assert_param(IS_POWER_FLAG(POWER_FLAG));

  MDR_POWER->PVDCS &= (uint32_t) ~POWER_FLAG;
}


void POWER_PVD_ITconfig(uint32_t POWER_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_POWER_PVD_IT(POWER_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    MDR_POWER->PVDCS |= POWER_IT;
  }
  else
  {
    MDR_POWER->PVDCS &= (uint32_t)~POWER_IT;
  }
}


void POWER_PVD_INVconfig(uint32_t POWER_INV, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_POWER_PVD_INV(POWER_INV));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    MDR_POWER->PVDCS |= POWER_INV;
  }
  else
  {
    MDR_POWER->PVDCS &= (uint32_t)~POWER_INV;
  }
}

void POWER_EnterSTOPMode(FunctionalState POWER_Regulator_state, uint8_t POWER_STOPentry)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(POWER_Regulator_state));
  assert_param(IS_POWER_STOP_ENTRY(POWER_STOPentry));

  /* Set UDcc stanby status */
   *(__IO uint32_t *) BKP_STANDBY_BB = (uint32_t) (!POWER_Regulator_state);

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  *(__IO uint32_t *) SCB -> SCR |= SCB_SCR_SLEEPDEEP_Msk;

  /* Select STOP mode entry --------------------------------------------------*/
  if(POWER_STOPentry == POWER_STOPentry_WFI)
  {
    /* Request Wait For Interrupt */
    __WFI();
  }
  else
  {
    /* Request Wait For Event */
    __WFE();
  }
}


void POWER_EnterSTANDBYMode(void)
{
  /* Select STANDBY mode */
  *(__IO uint32_t *) BKP_STANDBY_BB = (uint32_t) 0x01;

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  *(__IO uint32_t *) SCB -> SCR |= SCB_SCR_SLEEPDEEP_Msk;

  /* Request Wait For Interrupt */
  __WFI();
}



