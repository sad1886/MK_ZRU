/**
  ******************************************************************************
  * @file    lcd_MDR32F9Qx.h
  * @author  Phyton Application Team
  * @version V3.0.0
  * @date    10.09.2011
  * @brief   This file contains all the specific types, constants and variables
  *          for the LCD driver for MDR32F9Q2_Rev0, MDR32F9Q2_Rev1,
  *          MDR32F9Q3_Rev0 and MDR32F9Q3_Rev1 evaluation boards.
  ******************************************************************************
  * <br><br>
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 Phyton</center></h2>
  */

#if defined (USE_MDR32F9Q2_Rev0)  || defined (USE_MDR32F9Q2_Rev1) || \
    defined (USE_MDR32F9Q3_Rev0) || defined (USE_MDR32F9Q3_Rev1)

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H

/* Includes ------------------------------------------------------------------*/
//#include <MDR32F9x.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_rst_clk.h>
#include "types.h"


/* LCD control port pins definitions */

/* LCD crystals control */
#define LCD_CRYSTAL_PINs                 (PORT_Pin_7 | PORT_Pin_8)
#define LCD_CRYSTAL_POS                  7
#define LCD_CRYSTAL_PORT                 MDR_PORTB
#endif

/* LCD command/data switching */
#define LCD_CMD_DATA_PIN                 (PORT_Pin_0)
#define LCD_CMD_DATA_PORT                MDR_PORTC

/* LCD read/write switching */
#define LCD_RD_WR_PIN                    (PORT_Pin_10)
#define LCD_RD_WR_PORT                   MDR_PORTB

/* LCD CLOCK signal control */
#define LCD_CLOCK_PIN                    (PORT_Pin_1)
#define LCD_CLOCK_PORT                   MDR_PORTC

/* LCD RESET signal control */
#define LCD_RESET_PIN                    (PORT_Pin_9)
#define LCD_RESET_PORT                   MDR_PORTB

#endif /* __LCD_H */


