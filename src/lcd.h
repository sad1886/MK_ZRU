#define __LCD_H_

/* Includes ------------------------------------------------------------------*/
#include <MDR32F9x.h>
#include <MDR32F9Qx_port.h>
#include "MDR32F9Qx_board.h"

/* LCD crystal numbers */
typedef enum
{
    LCD_CRYSTAL1    = 0,
    LCD_CRYSTAL2    = 1,
    NUM_LCD_CRYSTALS
}LCD_Crystal;

/* Bit and byte output methods */
typedef enum
{
    MET_OR          = 0,
    MET_XOR         = 1,
    MET_NOT_OR      = 2,
    MET_NOT_XOR     = 3,
    MET_AND         = 4,
    NUM_LCD_METHOD
}LCD_Method;

/* LCD status flags */
#define LCD_BUSY_Pos      7
#define LCD_BUSY_Msk      (uint32_t)(1 << LCD_BUSY_Pos)

#define LCD_ONOFF_Pos     5
#define LCD_ONOFF_Msk     (uint32_t)(1 << LCD_ONOFF_Pos)

#define LCD_RESET_Pos     4
#define LCD_RESET_Msk     (uint32_t)(1 << LCD_RESET_Pos)

/* LCD Data Port */
#define LCD_DATA_PORT      MDR_PORTA

#define LCD_DATA_BUS_8     (PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2 | PORT_Pin_3 | \
                            PORT_Pin_4 | PORT_Pin_5 | PORT_Pin_6 | PORT_Pin_7)


/* LCD screen resolution (in pixels) */
#define MAX_X       127
#define MAX_Y       63

/* Specific LCD control commands */
#define LCD_ON                      WriteLCD_Cmd(0x3F)
#define LCD_OFF                     WriteLCD_Cmd(0x3E)
#define LCD_START_LINE(x)           WriteLCD_Cmd(0xC0 | x)
#define LCD_SET_PAGE(x)             WriteLCD_Cmd(0xB8 | x)
#define LCD_SET_ADDRESS(x)          WriteLCD_Cmd(0x40 | x)

/* Display context */
extern LCD_Crystal CurrentCrystal;  /*!< Currently selected crystal */
extern LCD_Method CurrentMethod;    /*!< Current display method    */

#include "lcd_MDR32F9Qx.h"

void ResetLCD(void);
void SetCrystal(LCD_Crystal num);
void WriteLCD_Cmd(uint32_t val);
void WriteLCD_Data(uint32_t val);
uint32_t ReadLCD_Cmd(void);
uint32_t ReadLCD_Data(void);
void LCD_INIT(void);
void LCD_CLS(void);
