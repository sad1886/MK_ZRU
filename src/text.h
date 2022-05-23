/* Define to prevent recursive inclusion -------------------------------------*/
#define __TEXT_H

/* Includes ------------------------------------------------------------------*/
#include "font_defs.h"

#define DefaultFontWidth                 6
#define DefaultFontHeight                8

/* LCD lines */
#define Interline                        2
#define LineMessage1                     DefaultFontHeight + 4
#define LineMessage2                     LineMessage1 + DefaultFontHeight + Interline
#define LineMessage3                     LineMessage2 + DefaultFontHeight + Interline
#define LineMessage4                     LineMessage3 + DefaultFontHeight + Interline
#define LineMessage5                     LineMessage4 + DefaultFontHeight + Interline

/* Styles */
typedef enum
{
  StyleSimple,
  StyleBlink,
  StyleFlipFlop,
  StyleVibratory
}TextStyle;

/* Macro for calculation of an address of symbol description structure in the
 * symbol description table. Gets symbol code and font description address,
 * returns symbol description address. */
#define Get_Char_Data_Addr(ch)    \
  (CurrentFont)->pData + (ch) * (CurrentFont)->Width * ((((CurrentFont)->Height % 8) != 0) ? (1 + (CurrentFont)->Height / 8) : ((CurrentFont)->Height / 8))


/* Selected font (used for symbol output) */
extern sFONT *CurrentFont;

/* Output symbols to screen using current font */
void LCD_PUTC(uint8_t x, uint8_t y, uint8_t ch);
void LCD_PUTS(uint8_t x, uint8_t y, ucint8_t* str);


