/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FONT_DEFS_H
#define __FONT_DEFS_H

/* Includes ------------------------------------------------------------------*/
#include "types.h"

typedef struct
{
  uint32_t  Height;        /*!< Specifies the character height in pixels          */
  uint32_t  Width;         /*!< Specifies the character width in pixels           */
  uint32_t  Count;         /*!< Specifies the count of characters in the font - 1 */
  ucint8_t  *pData;        /*!< Specifies the font table address                 */
}sFONT;

extern sFONT Font_12x16;         /*!< Font 12 õ 16 pixels (normal) */

#endif /* __FONT_DEFS_H */

