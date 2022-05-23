/**************************************************************************************
*	Сторожевой таймер IWDG																															*
***************************************************************************************/
#include "MDR32F9x.h"
#include "MDR32F9Qx_iwdg.h"

// Prescaler | Reload | Расчёт таймаута                 | Измеренное время таймаута
// 256       | 156    | (256 / 40000) * 156 = 998 мс    | 700 мс
// 256       | 0x30D  | (256 / 40000) * 0x30D = 4,99 c  | 
// 256       | 0xFFF  | (256 / 40000) * 0xFFF = 26,2 c  | 17 с

// Инициализация внутреннего сторожевого таймера.
// Таймаут на сброс -- 1 секунда.
// FIXME: Разобраться с проблемой. Измерил осциллографом период сброса -- 700 мс, а должен быть 1000 мс. 
// Получается, что частота тактирования внутреннего сторожевого таймера равна 57 кГц (номер на корпусе МК -- 1150).
void initInternalWatchdog(void)
{
    // В документации на микроконтроллер не указан источник тактирования внутреннего сторожевого
    // таймера.
    // На форуме (http://forum.milandr.ru/viewtopic.php?t=281) был следующий ответ:
    //   > Независимый сторожевой таймер тактируется сигналом внутреннего низкочастотного
    //   > генератора (LSI) частотой 40 кГц.
    enum
    {
        kPrescaler = IWDG_Prescaler_256,
        kReload    = 50 														// 1сек - 1/(256/40000Гц)=156.25 ~= 156,  5сек - 5/(256/40000Гц)=781.25 ~= 781
    };

    // Разрешаем тактирование схемы внутреннего сторожевого таймера.
    //RST_CLK_PCLKcmd(RST_CLK_PCLK_IWDG, ENABLE);
		MDR_RST_CLK->PER_CLOCK |= (1<<13);							// Подача тактирования на IWDG

    // Установка предварительного делителя частоты и значения перезагрузки доступна только
    // после установки разрешения доступа.
    IWDG_WriteAccessEnable();												// MDR_IWDG->KR = 0x5555; разрешает доступ по записи к регистрам PR и RLR
    IWDG_SetReload(kReload);												// Значение перезагрузки сторожевого таймера
    IWDG_SetPrescaler(kPrescaler);									// Делитель частоты сторожевого таймера
    IWDG_WriteAccessDisable();											// MDR_IWDG->KR = 0;

    // Устанавливаем значение загружаемое в сторожевой таймер равным kReload. Если этого не сделать,
    // то вслучае, если микроконтроллер зависнет до первого сброса внутреннего сторожевого таймера
    // (вызов функции resetInternalWatchdog) -- время таймаута не будет соответствовать расчетному!
    IWDG_ReloadCounter();														// MDR_IWDG->KR = 0xAAAA

    IWDG_Enable(); 																	// MDR_IWDG->KR = 0xCCCC. Разрешаем работу сторожевого таймера.
}

// Перезапуск (сброс) внутреннего сторожевого таймера.
void resetInternalWatchdog(void)
{
    IWDG_ReloadCounter();
}

