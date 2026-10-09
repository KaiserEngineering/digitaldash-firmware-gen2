/*
 * lv_conf.h (simulator)
 *
 * Reuses the firmware LVGL configuration and disables the STM32-only
 * draw/display backends so the UI renders with the software renderer.
 */
#ifndef SIM_LV_CONF_H
#define SIM_LV_CONF_H

#include "../../lv_conf.h"

#undef LV_USE_NEMA_GFX
#define LV_USE_NEMA_GFX 0
#undef LV_USE_NEMA_VG
#define LV_USE_NEMA_VG 0
#undef LV_USE_ST_LTDC
#define LV_USE_ST_LTDC 0
#undef LV_USE_DRAW_DMA2D
#define LV_USE_DRAW_DMA2D 0
#undef LV_USE_SCALE
#define LV_USE_SCALE 0  /* needs LV_USE_LINE, unused by the UI */
#undef LV_USE_WIN
#define LV_USE_WIN 0    /* needs LV_USE_BUTTON, unused by the UI */
#undef LV_USE_SYSMON
#define LV_USE_SYSMON 0

#endif /* SIM_LV_CONF_H */
