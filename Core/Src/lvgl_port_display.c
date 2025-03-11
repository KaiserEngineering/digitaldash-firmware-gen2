/*********************
 *      INCLUDES
 *********************/

#include "lvgl_port_display.h"
#include "main.h"
#include "ltdc.h"

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lvgl_display_init (void)
{
	/* display initialization */

#if LV_COLOR_DEPTH == 16
  static __attribute__((aligned(32))) uint8_t buf_2[UI_HOR_RES * UI_VER_RES * 2];
  lv_st_ltdc_create_direct((void *)0x20000000, buf_2, 0);
#elif LV_COLOR_DEPTH == 24 || LV_COLOR_DEPTH == 32
  static __attribute__((section(".noinit"), aligned(32))) uint8_t buf_1[UI_HOR_RES * UI_VER_RES * 4];
  static __attribute__((section(".noinit"), aligned(32))) uint8_t buf_2[UI_HOR_RES * UI_VER_RES * 4];
#if DISP_PARTIAL
  lv_st_ltdc_create_partial(buf_1, buf_2, sizeof(buf_1), 0);
#else
  lv_st_ltdc_create_direct(buf_1, buf_2, 0);
#endif
  HAL_LTDC_SetAddress(&hltdc, 0x20270000, 0);
#else
  #error LV_COLOR_DEPTH not supported
#endif
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
