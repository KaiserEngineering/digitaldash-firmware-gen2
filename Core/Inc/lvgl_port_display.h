#ifndef __LVGL_PORT_DISPLAY_H
#define __LVGL_PORT_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "lvgl.h"

/*********************
 *      DEFINES
 *********************/

#define UI_HOR_RES    800
#define UI_VER_RES    200
#define ACT_HOR_RES    800
#define ACT_VER_RES    480
#define DISP_PARTIAL 0

// LCD Timing
#define HSYNC 30
#define HFP 210
#define HBP 58

#define VSYNC 13
#define VFP 22
#define VBP 180

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void
lvgl_display_init (void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* __LVGL_PORT_DISPLAY_H */
