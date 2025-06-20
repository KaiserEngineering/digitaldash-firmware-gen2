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

// LCD Timing
#define HSYNC 30
#define HFP 16
#define HBP 130

#define VSYNC 13
#define VFP 12
#define VBP 206

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void
lvgl_display_init (void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* __LVGL_PORT_DISPLAY_H */
