/*
 * ui.h
 *
 *  Created on: Mar 30, 2025
 *      Author: Matth
 */

#ifndef UI_H_
#define UI_H_

#include <stdbool.h>
#include "lvgl.h"
#include "lib_pid.h"
#include "lvgl_port_display.h"
#include "ke_config.h"

#define X_OFFSET 21 // Screen is not perfectly centered, shift by X to center in housing
#define GAUGE_PADDING 5
#define Y_HEIGHT 118
#define X_PADDING 124
#define ANIM_SPEED 50

#define OSPI_BASE_ADDRESS           0xA0000000u
#define BACKGROUND_OFFSET           0x00400000u
#define BACKGROUND_IMAGE_COUNT      (15U)
#define BACKGROUND_BLOCK_SIZE       (0x10000U) // 64KB
#define BACKGROUND_PIXEL_WIDTH      UI_HOR_RES
#define BACKGROUND_PIXEL_HEIGHT     UI_VER_RES
#define BACKGROUND_BYTES_PER_PIXEL  UI_BYTES_PER_PIXEL       // e.g. ARGB8888 = 4 bytes, RGB565 = 2 bytes
#define BACKGROUND_BASE_ADDRESS    (OSPI_BASE_ADDRESS + BACKGROUND_OFFSET) // First 4MB is the binary

// Macro to perform integer ceiling division
#define CEIL_DIV(x, y)              (((x) + (y) - 1) / (y))

// Raw image size in bytes
#define BACKGROUND_RAW_SIZE         (BACKGROUND_PIXEL_WIDTH * BACKGROUND_PIXEL_HEIGHT * BACKGROUND_BYTES_PER_PIXEL)

// Aligned image size (to next 64KB boundary)
#define BACKGROUND_IMAGE_SIZE       (CEIL_DIV(BACKGROUND_RAW_SIZE, BACKGROUND_BLOCK_SIZE) * BACKGROUND_BLOCK_SIZE)

extern const char *float_with_units[3];
extern const char *float_only[3];
extern const char *two_float_with_slash[3];

typedef struct {
	// PID real-time data
	PID_DATA * pid;

	// Last gauge update
	uint32_t timestamp;

	// Last recorded pid_value (different than tracked pid value)
	float pid_value;

	// Value that is currently on-screen, may be scaled (e.g. 254.69 --> 25469)
	int32_t value_label;

	// Min that is currently on-screen, may be scaled (e.g. 254.69 --> 25469)
	int32_t min_label;

	// Max that is currently on-screen, may be scaled (e.g. 254.69 --> 25469)
	int32_t max_label;

	// The target position of the object, used for smoothing when needed
	int32_t target_y;
} GAUGE_DATA;

void build_ui(void);
void ui_service(void);
void ui_tick();
void ui_reset(void);
void skip_splash(void);

lv_obj_t * add_grumpy_cat_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, GAUGE_DATA* data);
lv_obj_t * add_linear_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, GAUGE_DATA* data);
lv_obj_t * add_radial_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, GAUGE_DATA* data);
lv_obj_t * add_arc_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, GAUGE_DATA* data);
lv_obj_t * add_stock_st_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, GAUGE_DATA* data);
lv_obj_t * add_stock_rs_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, GAUGE_DATA* data);
lv_obj_t * add_digital_gauge( int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, GAUGE_DATA* data);

lv_obj_t * add_gauge( GAUGE_THEME theme, int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, GAUGE_DATA* data);
int32_t scale_float( float val, uint8_t precision );
int32_t round_to_precision( float val, uint8_t precision );
void label_set_text_fmt_with_check(lv_obj_t * obj, const char * fmt, ...);
lv_color_t get_needle_color_from_value(float value, float min, float max);
bool pid_value_changed(GAUGE_DATA *data);
bool pid_value_label_changed(GAUGE_DATA *data);
bool pid_min_label_changed(GAUGE_DATA *data);
bool pid_max_label_changed(GAUGE_DATA *data);

lv_obj_t * add_alert( lv_obj_t * parent );
bool get_alert(void);
void set_alert(char *msg);
void clear_alert(void);
lv_obj_t * add_system_message( lv_obj_t * parent );
bool get_system_message(void);
void set_system_message(char *msg);
void clear_system_message(void);

typedef enum {
	ADDR_MEMORYMAPPED_DISABLED,
	ADDR_MEMORYMAPPED_ENABLED
} ADDR_MEMORYMAPPED_MODE;

uint32_t get_background_addr(uint8_t idx, ADDR_MEMORYMAPPED_MODE mode);
uint32_t calc_crc32(uint8_t idx, uint32_t reserved);

#endif /* UI_H_ */
