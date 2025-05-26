/*
 * themes.c
 *
 *  Created on: Mar 14, 2025
 *      Author: Matth
 */


#include "ui.h"

const char *float_with_units[] = {"%.0f%s", "%.1f%s", "%.2f%s"};
const char *float_only[] = {"%.0f", "%.1f", "%.2f"};
const char *two_float_with_slash[] = {"%.0f/%.0f", "%.1f/%.1f", "%.2f/%.2f"};

static const float pow10_table[] = {1.0f, 10.0f, 100.0f};

int32_t scale_float( float val, uint8_t precision )
{
	return (int32_t)(val * pow10_table[precision]);
}

void label_set_text_fmt_with_check(lv_obj_t * obj, const char * fmt, ...)
{
    LV_ASSERT_OBJ(obj, MY_CLASS);
    LV_ASSERT_NULL(fmt);

    // Format the new string first to compare it
    char new_text[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(new_text, sizeof(new_text), fmt, args);
    va_end(args);

	if (strcmp(lv_label_get_text(obj), new_text) != 0)
		lv_label_set_text(obj, new_text);
}

lv_obj_t * add_gauge( GAUGE_THEME theme, int32_t x, int32_t y, lv_obj_t * parent, PID_DATA * pid)
{
	switch(theme)
	{
		case GAUGE_THEME_GRUMPY_CAT:
			return add_grumpy_cat_gauge( x, y, parent, pid);
		case GAUGE_THEME_LINEAR:
			return add_linear_gauge( x, y, parent, pid);
		case GAUGE_THEME_RADIAL:
			return add_radial_gauge( x, y, parent, pid);
		case GAUGE_THEME_STOCK_ST:
		default:
			return add_stock_st_gauge( x, y, parent, pid);
	}
}
