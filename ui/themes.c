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

lv_obj_t * add_gauge( GAUGE_THEME theme, int32_t x, int32_t y, int32_t w, int32_t h, lv_obj_t * parent, PID_DATA * pid)
{
	switch(theme)
	{
		case GAUGE_THEME_GRUMPY_CAT:
			return add_grumpy_cat_gauge( x, y, w, h, parent, pid);
		case GAUGE_THEME_LINEAR:
			return add_linear_gauge( x, y, w, h, parent, pid);
		case GAUGE_THEME_RADIAL:
			return add_radial_gauge( x, y, w, h, parent, pid);
		case GAUGE_THEME_DIGITAL:
			return add_digital_gauge( x, y, w, h, parent, pid);
		case GAUGE_THEME_STOCK_RS:
			return add_stock_rs_gauge( x, y, w, h, parent, pid);
		case GAUGE_THEME_STOCK_ST:
		default:
			return add_stock_st_gauge( x, y, w, h, parent, pid);
	}
}

// Convert HSV to RGB — full saturation and value
static lv_color_t lv_color_from_hue(float hue_deg)
{
    float c = 1.0f; // Chroma (fully saturated)
    float x = 1.0f - fabsf(fmodf(hue_deg / 60.0f, 2) - 1.0f);
    float r = 0, g = 0, b = 0;

    if (hue_deg < 60)       { r = c; g = x; b = 0; }
    else if (hue_deg < 120) { r = x; g = c; b = 0; }
    else if (hue_deg < 180) { r = 0; g = c; b = x; }
    else if (hue_deg < 240) { r = 0; g = x; b = c; }
    else if (hue_deg < 300) { r = x; g = 0; b = c; }
    else                    { r = c; g = 0; b = x; }

    return lv_color_make((uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255));
}

lv_color_t get_needle_color_from_value(float value, float min, float max)
{
    float ratio = 0.0f;
    if (max > min) {
        ratio = (value - min) / (max - min);
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio > 1.0f) ratio = 1.0f;
    }

    // Hue: 240 (blue) → 0 (red)
    float hue = 240.0f - (240.0f * ratio);  // In degrees
    return lv_color_from_hue(hue);
}
