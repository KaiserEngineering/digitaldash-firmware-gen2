/*
 * graph.c
 *
 *  Created on: Mar 25, 2026
 *      Author: Ian Cleaver @CL_eav
 *
 * Scrolling line-chart gauge theme.
 *
 * Layout within the gauge container:
 *   child 0  — lv_chart  (fills bottom portion of the container)
 *   child 1  — PID name label  (top-left)
 *   child 2  — Current value + unit label  (top-right)
 *
 * The lv_chart widget maintains an internal circular buffer of
 * GRAPH_SAMPLE_COUNT points.  Each time the PID timestamp changes
 * (i.e. new data has arrived) the event callback pushes the latest
 * value into the chart with lv_chart_set_next_value(), which
 * automatically shifts the series left so the display scrolls.
 *
 * Y-axis range is set from pid->lower_limit to pid->upper_limit,
 * scaled by pid->precision to keep the integer chart values aligned
 * with the rest of the UI's scale_float() convention.
 */

#include "ui.h"
#include "lib_pid.h"

/* Number of data points retained in the chart's circular buffer.
 * At a typical OBD-II update rate of ~5 Hz this gives ~20 seconds
 * of visible history. */
#define GRAPH_SAMPLE_COUNT  100U

/* Pixel height reserved at the top of the container for the
 * PID name and current-value labels. */
#define GRAPH_LABEL_HEIGHT  28

/* Horizontal inset applied to the chart and labels within the gauge
 * container, keeping content clear of the physical case bezel.
 * Matches the BAR_PADDING convention used by the linear gauge (100px
 * total = 50px per side). */
#define GRAPH_H_PADDING     130

/* Only plot every Nth PID update into the chart.  The value label still
 * updates every cycle so the live reading is always current.
 * At a typical 10 Hz OBD-II rate, GRAPH_SUBSAMPLE 5 gives one chart
 * point every ~0.5 s → 100 samples covers ~50 seconds of history. */
#define GRAPH_SUBSAMPLE     5

static void event_cb(lv_event_t * e)
{
    GAUGE_DATA * data    = (GAUGE_DATA *)lv_event_get_param(e);
    lv_obj_t  * gauge    = lv_event_get_target(e);
    lv_obj_t  * chart    = lv_obj_get_child(gauge, 0);
    lv_obj_t  * val_lbl  = lv_obj_get_child(gauge, 2);

    if (pid_value_changed(data))
    {
        /* Auto-scale Y axis to session min/max whenever they change.
         * Guard against a zero-range at session start (min == max)
         * by falling back to the PID's configured limits. */
        if (pid_min_label_changed(data) || pid_max_label_changed(data))
        {
            int32_t y_min, y_max;
            if (data->pid->pid_min < data->pid->pid_max)
            {
                y_min = scale_float(data->pid->pid_min, data->pid->precision);
                y_max = scale_float(data->pid->pid_max, data->pid->precision);
            }
            else
            {
                y_min = scale_float(data->pid->lower_limit, data->pid->precision);
                y_max = scale_float(data->pid->upper_limit, data->pid->precision);
            }
            lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_min, y_max);
        }

        /* Plot every Nth update to stretch history without increasing
         * sample count or rendering cost. */
        static uint8_t skip = 0;
        if (++skip >= GRAPH_SUBSAMPLE)
        {
            skip = 0;
            lv_chart_series_t * ser = lv_chart_get_series_next(chart, NULL);
            lv_chart_set_next_value(chart, ser,
                scale_float(data->pid->pid_value, data->pid->precision));
        }

        /* Update the current-value label every cycle (always live) */
        char buf[32];
        snprintf(buf, sizeof(buf),
                 float_with_units[data->pid->precision],
                 data->pid->pid_value,
                 data->pid->unit_label);
        lv_label_set_text(val_lbl, buf);
    }
}

lv_obj_t * add_graph_gauge(int32_t x, int32_t y, int32_t w, int32_t h,
                            lv_obj_t * parent, GAUGE_DATA * data)
{
    /* ------------------------------------------------------------------ */
    /* Outer container                                                      */
    /* ------------------------------------------------------------------ */
    lv_obj_t * gauge = lv_obj_create(parent);
    lv_obj_remove_style_all(gauge);
    lv_obj_set_width(gauge, w);
    lv_obj_set_height(gauge, h);
    lv_obj_set_x(gauge, x);
    lv_obj_set_y(gauge, y);
    lv_obj_set_align(gauge, LV_ALIGN_TOP_MID);
    lv_obj_remove_flag(gauge, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_user_data(gauge, data);
    lv_obj_add_event_cb(gauge, event_cb, LV_EVENT_REFRESH, data);
#if UI_CONTAINER_DEBUG
    lv_obj_set_style_border_width(gauge, 2, 0);
    lv_obj_set_style_border_color(gauge, lv_color_white(), 0);
    lv_obj_set_style_border_opa(gauge, LV_OPA_COVER, 0);
#endif

    /* ------------------------------------------------------------------ */
    /* Chart widget (child 0)                                               */
    /* ------------------------------------------------------------------ */
    int32_t chart_h = h - GRAPH_LABEL_HEIGHT;

    lv_obj_t * chart = lv_chart_create(gauge);
    lv_obj_set_size(chart, w - GRAPH_H_PADDING, chart_h);
    lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_remove_flag(chart, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, GRAPH_SAMPLE_COUNT);

    /* Set Y-axis range from the PID's configured limits, scaled to match
     * the integer precision used throughout the gauge system. */
    int32_t y_min = scale_float(data->pid->lower_limit, data->pid->precision);
    int32_t y_max = scale_float(data->pid->upper_limit, data->pid->precision);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_min, y_max);

    /* Grid lines: 4 horizontal divisions, 8 vertical divisions */
    lv_chart_set_div_line_count(chart, 4, 8);

    /* Chart background and border */
    lv_obj_set_style_bg_color(chart, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(chart, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(chart, lv_color_hex(0x444444), LV_PART_MAIN);
    lv_obj_set_style_pad_all(chart, 4, LV_PART_MAIN);

    /* Grid line colour */
    lv_obj_set_style_line_color(chart, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_line_opa(chart, LV_OPA_COVER, LV_PART_MAIN);

    /* Hide the per-point dots so only the connecting line is visible */
    lv_obj_set_style_width(chart, 0, LV_PART_INDICATOR);
    lv_obj_set_style_height(chart, 0, LV_PART_INDICATOR);

    /* Line width for the data series */
    lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);

    /* Add the data series (cyan — consistent with the rest of the UI palette) */
    lv_chart_series_t * ser = lv_chart_add_series(
        chart,
        lv_color_hex(0x00DFFF),
        LV_CHART_AXIS_PRIMARY_Y);

    /* Pre-fill the buffer with the current PID value so the chart
     * doesn't display a ramp-up artefact on first display. */
    int32_t init_val = scale_float(data->pid->pid_value, data->pid->precision);
    for (uint16_t i = 0; i < GRAPH_SAMPLE_COUNT; i++)
        lv_chart_set_next_value(chart, ser, init_val);

    /* ------------------------------------------------------------------ */
    /* PID name label (child 1, top-left)                                  */
    /* ------------------------------------------------------------------ */
    lv_obj_t * pid_label = lv_label_create(gauge);
    lv_obj_set_style_text_font(pid_label, &Discongnate_22, LV_PART_MAIN);
    lv_obj_set_style_text_color(pid_label, lv_color_hex(0x00DFFF), LV_PART_MAIN);
    lv_label_set_text(pid_label, data->pid->label);
    lv_obj_align(pid_label, LV_ALIGN_TOP_LEFT, GRAPH_H_PADDING / 2, 2);

    /* ------------------------------------------------------------------ */
    /* Current value + unit label (child 2, top-right)                     */
    /* ------------------------------------------------------------------ */
    lv_obj_t * val_label = lv_label_create(gauge);
    lv_obj_set_style_text_font(val_label, &Discongnate_22, LV_PART_MAIN);
    lv_obj_set_style_text_color(val_label, lv_color_hex(0x00DFFF), LV_PART_MAIN);

    char buf[32];
    snprintf(buf, sizeof(buf),
             float_with_units[data->pid->precision],
             data->pid->pid_value,
             data->pid->unit_label);
    lv_label_set_text(val_label, buf);
    lv_obj_align(val_label, LV_ALIGN_TOP_RIGHT, -1 * (GRAPH_H_PADDING / 2), 2);

    return gauge;
}
