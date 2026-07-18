/*
 * system_message.c
 *
 *  Created on: May 12, 2025
 *      Author: Matth
 */

#include "ui.h"
#include "stm32u5xx.h"

lv_obj_t * ui_system_message_container = NULL;
lv_obj_t * ui_system_message = NULL;
static lv_obj_t * ui_system_message_expiration = NULL;
static lv_timer_t * ui_system_message_timer = NULL;
static uint32_t system_message_started_ms = 0;
static uint32_t system_message_duration_ms = 0;
static bool system_message_show_expiration = false;

typedef struct {
    SYSTEM_MESSAGE_ID id;
    uint32_t duration_ms;
    bool show_expiration;
} SYSTEM_MESSAGE_REQUEST;

static volatile SYSTEM_MESSAGE_REQUEST pending_system_message = {
    .id = SYSTEM_MESSAGE_NONE,
    .duration_ms = 0U,
    .show_expiration = false
};

#define SYSTEM_MESSAGE_TIMER_PERIOD_MS 20U
#define SYSTEM_MESSAGE_BAR_HEIGHT 5
#define SYSTEM_MESSAGE_BAR_PADDING 12

static void system_message_timer_cb(lv_timer_t * timer)
{
    (void)timer;

    if ((ui_system_message_container == NULL) ||
        lv_obj_has_flag(ui_system_message_container, LV_OBJ_FLAG_HIDDEN) ||
        (system_message_duration_ms == 0U))
    {
        return;
    }

    uint32_t elapsed_ms = lv_tick_elaps(system_message_started_ms);
    if (elapsed_ms >= system_message_duration_ms)
    {
        clear_system_message();
        return;
    }

    if (system_message_show_expiration)
    {
        uint32_t remaining_ms = system_message_duration_ms - elapsed_ms;
        int32_t full_width = lv_obj_get_width(ui_system_message_container) -
                             SYSTEM_MESSAGE_BAR_PADDING;
        int32_t remaining_width = (int32_t)(((uint32_t)full_width * remaining_ms) /
                                             system_message_duration_ms);
        lv_obj_set_width(ui_system_message_expiration, remaining_width);
    }
}

static const char * system_message_text(SYSTEM_MESSAGE_ID id)
{
    switch (id)
    {
        case SYSTEM_MESSAGE_TESTER_PRESENT:
            return "OBD-II device detected - paused";
        case SYSTEM_MESSAGE_DYNAMIC_VIEW_DISABLED:
            return "Selected dynamic view is not enabled";
        case SYSTEM_MESSAGE_NONE:
        case SYSTEM_MESSAGE_RESERVED:
        default:
            return NULL;
    }
}

lv_obj_t * add_system_message(lv_obj_t * parent) {
    ui_system_message_container = lv_obj_create(parent);
    lv_obj_remove_style_all(ui_system_message_container);
    lv_obj_set_width(ui_system_message_container, 670);
    lv_obj_set_height(ui_system_message_container, 75);
    lv_obj_set_x(ui_system_message_container, X_OFFSET);
    lv_obj_set_y(ui_system_message_container, 5);
    lv_obj_set_align(ui_system_message_container, LV_ALIGN_TOP_MID);

    lv_obj_remove_flag(ui_system_message_container,
                       LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE |
                       LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE |
                       LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);

    lv_obj_set_style_radius(ui_system_message_container, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_system_message_container, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_system_message_container, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);  // fully opaque
    lv_obj_set_style_border_color(ui_system_message_container, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_system_message_container, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_system_message_container, 3, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Remove all shadow styles by setting them to 0 or default
    lv_obj_set_style_shadow_width(ui_system_message_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_system_message_container, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_system_message_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_offset_x(ui_system_message_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_offset_y(ui_system_message_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_system_message = lv_label_create(ui_system_message_container);
    lv_obj_set_width(ui_system_message, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_system_message, LV_SIZE_CONTENT);
    lv_obj_set_align(ui_system_message, LV_ALIGN_CENTER);
    lv_label_set_text(ui_system_message, "No message set");
    lv_obj_set_style_text_font(ui_system_message, &Discongnate_30, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_system_message_expiration = lv_obj_create(ui_system_message_container);
    lv_obj_remove_style_all(ui_system_message_expiration);
    lv_obj_set_width(ui_system_message_expiration, 0);
    lv_obj_set_height(ui_system_message_expiration, SYSTEM_MESSAGE_BAR_HEIGHT);
    lv_obj_set_align(ui_system_message_expiration, LV_ALIGN_BOTTOM_MID);
    lv_obj_set_y(ui_system_message_expiration, -8);
    lv_obj_set_style_radius(ui_system_message_expiration, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_system_message_expiration, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_system_message_expiration, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_flag(ui_system_message_expiration, LV_OBJ_FLAG_HIDDEN);

    if (ui_system_message_timer != NULL)
    {
        lv_timer_delete(ui_system_message_timer);
    }
    ui_system_message_timer = lv_timer_create(system_message_timer_cb,
                                               SYSTEM_MESSAGE_TIMER_PERIOD_MS,
                                               NULL);
    lv_timer_pause(ui_system_message_timer);

    lv_obj_add_flag(ui_system_message_container, LV_OBJ_FLAG_HIDDEN);
    return ui_system_message_container;
}

bool get_system_message(void)
{
	return (ui_system_message_container == NULL) ||
	       lv_obj_has_flag(ui_system_message_container, LV_OBJ_FLAG_HIDDEN);
}

static void display_system_message(const char *msg, uint32_t duration_ms,
                                   bool show_expiration)
{
    if ((msg == NULL) || (ui_system_message == NULL) ||
        (ui_system_message_container == NULL))
    {
        clear_system_message();
        return;
    }

    lv_label_set_text(ui_system_message, msg);
    system_message_started_ms = lv_tick_get();
    system_message_duration_ms = duration_ms;
    system_message_show_expiration = show_expiration && (duration_ms > 0U);

    if (ui_system_message_timer != NULL)
    {
        lv_timer_reset(ui_system_message_timer);
        if (duration_ms > 0U)
            lv_timer_resume(ui_system_message_timer);
        else
            lv_timer_pause(ui_system_message_timer);
    }

    // Allow label size to update
    lv_obj_update_layout(ui_system_message);  // force layout update so size is valid

    int label_width = lv_obj_get_width(ui_system_message);
    int padding = 60;  // padding for left/right sides

    // Set container width based on label width plus padding, min 200, max 670
    int new_width = label_width + padding;
    if (new_width < 300) new_width = 300;
    if (new_width > UI_HOR_RES - X_PADDING) new_width = UI_HOR_RES - X_PADDING;

    lv_obj_set_width(ui_system_message_container, new_width);

    if (system_message_show_expiration)
    {
        lv_obj_set_width(ui_system_message_expiration,
                         new_width - SYSTEM_MESSAGE_BAR_PADDING);
        lv_obj_clear_flag(ui_system_message_expiration, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_set_width(ui_system_message_expiration, 0);
        lv_obj_add_flag(ui_system_message_expiration, LV_OBJ_FLAG_HIDDEN);
    }

    lv_obj_remove_flag(ui_system_message_container, LV_OBJ_FLAG_HIDDEN);
}

void set_system_message(SYSTEM_MESSAGE_ID id, uint32_t duration_ms, bool show_expiration)
{
    if ((id <= SYSTEM_MESSAGE_NONE) || (id >= SYSTEM_MESSAGE_RESERVED))
        return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    pending_system_message.duration_ms = duration_ms;
    pending_system_message.show_expiration = show_expiration;
    pending_system_message.id = id;
    if (primask == 0U)
        __enable_irq();
}

void system_message_service(void)
{
    SYSTEM_MESSAGE_REQUEST request;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    request.id = pending_system_message.id;
    request.duration_ms = pending_system_message.duration_ms;
    request.show_expiration = pending_system_message.show_expiration;
    pending_system_message.id = SYSTEM_MESSAGE_NONE;
    if (primask == 0U)
        __enable_irq();

    if (request.id == SYSTEM_MESSAGE_NONE)
        return;

    const char *msg = system_message_text(request.id);
    if (msg != NULL)
        display_system_message(msg, request.duration_ms,
                               request.show_expiration);
}

void clear_system_message(void)
{
	system_message_duration_ms = 0U;
	system_message_show_expiration = false;
	if (ui_system_message_timer != NULL)
		lv_timer_pause(ui_system_message_timer);

	if (ui_system_message != NULL)
		lv_label_set_text(ui_system_message, "No message set");

	if (ui_system_message_expiration != NULL)
	{
		lv_obj_set_width(ui_system_message_expiration, 0);
		lv_obj_add_flag(ui_system_message_expiration, LV_OBJ_FLAG_HIDDEN);
	}

	if (ui_system_message_container != NULL)
		lv_obj_add_flag(ui_system_message_container, LV_OBJ_FLAG_HIDDEN);
}
