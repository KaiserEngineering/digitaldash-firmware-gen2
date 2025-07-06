/*
 * ui.c
 *
 *  Created on: Jun 22, 2025
 *      Author: Matth
 */

#include "ui.h"
#include "lib_digital_dash.h"
#include "../build_info.h"

LV_IMG_DECLARE(ui_img_ford_performance_logo_png);

#define BACKGROUND_IMAGE_COUNT      (15U)
#define BACKGROUND_BLOCK_SIZE       (0x10000U) // 64KB
#define BACKGROUND_PIXEL_WIDTH      UI_HOR_RES
#define BACKGROUND_PIXEL_HEIGHT     UI_VER_RES
#define BACKGROUND_BYTES_PER_PIXEL  UI_BYTES_PER_PIXEL       // e.g. ARGB8888 = 4 bytes, RGB565 = 2 bytes
#define BACKGROUND_BASE_ADDRESS    (0xA0000000U)

// Macro to perform integer ceiling division
#define CEIL_DIV(x, y)              (((x) + (y) - 1) / (y))

// Raw image size in bytes
#define BACKGROUND_RAW_SIZE         (BACKGROUND_PIXEL_WIDTH * BACKGROUND_PIXEL_HEIGHT * BACKGROUND_BYTES_PER_PIXEL)

// Aligned image size (to next 64KB boundary)
#define BACKGROUND_IMAGE_SIZE       (CEIL_DIV(BACKGROUND_RAW_SIZE, BACKGROUND_BLOCK_SIZE) * BACKGROUND_BLOCK_SIZE)

// Start addresses of backgrounds (computed for BACKGROUND_IMAGE_SIZE)
static const uint32_t USER_BACKGROUND_ADDRESSES[BACKGROUND_IMAGE_COUNT] = {
    BACKGROUND_BASE_ADDRESS,
    BACKGROUND_BASE_ADDRESS + 1 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 2 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 3 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 4 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 5 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 6 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 7 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 8 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 9 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 10 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 11 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 12 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 13 * BACKGROUND_IMAGE_SIZE,
    BACKGROUND_BASE_ADDRESS + 14 * BACKGROUND_IMAGE_SIZE
};

uint32_t get_background_addr(uint8_t idx, ADDR_MEMORYMAPPED_MODE mode)
{
	if( mode )
		return USER_BACKGROUND_ADDRESSES[idx];
	else
		return USER_BACKGROUND_ADDRESSES[idx] - BACKGROUND_BASE_ADDRESS;
}

#define BACKGROUND_COLOR_FORMAT LV_COLOR_FORMAT_NATIVE_WITH_ALPHA

#define DEFINE_BACKGROUND_USER(n) \
const lv_image_dsc_t ui_background_user##n = { \
    .header.w = UI_HOR_RES, \
    .header.h = UI_VER_RES, \
    .data_size = UI_HOR_RES * UI_VER_RES * UI_BYTES_PER_PIXEL, \
    .header.cf = BACKGROUND_COLOR_FORMAT, \
    .header.magic = LV_IMAGE_HEADER_MAGIC, \
    .data = (const uint8_t *)USER_BACKGROUND_ADDRESSES[VIEW_BACKGROUND_USER##n] \
}

DEFINE_BACKGROUND_USER(1);
DEFINE_BACKGROUND_USER(2);
DEFINE_BACKGROUND_USER(3);
DEFINE_BACKGROUND_USER(4);
DEFINE_BACKGROUND_USER(5);
DEFINE_BACKGROUND_USER(6);
DEFINE_BACKGROUND_USER(7);
DEFINE_BACKGROUND_USER(8);
DEFINE_BACKGROUND_USER(9);
DEFINE_BACKGROUND_USER(10);
DEFINE_BACKGROUND_USER(11);
DEFINE_BACKGROUND_USER(12);
DEFINE_BACKGROUND_USER(13);
DEFINE_BACKGROUND_USER(14);
DEFINE_BACKGROUND_USER(15);

static bool is_all_ff(const uint8_t *data, uint32_t size) {
	for (uint32_t i = 0; i < size; i++) {
		if (data[i] != 0xFF) {
			return false;
		}
	}
	return true;
}

static uint32_t crc32_table[256];

void crc32_init_table(void) {
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int j = 0; j < 8; ++j) {
            if (c & 1)
                c = 0xEDB88320L ^ (c >> 1);
            else
                c >>= 1;
        }
        crc32_table[i] = c;
    }
}

uint32_t crc32_update(uint32_t crc, const uint8_t *buf, size_t len) {
    crc ^= 0xFFFFFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = (crc >> 8) ^ crc32_table[(crc ^ buf[i]) & 0xFF];
    }
    return crc ^ 0xFFFFFFFF;
}

uint32_t calc_crc32(uint8_t idx, uint32_t reserved) {
    if (idx >= BACKGROUND_IMAGE_COUNT) return 0;

    // Cast address to data pointer
    const uint8_t *data = (const uint8_t *)USER_BACKGROUND_ADDRESSES[idx];
    uint32_t size = BACKGROUND_RAW_SIZE;

    // Ensure the lookup table is initialized
    crc32_init_table();

    // Use optimized table-based CRC
    return crc32_update(0xFFFFFFFF, data, size);
}

// UI Variables
#define SPLASH_SCREEN_T 5000
#define MIN_TO_MILLI (60 * 1000)
#ifdef DEBUG
#define SCREEN_SAVER_T 10 * MIN_TO_MILLI // 10 min
#else
#define SCREEN_SAVER_T 120 * MIN_TO_MILLI // 120 min
#endif
#define SCREEN_SAVER_DURATION_T 1 * MIN_TO_MILLI // 1 min
uint32_t boot_time = 0;
uint32_t next_screen_saver = SCREEN_SAVER_T;
lv_obj_t * ui_screen;
lv_obj_t * splash_screen;
lv_obj_t * ui_view[MAX_VIEWS];
uint8_t active_view_idx = 0;
uint32_t timestamp[MAX_VIEWS][MAX_GAUGES_PER_VIEW] = {0};
float prev_pid_value[MAX_VIEWS][MAX_GAUGES_PER_VIEW] = {0};

/* Collection of gauges and the associated pid */
lv_obj_t * ui_gauge[MAX_VIEWS][MAX_GAUGES_PER_VIEW] = {0};
PID_DATA * ui_gauge_pid[MAX_VIEWS][MAX_GAUGES_PER_VIEW] = {0};
PID_DATA * ui_dynamic_pid[MAX_DYNAMICS] = {0};
PID_DATA * ui_alert_pid[MAX_ALERTS] = {0};

static uint32_t ui_tick_cnt = 0;

static uint8_t compare_values(float a, float b, ALERT_COMPARISON comparison)
{
	switch (comparison) {
		case ALERT_COMPARISON_LESS_THAN:
			return a < b;
		case ALERT_COMPARISON_LESS_THAN_OR_EQUAL_TO:
			return a <= b;
		case ALERT_COMPARISON_GREATER_THAN:
			return a > b;
		case ALERT_COMPARISON_GREATER_THAN_OR_EQUAL_TO:
			return a >= b;
		case ALERT_COMPARISON_EQUAL:
			return a == b;
		case ALERT_COMPARISON_NOT_EQUAL:
			return a != b;
		default:
			return 0;  // Return false by default if comparison is invalid
	}
}

static void switch_screen(struct _lv_obj_t *scr, uint32_t anim_t)
{
    if (lv_disp_get_scr_act(NULL) != scr)
    {
    	lv_screen_load_anim(scr, LV_SCR_LOAD_ANIM_FADE_IN, anim_t, 0, false);
    }
}

static void log_minmax( PID_DATA* pid )
{
	// Only log min/max if a value has been read
	if( pid->timestamp > 0 ) {
		if( pid->pid_value > pid->pid_max )
			pid->pid_max = pid->pid_value;
		if( pid->pid_value < pid->pid_min )
			pid->pid_min = pid->pid_value;
	}
}

/**
 * @brief Selects the highest priority dynamic gauge that should be displayed.
 *
 * This function iterates through all dynamic gauges, starting from the highest
 * priority level and descending to the lowest. For each priority level, it scans
 * all configured dynamic gauges to find one that:
 *   - Matches the current priority level,
 *   - Is marked as enabled,
 *   - Has a comparison that evaluates as true (e.g., RPM > 4000).
 *
 * The first dynamic gauge that satisfies all three conditions will be selected,
 * and its view index will be returned.
 *
 * If no matching dynamic gauge is found, the currently active view index is returned.
 *
 * @return uint8_t The view index of the highest-priority valid dynamic gauge,
 *                 or the current active view index if none match.
 */
static uint8_t dynamic_gauge_check( void )
{
	// Start from highest priority.
	for( DYNAMIC_PRIORITY priority = DYNAMIC_PRIORITY_HIGH; priority > DYNAMIC_PRIORITY_LOW; priority--)
	{
		// Iterate through each dynamic gauge to find the current priority
		for( uint8_t dynamic = 0; dynamic < MAX_DYNAMICS; dynamic++){
			// Find the matching priority dynamic setting
			if( get_dynamic_priority(dynamic) == priority ) {
				// Once found, see if it is enabled
				if( get_dynamic_enable(dynamic) == DYNAMIC_STATE_ENABLED ) {
					// Verify the pid pointer is not null
					if( ui_dynamic_pid[dynamic] == NULL ) {
						continue;
					// Now check if it should be enabled
					} else if( compare_values( ui_dynamic_pid[dynamic]->pid_value, get_dynamic_threshold(dynamic), get_dynamic_compare(dynamic)) ) {
						return get_dynamic_index(dynamic);
					}
				}
			}
		}
	}
	return active_view_idx;
}

static void switch_view(uint8_t idx)
{
	// Check if the selected view is hidden
	if( lv_obj_has_flag(ui_view[idx], LV_OBJ_FLAG_HIDDEN) )
	{
		// Hide all views except the active
		for( uint8_t i = 0; i < MAX_VIEWS; i++)
		{
			if( ui_view[i] == NULL )
				continue;  // Skip null entries
			else if( i == idx )
				lv_obj_remove_flag(ui_view[i], LV_OBJ_FLAG_HIDDEN);
			else
				lv_obj_add_flag(ui_view[i], LV_OBJ_FLAG_HIDDEN);
		}

	}
}

void show_build_info_overlay(void)
{
	#ifdef DEBUG
		#define BUILD_TYPE "Debug"
	#elif defined(RELEASE)
		#define BUILD_TYPE "Release"
	#else
		#define BUILD_TYPE "Unknown"
	#endif

    // Get the top layer
    lv_obj_t * top_layer = lv_layer_top();

    // Create the label
    lv_obj_t * build_label = lv_label_create(top_layer);

    // Format the build info
    char buf[128];
    snprintf(buf, sizeof(buf), "%s: %s (%s) %s", BUILD_TYPE, BUILD_VERSION, BUILD_COMMIT, BUILD_TIMESTAMP);

    lv_label_set_text(build_label, buf);

    // Optional: Make background transparent
    lv_obj_set_style_text_opa(build_label, LV_OPA_COVER, 0); // Semi-transparent text
    lv_obj_set_style_text_color(build_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(build_label, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Align it to the bottom-right corner (or wherever you prefer)
    lv_obj_align(build_label, LV_ALIGN_BOTTOM_MID, -0, -5);
}


void build_ui(void)
{
	  // Initialize the CRC32 table
	  crc32_init_table();

	  // Create the screen
	  lv_disp_t * dispp = lv_display_get_default();
	  lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
	  lv_disp_set_theme(dispp, theme);

	  // Create the splash screen
	  splash_screen = lv_obj_create(NULL);
	  lv_obj_remove_flag(splash_screen, LV_OBJ_FLAG_SCROLLABLE);
	  lv_obj_set_style_bg_color(splash_screen, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
	  lv_obj_set_style_bg_opa(splash_screen, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

	  lv_obj_t * splash_icon;
	  splash_icon = lv_image_create(splash_screen);
	  lv_image_set_src(splash_icon, &ui_img_ford_performance_logo_png);
	  lv_obj_set_width(splash_icon, LV_SIZE_CONTENT);   /// 1
	  lv_obj_set_height(splash_icon, LV_SIZE_CONTENT);    /// 1
	  lv_obj_set_x(splash_icon, 0);
	  lv_obj_set_y(splash_icon, 50);
	  lv_obj_set_align(splash_icon, LV_ALIGN_TOP_MID);
	  lv_obj_remove_flag(splash_icon, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

	  // Create the base screen
	  ui_screen = lv_obj_create(NULL);
	  lv_obj_remove_flag(ui_screen, LV_OBJ_FLAG_SCROLLABLE);

	  // Iterate through each view
	  for(uint8_t view = 0; view < MAX_VIEWS; view++)
	  {
		  if( get_view_enable(view) == VIEW_STATE_ENABLED ) {
			  // Create the view as a container
			  ui_view[view] = lv_obj_create(ui_screen);
			  lv_obj_remove_style_all(ui_view[view]);
			  lv_obj_set_width(ui_view[view], lv_pct(100));
			  lv_obj_set_height(ui_view[view], lv_pct(100));
			  lv_obj_set_align(ui_view[view], LV_ALIGN_CENTER);
			  lv_obj_add_flag(ui_view[view], LV_OBJ_FLAG_HIDDEN);
			  lv_obj_remove_flag(ui_view[view], LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE |
								   LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
								   LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);

			  uint8_t is_image = 0;
			  const lv_image_dsc_t * img = NULL;
			  lv_color_t color = {0};

			  switch( get_view_background(view) )
			  {

				  case VIEW_BACKGROUND_USER1:
					  img = &ui_background_user1;
					  is_image = 1;
					  break;

				  case VIEW_BACKGROUND_USER2:
					  img = &ui_background_user2;
					  is_image = 1;
					  break;

				  default:
					  color.red = 0;
					  color.green = 0;
					  color.blue = 0;
					  is_image = 0;
					  break;
			  }

			  if( is_image ) {
				  if( is_all_ff(img->data, img->data_size) )
				  {
					  lv_obj_set_style_bg_opa(ui_view[view], LV_OPA_COVER, LV_PART_MAIN);
					  lv_obj_set_style_bg_color(ui_view[view], color, LV_PART_MAIN | LV_STATE_DEFAULT);
				  } else {
					  lv_obj_set_style_bg_image_src(ui_view[view], img, LV_PART_MAIN | LV_STATE_DEFAULT);
				  }
			  } else {
				  lv_obj_set_style_bg_opa(ui_view[view], LV_OPA_COVER, LV_PART_MAIN);
				  lv_obj_set_style_bg_color(ui_view[view], color, LV_PART_MAIN | LV_STATE_DEFAULT);
			  }

			  int x_pos[MAX_GAUGES_PER_VIEW] = {0};

			  uint8_t num_gauges = get_view_num_gauges(view);
			  switch( num_gauges )
			  {
			  	  case 1:
			  		  x_pos[0] = 0;
			  		  break;
			  	  case 2:
					  x_pos[0] = -200;
					  x_pos[1] = 200;
					  break;
			  	  case 3:
			  	  default:
					  x_pos[0] = -300;
					  x_pos[1] = 0;
					  x_pos[2] = 300;
					  break;

			  }

			  // Iterate through each gauge in the view
			  for(uint8_t gauge = 0; gauge < num_gauges; gauge++)
			  {
				  PID_DATA pid_req;

				  // Get PID universally unique ID, PID, and mode
				  pid_req.pid_uuid = get_view_gauge_pid(view, gauge);

				  // Load the unit and default to base unit if error
				  pid_req.pid_unit = get_view_gauge_units(view, gauge);
				  if( pid_req.pid_unit == PID_UNITS_RESERVED )
					  pid_req.pid_unit = get_pid_base_unit(pid_req.pid_uuid);

				  // Start the PID stream and save the pointer
				  ui_gauge_pid[view][gauge] = DigitalDash_Add_PID_To_Stream( &pid_req );

				  // Finally, add the gauge to the view
				  ui_gauge[view][gauge] = add_gauge(get_view_gauge_theme(view, gauge), x_pos[gauge], 0, ui_view[view], ui_gauge_pid[view][gauge]);
			  }
		  }
	  }

	  for(uint8_t idx = 0; idx < MAX_DYNAMICS; idx++)
	  {
		  if( get_dynamic_enable(idx) == DYNAMIC_STATE_ENABLED ) {

			  PID_DATA pid_req;

			  // Get PID universally unique ID, PID, and mode
			  pid_req.pid_uuid = get_dynamic_pid(idx);

			  // Load the unit and default to base unit if error
			  pid_req.pid_unit = get_dynamic_units(idx);
			  if( pid_req.pid_unit == PID_UNITS_RESERVED )
				  pid_req.pid_unit = get_pid_base_unit(pid_req.pid_uuid);

			  // Start the PID stream and save the pointer
			  ui_dynamic_pid[idx] = DigitalDash_Add_PID_To_Stream( &pid_req );
		  }
	  }

	  for(uint8_t idx = 0; idx < MAX_ALERTS; idx++)
	  {
		  if( get_alert_enable(idx) == ALERT_STATE_ENABLED )
		  {
			  PID_DATA pid_req;

			  pid_req.pid_uuid = get_alert_pid(idx);

			  // Load the unit and default to base unit if error
			  pid_req.pid_unit = get_alert_units(idx);
			  if( pid_req.pid_unit == PID_UNITS_RESERVED )
				  pid_req.pid_unit = get_pid_base_unit(pid_req.pid_uuid);

			  // Start the PID stream and save the pointer
			  ui_alert_pid[idx] = DigitalDash_Add_PID_To_Stream( &pid_req );
		  }
	  }

	  // Load the splash screen
	  switch_screen(splash_screen, SCREEN_FADE_INIT_T);

	  add_alert(ui_screen);
	  show_build_info_overlay();
}

void ui_service(void)
{
	lv_timer_handler();

	if( ui_tick_cnt <= SPLASH_SCREEN_T ) {
		switch_screen(splash_screen, SCREEN_FADE_T);
	} else if( ui_tick_cnt >= next_screen_saver ) {
		switch_screen(splash_screen, SCREEN_FADE_T);
		if( ui_tick_cnt >= (next_screen_saver + SCREEN_SAVER_DURATION_T) )
			next_screen_saver = ui_tick_cnt + SCREEN_SAVER_T;
	} else {
		switch_screen(ui_screen, SCREEN_FADE_T);
	}

	/* Log min/max values */
	for( uint8_t view = 0; view < MAX_VIEWS; view++) {
		if( get_view_enable(view) == VIEW_STATE_ENABLED ) {
			for( uint8_t gauge = 0; gauge < get_view_num_gauges(view); gauge++) {
				log_minmax(ui_gauge_pid[view][gauge]);
			}
		}
	}

	/* Check for dynamic gauge change */
	active_view_idx = dynamic_gauge_check();

	/* Switch to the active view, this can be called each loop. A check will
	 * be made to ensure that the screen is only re-loaded if it is not active. */
	if( get_view_enable(active_view_idx) == VIEW_STATE_ENABLED )
		switch_view(active_view_idx);

	/* Parse through each alert and check if it needs to be activated */
	for(uint8_t idx = 0; idx < MAX_ALERTS; idx++)
	{
		if( get_alert_enable(idx) == ALERT_STATE_DISABLED ) {
			// Skip if not enabled
		} else if( compare_values(ui_alert_pid[idx]->pid_value, get_alert_threshold(idx), get_alert_compare(idx) ) ) {
			if(get_alert()) {
				char msg[ALERT_MESSAGE_LEN] = {0};
				get_alert_message(idx, msg);
				set_alert(msg);
			}
		} else {
			clear_alert();
		}
	}

	/* Update gauges on current view */
	for( uint8_t i = 0; i < get_view_num_gauges(active_view_idx); i++)
	{
		// Check if new pid data has been received.
		if( timestamp[active_view_idx][i] != ui_gauge_pid[active_view_idx][i]->timestamp )
		{
			// Log the timestamp
			timestamp[active_view_idx][i] = ui_gauge_pid[active_view_idx][i]->timestamp;

			// Check if the value has changed
			if( prev_pid_value[active_view_idx][i] != ui_gauge_pid[active_view_idx][i]->pid_value )
			{
				// Log the value
				prev_pid_value[active_view_idx][i] = ui_gauge_pid[active_view_idx][i]->pid_value;

				// Some values are interrupt driven, log the min/max incase they were missed in the main loop
				log_minmax(ui_gauge_pid[active_view_idx][i]);

				// Send an event to the gauge
				lv_obj_send_event(ui_gauge[active_view_idx][i], LV_EVENT_REFRESH, ui_gauge_pid[active_view_idx][i]);
			}
		}
	}
}

void ui_tick( void )
{
	ui_tick_cnt++;
}
