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

// Define background address macro
#define BG_ADDR(n) (BACKGROUND_BASE_ADDRESS + ((n) * BACKGROUND_IMAGE_SIZE))

// Start addresses of backgrounds (computed for BACKGROUND_IMAGE_SIZE)
static const uint32_t USER_BACKGROUND_ADDRESSES[BACKGROUND_IMAGE_COUNT] = {
    BG_ADDR(0), BG_ADDR(1), BG_ADDR(2), BG_ADDR(3), BG_ADDR(4),
    BG_ADDR(5), BG_ADDR(6), BG_ADDR(7), BG_ADDR(8), BG_ADDR(9),
    BG_ADDR(10), BG_ADDR(11), BG_ADDR(12), BG_ADDR(13), BG_ADDR(14)
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

/**
 * @brief Check if a memory block is completely erased (all bytes are 0xFF).
 *
 * This utility function scans a block of memory and returns `true` if all bytes
 * are equal to `0xFF`, which typically indicates an erased flash memory region.
 *
 * @param data Pointer to the memory block to check.
 * @param size Number of bytes to check.
 * @return true if all bytes are 0xFF, false otherwise.
 */
static bool is_all_ff(const uint8_t *data, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++) {
        if (data[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

static uint32_t crc32_table[256];

/**
 * @brief Initialize the CRC32 lookup table.
 *
 * This function precomputes the CRC32 lookup table used for efficient CRC
 * calculations. It populates the global `crc32_table` array with the
 * polynomial-remainder values for all 256 possible byte values.
 *
 * The polynomial used is 0xEDB88320 (standard CRC32).
 */
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

/**
 * @brief Compute CRC32 of a buffer using the precomputed lookup table.
 *
 * Updates the CRC value by processing a buffer of bytes using the
 * precomputed `crc32_table`. This function assumes the table is already
 * initialized by `crc32_init_table()`.
 *
 * @param crc Initial CRC value (typically 0xFFFFFFFF).
 * @param buf Pointer to the input buffer.
 * @param len Length of the buffer in bytes.
 * @return Updated CRC32 value after processing the buffer.
 */
static uint32_t crc32_update(uint32_t crc, const uint8_t *buf, size_t len) {
    crc ^= 0xFFFFFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = (crc >> 8) ^ crc32_table[(crc ^ buf[i]) & 0xFF];
    }
    return crc ^ 0xFFFFFFFF;
}

/**
 * @brief Calculate the CRC32 checksum of a background image by index.
 *
 * Calculates the CRC32 checksum of the background image data stored at
 * `USER_BACKGROUND_ADDRESSES[idx]`. The size of the data is fixed as
 * `BACKGROUND_RAW_SIZE`. The CRC table is initialized automatically on each call.
 *
 * @param idx Index of the background image to checksum (must be < BACKGROUND_IMAGE_COUNT).
 * @param reserved Reserved parameter, currently unused.
 * @return CRC32 checksum of the image data, or 0 if index is invalid.
 */
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
uint32_t splash_screen_t = 0;
uint32_t boot_time = 0;
uint32_t next_screen_saver = SCREEN_SAVER_T;
lv_obj_t * ui_screen;
lv_obj_t * splash_screen;
lv_obj_t * ui_view[MAX_VIEWS];
uint8_t active_view_idx = 0;

/* Collection of gauges and the associated pid */
lv_obj_t * ui_gauge[MAX_VIEWS][MAX_GAUGES_PER_VIEW] = {0};
GAUGE_DATA ui_gauge_data[MAX_VIEWS][MAX_GAUGES_PER_VIEW];
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

/**
 * @brief Switch to the specified screen with a fade-in animation.
 *
 * If the given screen is not currently active, this function performs
 * a screen transition using a fade-in animation for the specified duration.
 *
 * @param scr     Pointer to the target screen object.
 * @param anim_ms Duration of the fade-in animation in milliseconds. Use 0 for immediate switch.
 */
static void switch_screen(lv_obj_t *scr, uint32_t anim_ms)
{
    if (scr == NULL || lv_disp_get_scr_act(NULL) == scr)
        return;

    lv_screen_load_anim(scr, LV_SCR_LOAD_ANIM_FADE_IN, anim_ms, 0, false);
}

void skip_splash(void)
{
	splash_screen_t = 0;
	switch_screen(ui_screen, 0);
}

static void log_minmax( PID_DATA* pid )
{
	if( pid == NULL )
		return;

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
 * If no matching dynamic gauge is found, the lowest priority (default) dynamic
 * gauge view index is returned.
 *
 * On error the currently active view index is returned.
 *
 * @return uint8_t The view index of the highest-priority valid dynamic gauge,
 *                 or the current active view index if none match.
 */
static uint8_t dynamic_gauge_check(void)
{
    // Iterate from highest to lowest priority
    for (DYNAMIC_PRIORITY priority = DYNAMIC_PRIORITY_HIGH; priority > DYNAMIC_PRIORITY_LOW; priority--)
    {
        for (uint8_t i = 0; i < MAX_DYNAMICS; i++)
        {
        	if (get_dynamic_priority(i) == DYNAMIC_PRIORITY_LOW)
        		return get_dynamic_view_index(i);

            if (get_dynamic_priority(i) != priority)
                continue;

            if (get_dynamic_enable(i) != DYNAMIC_STATE_ENABLED)
                continue;

            const PID_DATA *pid = ui_dynamic_pid[i];
            if (pid == NULL)
                continue;

            if (compare_values(pid->pid_value, get_dynamic_threshold(i), get_dynamic_compare(i)))
                return get_dynamic_view_index(i);
        }
    }

    return active_view_idx;
}

/**
 * @brief Switch the currently visible view to the specified index.
 *
 * This function checks if the view at the given index is already visible.
 * If it is hidden, it hides all other views and makes the selected view visible.
 *
 * Views are assumed to be stored in the `ui_view[]` array. Views that are `NULL`
 * are skipped safely. The visibility is controlled using the `LV_OBJ_FLAG_HIDDEN` flag.
 *
 * @param idx Index of the view to activate.
 */
static void switch_view(uint8_t idx)
{
    if (!lv_obj_has_flag(ui_view[idx], LV_OBJ_FLAG_HIDDEN))
        return;  // View is already active

    for (uint8_t i = 0; i < MAX_VIEWS; i++) {
        if (ui_view[i] == NULL)
            continue;

        if (i == idx)
        	lv_obj_remove_flag(ui_view[i], LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(ui_view[i], LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * @brief Display a build information overlay on the top LVGL layer.
 *
 * This function creates a label on the top-most screen layer showing the current
 * build type (Debug, Release, or Unknown), version, commit hash, and build timestamp.
 * It's typically used for developer diagnostics or QA purposes.
 *
 * The overlay appears semi-transparent in white text and is aligned to the bottom center
 * of the screen. It uses the `lv_font_montserrat_16` font.
 */
void show_build_info_overlay(void)
{
    // Determine build type string
#if defined(DEBUG)
    #define BUILD_TYPE "Debug"
#elif defined(RELEASE)
    #define BUILD_TYPE "Release"
#else
    #define BUILD_TYPE "Unknown"
#endif

    // Create label on the top-most LVGL layer
    lv_obj_t *top_layer = lv_layer_top();
    lv_obj_t *build_label = lv_label_create(top_layer);

    // Format build info text
    char buf[128];
    snprintf(buf, sizeof(buf), "%s: %s (%s) %s", BUILD_TYPE, BUILD_VERSION, BUILD_COMMIT, BUILD_TIMESTAMP);
    lv_label_set_text(build_label, buf);

    // Style the label
    lv_obj_set_style_text_color(build_label, lv_color_white(), 0);
    lv_obj_set_style_text_opa(build_label, LV_OPA_COVER, 0);
    lv_obj_set_style_text_font(build_label, &lv_font_montserrat_18, 0);

    // Align to bottom center with small vertical padding
    lv_obj_align(build_label, LV_ALIGN_BOTTOM_MID, 0 + X_OFFSET, -5);
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
	  lv_obj_set_x(splash_icon, 0 + X_OFFSET);
	  lv_obj_set_y(splash_icon, 45);
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

			  uint8_t is_image = 1;
			  const lv_image_dsc_t * img = NULL;
			  lv_color_t color = {0};

			  switch( get_view_background(view) )
			  {
					case VIEW_BACKGROUND_USER1:  img = &ui_background_user1;  break;
					case VIEW_BACKGROUND_USER2:  img = &ui_background_user2;  break;
					case VIEW_BACKGROUND_USER3:  img = &ui_background_user3;  break;
					case VIEW_BACKGROUND_USER4:  img = &ui_background_user4;  break;
					case VIEW_BACKGROUND_USER5:  img = &ui_background_user5;  break;
					case VIEW_BACKGROUND_USER6:  img = &ui_background_user6;  break;
					case VIEW_BACKGROUND_USER7:  img = &ui_background_user7;  break;
					case VIEW_BACKGROUND_USER8:  img = &ui_background_user8;  break;
					case VIEW_BACKGROUND_USER9:  img = &ui_background_user9;  break;
					case VIEW_BACKGROUND_USER10: img = &ui_background_user10; break;
					case VIEW_BACKGROUND_USER11: img = &ui_background_user11; break;
					case VIEW_BACKGROUND_USER12: img = &ui_background_user12; break;
					case VIEW_BACKGROUND_USER13: img = &ui_background_user13; break;
					case VIEW_BACKGROUND_USER14: img = &ui_background_user14; break;
					case VIEW_BACKGROUND_USER15: img = &ui_background_user15; break;
					default: is_image = 0; break; // Default to black
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
			  int width = 0;
			  int height = Y_HEIGHT;

			  uint8_t num_gauges = get_view_num_gauges(view);
			  if( num_gauges > 1 )
				  width = (UI_HOR_RES - X_PADDING - (GAUGE_PADDING*(num_gauges-1)))/num_gauges;
			  else
				  width = UI_HOR_RES - X_PADDING;
			  switch( num_gauges )
			  {
			  	  case 1:
			  		  x_pos[0] = 0 + X_OFFSET;
			  		  break;
			  	  case 2:
					  x_pos[0] = ((-1*width) + X_OFFSET - GAUGE_PADDING)/2;
					  x_pos[1] = (width + X_OFFSET + GAUGE_PADDING)/2;
					  break;
			  	  case 3:
			  	  default:
					  x_pos[0] = (-1*width) + X_OFFSET - GAUGE_PADDING;
					  x_pos[1] = 0 + X_OFFSET;
					  x_pos[2] = width + X_OFFSET + GAUGE_PADDING;
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
				  ui_gauge_data[view][gauge].pid = DigitalDash_Add_PID_To_Stream( &pid_req );

				  // Finally, add the gauge to the view
				  ui_gauge[view][gauge] = add_gauge(get_view_gauge_theme(view, gauge), x_pos[gauge], 0, width, height, ui_view[view], &ui_gauge_data[view][gauge]);
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

	  splash_screen_t = ui_tick_cnt + SPLASH_SCREEN_T;
}

/**
 * @brief Main UI service loop.
 *
 * This function should be called periodically (e.g., in the main loop or from a timer).
 * It performs several tasks related to UI operation:
 *
 * - Handles the LVGL timer to keep the UI responsive.
 * - Manages screen switching logic between splash screen, UI screen, and screen saver.
 * - Logs min/max values for each gauge in enabled views.
 * - Checks and switches to the active view if necessary.
 * - Evaluates and triggers alerts based on threshold conditions.
 * - Updates gauges on the current active view when new data is received.
 *
 * It is designed to be lightweight and responsive, ensuring the UI remains up-to-date
 * with minimal CPU overhead.
 */
void ui_service(void)
{
	lv_timer_handler();

	// Determine which screen to show based on elapsed UI time
	if (ui_tick_cnt <= splash_screen_t) {
	    // If we're still within the splash screen duration, keep showing the splash screen
	    switch_screen(splash_screen, SCREEN_FADE_T);

	} else if (ui_tick_cnt >= next_screen_saver) {
	    // If it's time to activate the screen saver, switch to splash screen
	    switch_screen(splash_screen, SCREEN_FADE_T);

	    // If the screen saver duration has passed, schedule the next screen saver
	    if (ui_tick_cnt >= (next_screen_saver + SCREEN_SAVER_DURATION_T)) {
	        next_screen_saver = ui_tick_cnt + SCREEN_SAVER_T;
	    }

	} else {
	    // Otherwise, show the regular UI screen
	    switch_screen(ui_screen, SCREEN_FADE_T);
	}

	// Log minimum and maximum values for all enabled views
	for (uint8_t view = 0; view < MAX_VIEWS; view++) {
	    // Check if the current view is enabled
	    if (get_view_enable(view) == VIEW_STATE_ENABLED) {
	        // Loop through all gauges in the enabled view
	        for (uint8_t gauge = 0; gauge < get_view_num_gauges(view); gauge++) {
	            // Log the min/max values for the PID associated with this gauge
	            log_minmax(ui_gauge_data[view][gauge].pid);
	        }
	    }
	}

	// Check for dynamic gauge change
	active_view_idx = dynamic_gauge_check();

	// Switch to the active view, this can be called each loop. A check will
	// be made to ensure that the screen is only re-loaded if it is not active.
	if( get_view_enable(active_view_idx) == VIEW_STATE_ENABLED )
		switch_view(active_view_idx);

	// Parse through each alert and check if it needs to be activated
	for(uint8_t idx = 0; idx < MAX_ALERTS; idx++)
	{
		if( (get_alert_enable(idx) == ALERT_STATE_DISABLED) || (ui_alert_pid[idx] == NULL) ) {
			// Skip if not enabled or if NULL
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

	// Update gauges on current view
	for( uint8_t i = 0; i < get_view_num_gauges(active_view_idx); i++)
	{
		// Check if new pid data has been received.
		if( ui_gauge_data[active_view_idx][i].timestamp != ui_gauge_data[active_view_idx][i].pid->timestamp )
		{
			// Log the timestamp
			ui_gauge_data[active_view_idx][i].timestamp = ui_gauge_data[active_view_idx][i].pid->timestamp;

			// Check if the value has changed
			if( ui_gauge_data[active_view_idx][i].pid_value != ui_gauge_data[active_view_idx][i].pid->pid_value )
			{
				// Some values are interrupt driven, log the min/max incase they were missed in the main loop
				log_minmax(ui_gauge_data[active_view_idx][i].pid);

				// Send an event to the gauge
				lv_obj_send_event(ui_gauge[active_view_idx][i], LV_EVENT_REFRESH, &ui_gauge_data[active_view_idx][i]);

				// Log the value
				ui_gauge_data[active_view_idx][i].pid_value = ui_gauge_data[active_view_idx][i].pid->pid_value;
			}
		}
	}
}

/**
 * @brief Increments the UI tick counter.
 *
 * This function is called every 1ms to increment the `ui_tick_cnt`
 * variable, which can be used to track elapsed time or trigger
 * periodic UI updates.
 */
void ui_tick( void )
{
	ui_tick_cnt++;
}
