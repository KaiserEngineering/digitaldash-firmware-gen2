/*
 * sim_main.c (simulator)
 *
 * Runs the firmware UI (ui/) on LVGL's software renderer inside WebAssembly.
 * JavaScript owns the clock: it calls sim_step() once per animation frame
 * and copies sim_framebuffer() to a canvas whenever sim_frame_dirty() is set.
 *
 * The framebuffer is XRGB8888 in memory order B, G, R, X.
 */
#include <string.h>
#include <emscripten/emscripten.h>
#include "lvgl.h"
#include "ui.h"
#include "lib_digital_dash.h"
#include "cjson_shared.h"
#include "lib_obdii_default_json.h"
#include "lib_CAN_bus_sniffer_default_json.h"
#include "config_json_example.h"

uint8_t sim_ospi_flash[BACKGROUND_IMAGE_COUNT * BACKGROUND_IMAGE_SIZE];

static uint8_t framebuffer[UI_HOR_RES * UI_VER_RES * 4];
static uint8_t cjson_buffer[CJSON_BUFFER_SIZE];
static uint8_t eeprom[UINT16_MAX + 1];
static uint32_t sim_time_ms = 0;
static bool frame_dirty = false;

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
	(void)area; (void)px_map;
	frame_dirty = true;
	lv_display_flush_ready(disp);
}

static uint32_t tick_cb(void) { return sim_time_ms; }

static void eeprom_write(uint16_t addr, uint8_t data) { eeprom[addr] = data; }
static uint8_t eeprom_read(uint16_t addr) { return eeprom[addr]; }

static void register_pid_json(const char *json)
{
	if( !cjson_shared_acquire() )
		return;

	cJSON *root = cJSON_Parse(json);
	if( root != NULL )
	{
		cJSON *entry;
		cJSON_ArrayForEach(entry, root)
			(void)pid_metadata_register_json(entry, 0U);
		cJSON_Delete(root);
	}
	cjson_shared_release();
}

EMSCRIPTEN_KEEPALIVE
void sim_init(void)
{
	memset(sim_ospi_flash, 0xFF, sizeof(sim_ospi_flash));
	memset(eeprom, 0xFF, sizeof(eeprom));

	cjson_shared_init();
	cjson_shared_set_buffer(cjson_buffer, sizeof(cjson_buffer));

	settings_setWriteHandler(&eeprom_write);
	settings_setReadHandler(&eeprom_read);

	pid_metadata_clear_all();
	/* The sniffer reuses OBDII UUIDs with different descriptions; register it
	 * first so the OBDII names that configs refer to win. */
	register_pid_json(default_sniffer_json);
	register_pid_json(default_obdii_json);

	(void)json_to_config(default_config_json);

	lv_init();
	lv_tick_set_cb(tick_cb);

	lv_display_t *disp = lv_display_create(UI_HOR_RES, UI_VER_RES);
	lv_display_set_color_format(disp, LV_COLOR_FORMAT_XRGB8888);
	lv_display_set_buffers(disp, framebuffer, NULL, sizeof(framebuffer), LV_DISPLAY_RENDER_MODE_DIRECT);
	lv_display_set_flush_cb(disp, flush_cb);

	build_ui();
	skip_splash();
}

/* Advance the firmware clock by elapsed_ms and run one UI service pass. */
EMSCRIPTEN_KEEPALIVE
void sim_step(uint32_t elapsed_ms)
{
	for( uint32_t i = 0; i < elapsed_ms; i++ )
		ui_tick();
	sim_time_ms += elapsed_ms;
	ui_service();
}

EMSCRIPTEN_KEEPALIVE
uint8_t *sim_framebuffer(void) { return framebuffer; }

EMSCRIPTEN_KEEPALIVE
int sim_width(void) { return UI_HOR_RES; }

EMSCRIPTEN_KEEPALIVE
int sim_height(void) { return UI_VER_RES; }

/* Returns true once per rendered frame. */
EMSCRIPTEN_KEEPALIVE
bool sim_frame_dirty(void)
{
	bool dirty = frame_dirty;
	frame_dirty = false;
	return dirty;
}

/* Apply a full device config JSON (same format as the webapp sends) and
 * rebuild the UI on the next sim_step(). Returns false if parsing failed. */
EMSCRIPTEN_KEEPALIVE
bool sim_load_config(const char *json)
{
	bool ok = json_to_config(json);
	ui_request_rebuild();
	return ok;
}

/* Replace the PID metadata table with the webapp's /api/pids list
 * ({desc, label, units, min, max, decimals}). Units are the display names
 * ("Celsius", "rpm"), and each entry gets a synthetic UUID by position. */
EMSCRIPTEN_KEEPALIVE
uint32_t sim_load_pids(const char *json)
{
	uint32_t count = 0;

	if( !cjson_shared_acquire() )
		return 0;

	cJSON *root = cJSON_Parse(json);
	pid_metadata_clear_all();

	cJSON *entry;
	cJSON_ArrayForEach(entry, root)
	{
		PID_METADATA metadata;
		memset(&metadata, 0, sizeof(metadata));
		metadata.pid_uuid = PID_UUID(SNIFF, count + 1U);

		cJSON *label = cJSON_GetObjectItem(entry, "label");
		cJSON *desc = cJSON_GetObjectItem(entry, "desc");
		if( cJSON_IsString(label) )
			strncpy(metadata.label, label->valuestring, sizeof(metadata.label) - 1U);
		if( cJSON_IsString(desc) )
			strncpy(metadata.desc, desc->valuestring, sizeof(metadata.desc) - 1U);

		cJSON *units = cJSON_GetObjectItem(entry, "units");
		cJSON *min = cJSON_GetObjectItem(entry, "min");
		cJSON *max = cJSON_GetObjectItem(entry, "max");
		cJSON *decimals = cJSON_GetObjectItem(entry, "decimals");
		cJSON *unit;
		cJSON_ArrayForEach(unit, units)
		{
			uint8_t i = metadata.num_supported_units;
			if( (i >= PID_MAX_SUPPORTED_UNITS) || !cJSON_IsString(unit) )
				break;
			metadata.supported_units[i] = get_unit_by_string(unit->valuestring);
			metadata.lower_limit[i] = (float)cJSON_GetNumberValue(cJSON_GetArrayItem(min, i));
			metadata.upper_limit[i] = (float)cJSON_GetNumberValue(cJSON_GetArrayItem(max, i));
			metadata.precision[i] = (uint8_t)cJSON_GetNumberValue(cJSON_GetArrayItem(decimals, i));
			metadata.num_supported_units++;
		}
		metadata.base_unit = metadata.supported_units[0];

		if( pid_metadata_register(&metadata) == PID_METADATA_OK )
			count++;
	}

	cJSON_Delete(root);
	cjson_shared_release();
	ui_request_rebuild();
	return count;
}

EMSCRIPTEN_KEEPALIVE
uint32_t sim_pid_by_name(const char *name) { return get_pid_by_string(name); }

/* Set a PID value in its base unit, as the vehicle would report it. */
EMSCRIPTEN_KEEPALIVE
void sim_set_pid(uint32_t pid_uuid, float value)
{
	sim_stream_set_value(pid_uuid, value, sim_time_ms == 0 ? 1U : sim_time_ms);
}

/* Copy raw background pixels (UI_HOR_RES x UI_VER_RES, B G R A) into a user
 * background slot (1-10), as the device would after an upload. */
EMSCRIPTEN_KEEPALIVE
bool sim_set_background(uint8_t slot, const uint8_t *bgra, uint32_t len)
{
	if( (slot < 1) || (slot > 10) || (len != BACKGROUND_RAW_SIZE) )
		return false;
	memcpy(sim_ospi_flash + ((VIEW_BACKGROUND_USER1 + slot - 1) * BACKGROUND_IMAGE_SIZE), bgra, len);
	ui_request_rebuild();
	return true;
}

EMSCRIPTEN_KEEPALIVE
uint32_t sim_pid_count(void) { return get_pid_list_size(); }
