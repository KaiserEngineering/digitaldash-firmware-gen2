/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcache.h"
#include "dma2d.h"
#include "fdcan.h"
#include "flash.h"
#include "gpu2d.h"
#include "gtzc.h"
#include "hspi.h"
#include "i2c.h"
#include "icache.h"
#include "ltdc.h"
#include "memorymap.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ltdc.h"
#include "dma2d.h"
#include "lvgl.h"
#include "demos/lv_demos.h"
#include "lvgl_port_display.h"
#include <string.h>
#include "stm32u5g9j_discovery_hspi.h"
#include "themes.h"
#include "lib_pid.h"
#include "ke_digitaldash.h"
#include "lib_digital_dash.h"
#include "ke_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION "v1.0.0"

static const __attribute__((section(".ExtFlash_Section"))) __attribute__((used)) uint8_t backgrounds_external[1][UI_HOR_RES*UI_VER_RES*3];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_DATA iat;
PID_DATA boost;
PID_DATA oil;
PID_DATA coolant;
PID_DATA speed;

digitaldash FordFocusSTRS;
lv_obj_t * ui_view[MAX_VIEWS];
lv_obj_t * ui_alert[MAX_ALERTS];
lv_obj_t * ui_alert_container[MAX_ALERTS];

volatile uint32_t can_tx_mailbox_status = 0;
volatile uint32_t can_rx_mailbox_status = 0;

uint32_t CAN_Filter_Count = 0;

uint8_t active_view_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void switch_screen(struct _lv_obj_t * scr)
{
	if( lv_display_get_screen_active(NULL) != scr)
	{
		lv_screen_load(scr);
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	 HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	 //HAL_SPI_Receive_IT(&hspi1, rx_buffer, sizeof(rx_buffer));
}

void log_minmax( PID_DATA* pid )
{
	if( pid->pid_value >= pid->pid_max )
		pid->pid_max = pid->pid_value;
	if( pid->pid_value <= pid->pid_min )
		pid->pid_min = pid->pid_value;
}

uint8_t compare_values(float a, float b, digitaldash_compare comparison)
{
	switch (comparison) {
		case DD_LESS_THAN:
			return a < b;
		case DD_LESS_THAN_OR_EQUAL_TO:
			return a <= b;
		case DD_GREATER_THAN:
			return a > b;
		case DD_GREATER_THAN_OR_EQUAL_TO:
			return a >= b;
		case DD_EQUAL:
			return a == b;
		case DD_NOT_EQUAL:
			return a != b;
		default:
			return 0;  // Return false by default if comparison is invalid
	}
}

HAL_StatusTypeDef can_filter( uint32_t id, uint32_t mask, uint32_t filterIndex, uint32_t FIFO  )
{
	HAL_StatusTypeDef status = HAL_OK;

	//TODO check if CAN is enabled
	if( HAL_FDCAN_GetState(&hfdcan1) != HAL_FDCAN_STATE_RESET ) {
		status = HAL_FDCAN_Stop(&hfdcan1);

		// Abort is CAN couldn't be stopped
		if( status != HAL_OK )
			return status;
	}

	/* Declare a CAN filter configuration */
	FDCAN_FilterTypeDef  sFilterConfig;

	/* Verify the filter bank is possible */
	if ( ( filterIndex >= 0 ) && ( filterIndex < 28 ) )
		sFilterConfig.FilterIndex = filterIndex;
	else
		return -1;

	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterID1 = id;
	sFilterConfig.FilterID2 = 0x7FF;
	sFilterConfig.FilterConfig  = FIFO;
	sFilterConfig.FilterIndex = filterIndex;

	status = HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

	// Abort is CAN filter did not work
	if( status != HAL_OK )
		return status;

	return HAL_FDCAN_Start(&hfdcan1);
}

void Add_CAN_Filter( uint16_t id )
{
	can_filter( id, 0x7FF, CAN_Filter_Count++, FDCAN_FILTER_TO_RXFIFO0 );
}

void process_can_packet( FDCAN_HandleTypeDef *hfdcan, uint32_t fifo )
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buf[8];
    HAL_FDCAN_GetRxMessage( hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_buf );

    if( rx_header.IdType == FDCAN_STANDARD_ID ) {
    	DigitalDash_Add_CAN_Packet( rx_header.Identifier ,rx_buf);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    process_can_packet( hfdcan, RxFifo0ITs );
}

void HAL_FDCAN_RxFifo1MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    process_can_packet( hfdcan, RxFifo1ITs );
}

static uint8_t ECU_CAN_Tx( uint8_t data[], uint8_t len )
{

	if( !IS_FDCAN_DLC(len) ) {
		Error_Handler();
	}

	FDCAN_TxHeaderTypeDef Header = {
	           .Identifier          = 0x7E0,
	           .IdType              = FDCAN_STANDARD_ID,
	           .TxFrameType         = FDCAN_DATA_FRAME,
	           .DataLength          = len,
			   .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
			   .BitRateSwitch       = FDCAN_BRS_OFF,
			   .FDFormat            = FDCAN_CLASSIC_CAN,
			   .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
			   .MessageMarker       = 0
	};

    /* Copy the buffer */
    uint8_t tx_buf[8];
    memcpy(tx_buf, data, len);

	/* Start the Transmission process */
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Header, tx_buf);
	//TODO What happens if tx fails?

	return 1;
}

static void LCD_Brightness( uint8_t brightness )
{
	if( brightness > 0 )
		HAL_GPIO_WritePin(BLKT_EN_GPIO_Port, BLKT_EN_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(BLKT_EN_GPIO_Port, BLKT_EN_Pin, GPIO_PIN_RESET);
}

static void esp32_reset( HOST_PWR_STATE state )
{
	if( state == HOST_PWR_ENABLED )
		HAL_GPIO_WritePin(ESP32_RESET_N_GPIO_Port, ESP32_RESET_N_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(ESP32_RESET_N_GPIO_Port, ESP32_RESET_N_Pin, GPIO_PIN_RESET);
}

void Digitaldash_Init( void )
{
    DIGITALDASH_CONFIG config;
    config.dd_ecu_tx            = &ECU_CAN_Tx;
    config.dd_host_ctrl         = &esp32_reset;
    config.dd_set_backlight     = &LCD_Brightness;
    config.dd_filter            = &Add_CAN_Filter;

    if( digitaldash_init( &config ) != DIGITALDASH_INIT_OK )
        Error_Handler();
}

void spoof_config(void)
{
	// View 0
	set_view_enable(0, VIEW_STATE_ENABLED, false);
	set_view_num_gauges(0, 3, false);
	set_view_background(0, VIEW_BACKGROUND_FLARE, false);
	set_view_gauge_theme(0, 0, GAUGE_THEME_STOCK_ST, false);
	set_view_gauge_theme(0, 1, GAUGE_THEME_GRUMPY_CAT, false);
	set_view_gauge_theme(0, 2, GAUGE_THEME_STOCK_ST, false);
	set_view_gauge_pid(0, 0, MODE1_ENGINE_SPEED_UUID, 0);
	set_view_gauge_units(0, 0, PID_UNITS_RPM, 0);
	set_view_gauge_pid(0, 1, MODE1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE_UUID, 0);
	set_view_gauge_units(0, 1, PID_UNITS_PSI, 0);
	set_view_gauge_pid(0, 2, MODE1_ENGINE_COOLANT_TEMPERATURE_UUID, 0);
	set_view_gauge_units(0, 2, PID_UNITS_FAHRENHEIT, 0);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();
  /* GTZC initialisation */
  MX_GTZC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA2D_Init();
  MX_HSPI1_Init();
  MX_I2C2_Init();
  MX_ICACHE_Init();
  MX_LTDC_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_GPU2D_Init();
  MX_FLASH_Init();
  MX_SPI1_Init();
  MX_DCACHE1_Init();
  MX_DCACHE2_Init();
  MX_FDCAN1_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  lv_init();
  lv_tick_set_cb(HAL_GetTick);
  lvgl_display_init();

  // Start 1ms timer tick for Digital Dash
  if (HAL_TIM_Base_Start_IT(&htim17) != HAL_OK)
      Error_Handler();

  // Spoof a config if EEPROM isn't present
  spoof_config();



  // Alert 1
  strcpy(FordFocusSTRS.alert[0].msg, "Max oil pressure reached");
  FordFocusSTRS.alert[0].trigger.pid = &oil;
  FordFocusSTRS.alert[0].trigger.compare = DD_GREATER_THAN;
  FordFocusSTRS.alert[0].trigger.thresh = 160;

  // Dynamic 1
  FordFocusSTRS.dynamic[0].enabled = 1;
  FordFocusSTRS.dynamic[0].priority = DD_MEDIUM_PRIORITY;
  FordFocusSTRS.dynamic[0].trigger.compare = DD_GREATER_THAN;
  FordFocusSTRS.dynamic[0].trigger.pid = &oil;
  FordFocusSTRS.dynamic[0].trigger.thresh = 100;
  FordFocusSTRS.dynamic[0].view_index = 0;

  // Dynamic 2
  FordFocusSTRS.dynamic[1].enabled = 1;
  FordFocusSTRS.dynamic[1].priority = DD_HIGH_PRIORITY;
  FordFocusSTRS.dynamic[1].trigger.compare = DD_GREATER_THAN;
  FordFocusSTRS.dynamic[1].trigger.pid = &oil;
  FordFocusSTRS.dynamic[1].trigger.thresh = 260;
  FordFocusSTRS.dynamic[1].view_index = 1;

  BSP_HSPI_NOR_Init_t hspi_init;
  hspi_init.InterfaceMode = MX25LM51245G_OPI_MODE;
  hspi_init.TransferRate = MX25LM51245G_DTR_TRANSFER;

  if( BSP_HSPI_NOR_Init(0, &hspi_init) != BSP_ERROR_NONE )
  {
	  while(1){}
  }

  /*
  if( BSP_HSPI_NOR_Write(0, ui_img_flare_png.data, (uint32_t)&backgrounds_external[0], sizeof(backgrounds_external[0])) != BSP_ERROR_NONE )
  {
	while(1){}
  }
  */

  if( BSP_HSPI_NOR_EnableMemoryMappedMode(0) != BSP_ERROR_NONE )
  {
	while(1){}
  }


  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2U * 50);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
	  /* PWM Generation Error */
	  Error_Handler();
  }

  //ui_init();

  // Create the screen
  lv_disp_t * dispp = lv_display_get_default();
  lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
  lv_disp_set_theme(dispp, theme);

  // Add background(s)
  for( uint8_t idx = 0; idx < FordFocusSTRS.num_views; idx++) {

	  uint8_t is_image = 0;
	  const lv_image_dsc_t * img = NULL;
	  lv_color_t color = {0};

	  lv_image_dsc_t ext_background = {
	    .header.cf = LV_COLOR_FORMAT_RGB888,
	    .header.magic = LV_IMAGE_HEADER_MAGIC,
	    .header.w = UI_HOR_RES,
	    .header.h = UI_VER_RES,
	    .data_size = sizeof(backgrounds_external[0]),
	    .data = backgrounds_external[0],
	  };

	  switch( FordFocusSTRS.view[idx].background )
	  {
		  case BACKGROUND_FLARE:
			  //img = &ui_img_flare_png;
			  is_image = 1;
			  break;

		  case BACKGROUND_USER1:
			  img = &ext_background;
			  is_image = 1;
			  break;

		  case BACKGROUND_RED:
			  color.red = 255;
			  color.green = 0;
			  color.blue = 0;
			  is_image = 0;
			  break;

		  case BACKGROUND_GREEN:
			  color.red = 0;
			  color.green = 255;
			  color.blue = 0;
			  is_image = 0;
			  break;

		  case BACKGROUND_BLUE:
			  color.red = 0;
			  color.green = 0;
			  color.blue = 255;
			  is_image = 0;
			  break;

		  case BACKGROUND_BLACK:
		  default:
			  color.red = 0;
			  color.green = 0;
			  color.blue = 0;
			  is_image = 0;
			  break;
	  }

	  if( is_image )
		  lv_obj_set_style_bg_image_src(ui_view[idx], img, LV_PART_MAIN | LV_STATE_DEFAULT);
	  else
		  lv_obj_set_style_bg_color(ui_view[idx], color, LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  ui_alert_container[0] = lv_obj_create(ui_view[0]);
  lv_obj_remove_style_all(ui_alert_container[0]);
  lv_obj_set_width(ui_alert_container[0], 450);
  lv_obj_set_height(ui_alert_container[0], 75);
  lv_obj_set_x(ui_alert_container[0], 0);
  lv_obj_set_y(ui_alert_container[0], 5);
  lv_obj_set_align(ui_alert_container[0], LV_ALIGN_TOP_MID);
  //lv_obj_add_flag(ui_alert_container[0], LV_OBJ_FLAG_HIDDEN);     /// Flags
  lv_obj_remove_flag(ui_alert_container[0],
                     LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                     LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                     LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
  lv_obj_set_style_radius(ui_alert_container[0], 10, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(ui_alert_container[0], lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_alert_container[0], 225, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_border_color(ui_alert_container[0], lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_border_opa(ui_alert_container[0], 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_border_width(ui_alert_container[0], 3, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_color(ui_alert_container[0], lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_opa(ui_alert_container[0], 225, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_width(ui_alert_container[0], 20, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_spread(ui_alert_container[0], 5, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_offset_x(ui_alert_container[0], 12, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_offset_y(ui_alert_container[0], 12, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_alert[0] = lv_label_create(ui_alert_container[0]);
  lv_obj_set_width(ui_alert[0], LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(ui_alert[0], LV_SIZE_CONTENT);    /// 1
  lv_obj_set_align(ui_alert[0], LV_ALIGN_CENTER);
  lv_label_set_text(ui_alert[0], FordFocusSTRS.alert[0].msg);
  lv_obj_set_style_text_font(ui_alert[0], &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);

  // Hide the alert by default
  lv_obj_add_flag(ui_alert_container[0], LV_OBJ_FLAG_HIDDEN);

  uint8_t alert_active = 1;

  /* Configure global filter to reject all non-matching frames */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  if( HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0) != HAL_OK ) {
	  Error_Handler();
  }

  /* Activate Interrupts */
  if( HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK ) {
	  Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }

  if( HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0) != HAL_OK ) {
	  Error_Handler();
  }

  /* Activate Interrupts */
  if( HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK ) {
	  Error_Handler();
  }

  Digitaldash_Init();

  // Iterate through each view
  for(uint8_t view = 0; view < MAX_VIEWS; view++)
  {
	  FordFocusSTRS.view[view].enabled = get_view_enable(view);

	  if( FordFocusSTRS.view[view].enabled ) {
		  // Create the view
		  ui_view[view] = lv_obj_create(NULL);
		  lv_obj_remove_flag(ui_view[view], LV_OBJ_FLAG_SCROLLABLE);
		  FordFocusSTRS.num_views++;

		  FordFocusSTRS.view[view].num_gauges = get_view_num_gauges(view);
		  FordFocusSTRS.view[view].background = get_view_background(view);

		  int x_pos[GAUGES_PER_VIEW] = {0};

		  if( FordFocusSTRS.view[view].num_gauges == 1) {
			  x_pos[0] = 0;
		  } else if( FordFocusSTRS.view[view].num_gauges == 2) {
			  x_pos[0] = -200;
			  x_pos[1] = 200;
		  }if( FordFocusSTRS.view[view].num_gauges == 3) {
			  x_pos[0] = -225;
			  x_pos[1] = 0;
			  x_pos[2] = 225;
		  }

		  // Iterate through each gauge in the view
		  for(uint8_t gauge = 0; gauge < FordFocusSTRS.view[view].num_gauges; gauge++)
		  {
			  PID_DATA pid_req;

			  // Get PID universally unique ID, PID, and mode
			  pid_req.pid_uuid = get_view_gauge_pid(view, gauge);
			  pid_req.pid = get_pid_by_uuid(pid_req.pid_uuid);
			  pid_req.mode = get_mode_by_uuid(pid_req.pid_uuid);

			  // Load the unit and default to base unit if error
			  pid_req.pid_unit = get_view_gauge_units(view, gauge);
			  if( pid_req.pid_unit == PID_UNITS_RESERVED )
				  pid_req.pid_unit = get_pid_base_unit(pid_req.pid_uuid);

			  // Load the labels
			  get_pid_label(pid_req.pid_uuid, pid_req.label);
			  get_unit_label(pid_req.pid_unit, pid_req.unit_label);

			  pid_req.lower_limit = get_pid_lower_limit(pid_req.pid_uuid ,pid_req.pid_unit);
			  pid_req.upper_limit = get_pid_upper_limit(pid_req.pid_uuid ,pid_req.pid_unit);
			  pid_req.precision = get_pid_precision(pid_req.pid_uuid ,pid_req.pid_unit);

			  // Start the PID stream and save the pointer
			  FordFocusSTRS.view[view].gauge[gauge].pid = DigitalDash_Add_PID_To_Stream( &pid_req );

			  // Load the gauge theme
			  FordFocusSTRS.view[view].gauge[gauge].theme = get_view_gauge_theme(view, gauge);

			  // Finally, add the gauge to the view
			  FordFocusSTRS.view[view].gauge[gauge].obj = add_gauge(FordFocusSTRS.view[view].gauge[gauge].theme, x_pos[gauge], 0, ui_view[view], FordFocusSTRS.view[view].gauge[gauge].pid);

			  //get_unit_label(PID_UNITS_MPH, FordFocusSTRS.view[view].gauge[gauge].pid->unit_label);
		  }
	  }
  }

  lv_screen_load(ui_view[0]);

  uint32_t timestamp[MAX_VIEWS][GAUGES_PER_VIEW] = {0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	digitaldash_service();
	lv_timer_handler();

	/* Log min/max values */
	for( uint8_t idx = 0; idx < FordFocusSTRS.num_views; idx++) {
		for( uint8_t i = 0; i < FordFocusSTRS.view[idx].num_gauges; i++) {
			log_minmax(FordFocusSTRS.view[idx].gauge[i].pid);
		}
	}

	/* Check for dynamic gauge change */
	if( compare_values(FordFocusSTRS.dynamic[0].trigger.pid->pid_value, FordFocusSTRS.dynamic[0].trigger.thresh, FordFocusSTRS.alert[0].trigger.compare) ) {
		active_view_idx = FordFocusSTRS.dynamic[0].view_index;
	} else {
		active_view_idx = 0;
	}

	if( FordFocusSTRS.view[active_view_idx].enabled )
		switch_screen(ui_view[active_view_idx]);


	/* Check for Alert(s) */
	if( compare_values(FordFocusSTRS.alert[0].trigger.pid->pid_value, FordFocusSTRS.alert[0].trigger.thresh, FordFocusSTRS.alert[0].trigger.compare) )
	{
		if( alert_active == 0 ) {
			alert_active = 1;
			lv_obj_remove_flag(ui_alert_container[0], LV_OBJ_FLAG_HIDDEN);
		}
	} else {
		if( alert_active == 1 ) {
			alert_active = 0;
			lv_obj_add_flag(ui_alert_container[0], LV_OBJ_FLAG_HIDDEN);
		}
	}

	/* Update gauges on current view */
	for( uint8_t i = 0; i < FordFocusSTRS.view[active_view_idx].num_gauges; i++) {
		if( timestamp[active_view_idx][i] != FordFocusSTRS.view[active_view_idx].gauge[i].pid->timestamp ) {
			 timestamp[active_view_idx][i] = FordFocusSTRS.view[active_view_idx].gauge[i].pid->timestamp;
			 lv_obj_send_event(FordFocusSTRS.view[active_view_idx].gauge[i].obj, LV_EVENT_REFRESH, FordFocusSTRS.view[active_view_idx].gauge[i].pid);
		}

	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 8;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/* USER CODE BEGIN 4 */

// Timer interrupt callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM17) {
    	digitaldash_tick();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
