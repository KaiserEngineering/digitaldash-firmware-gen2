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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION "v1.0.0"

// IMAGES AND IMAGE SETS
LV_IMG_DECLARE(ui_img_splash_png);    // assets/splash.png
LV_IMG_DECLARE(ui_img_flare_png);    // assets/Flare.png
LV_IMG_DECLARE(ui_img_gauge200_png);    // assets/gauge200.png
LV_IMG_DECLARE(ui_img_needle200_png);    // assets/needle200.png
LV_IMG_DECLARE(ui_img_1303014416);    // assets/ezgif-frame-038.png
LV_IMG_DECLARE(ui_img_132361813);    // assets/ezgif-frame-039.png
LV_IMG_DECLARE(ui_img_1604144407);    // assets/ezgif-frame-040.png
LV_IMG_DECLARE(ui_img_1255446660);    // assets/ezgif-frame-041.png
LV_IMG_DECLARE(ui_img_527364531);    // assets/ezgif-frame-042.png
LV_IMG_DECLARE(ui_img_1609093710);    // assets/ezgif-frame-043.png
LV_IMG_DECLARE(ui_img_881011581);    // assets/ezgif-frame-044.png
LV_IMG_DECLARE(ui_img_554364648);    // assets/ezgif-frame-045.png
LV_IMG_DECLARE(ui_img_1282446777);    // assets/ezgif-frame-046.png
LV_IMG_DECLARE(ui_img_200717598);    // assets/ezgif-frame-047.png
LV_IMG_DECLARE(ui_img_1303017489);    // assets/ezgif-frame-048.png
LV_IMG_DECLARE(ui_img_132358740);    // assets/ezgif-frame-049.png
LV_IMG_DECLARE(ui_img_1604145430);    // assets/ezgif-frame-050.png
LV_IMG_DECLARE(ui_img_1255445637);    // assets/ezgif-frame-051.png
LV_IMG_DECLARE(ui_img_527363508);    // assets/ezgif-frame-052.png
LV_IMG_DECLARE(ui_img_1609092687);    // assets/ezgif-frame-053.png
LV_IMG_DECLARE(ui_img_881010558);    // assets/ezgif-frame-054.png
LV_IMG_DECLARE(ui_img_554365671);    // assets/ezgif-frame-055.png
LV_IMG_DECLARE(ui_img_1282447800);    // assets/ezgif-frame-056.png
LV_IMG_DECLARE(ui_img_200718621);    // assets/ezgif-frame-057.png
LV_IMG_DECLARE(ui_img_1303016466);    // assets/ezgif-frame-058.png
LV_IMG_DECLARE(ui_img_132359763);    // assets/ezgif-frame-059.png
LV_IMG_DECLARE(ui_img_1604142357);    // assets/ezgif-frame-060.png
LV_IMG_DECLARE(ui_img_1255448710);    // assets/ezgif-frame-061.png
LV_IMG_DECLARE(ui_img_527366581);    // assets/ezgif-frame-062.png
LV_IMG_DECLARE(ui_img_1609095760);    // assets/ezgif-frame-063.png
LV_IMG_DECLARE(ui_img_881013631);    // assets/ezgif-frame-064.png
LV_IMG_DECLARE(ui_img_554362598);    // assets/ezgif-frame-065.png
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MPU_Config(void);
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

static uint8_t screen_active(struct _lv_obj_t * scr)
{
	if( lv_display_get_screen_active(NULL) == scr)
		return 1;
	else
		return 0;
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

PID_DATA iat;
PID_DATA boost;
PID_DATA oil;

digitaldash FordFocusSTRS;
lv_obj_t * ui_view[MAX_VIEWS];

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

  /* MPU Configuration--------------------------------------------------------*/
  //MPU_Config();

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
  /* USER CODE BEGIN 2 */
  lv_init();
  lv_tick_set_cb(HAL_GetTick);
  lvgl_display_init();

  FordFocusSTRS.num_views = 1;

  // View 1
  FordFocusSTRS.view[0].enabled = 1;
  FordFocusSTRS.view[0].view_index = 0;
  FordFocusSTRS.view[0].num_gauges = 3;
  FordFocusSTRS.view[0].background = BACKGROUND_FLARE;

  // View 1 - Gauge 1
  strcpy(iat.label, "IAT");
  strcpy(iat.unit_label, PID_UNITS_FAHRENHEIT_LABEL);
  iat.lower_limit = 30;
  iat.upper_limit = 150;
  FordFocusSTRS.view[0].gauge[0].pid = &iat;
  FordFocusSTRS.view[0].gauge[0].theme = THEME_STOCK_ST;

  // View 1 - Gauge 2
  strcpy(boost.label, "Boost");
  strcpy(boost.unit_label, PID_UNITS_PSI_LABEL);
  boost.lower_limit = -15;
  boost.upper_limit = 25;
  FordFocusSTRS.view[0].gauge[1].pid = &boost;
  FordFocusSTRS.view[0].gauge[1].theme = THEME_STOCK_ST;

  // View 1 - Gauge 3
  strcpy(oil.label, "Oil");
  strcpy(oil.unit_label, PID_UNITS_FAHRENHEIT_LABEL);
  oil.lower_limit = 32;
  oil.upper_limit = 150;
  FordFocusSTRS.view[0].gauge[2].pid = &oil;
  FordFocusSTRS.view[0].gauge[1].theme = THEME_STOCK_ST;


  BSP_HSPI_NOR_Init_t hspi_init;
  hspi_init.InterfaceMode = MX66UW1G45G_OPI_MODE;
  hspi_init.TransferRate = MX66UW1G45G_DTR_TRANSFER;

  if( BSP_HSPI_NOR_Init(0, &hspi_init) != BSP_ERROR_NONE )
  {
	  while(1){}
  }

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

  /* reset display */
  HAL_GPIO_WritePin(LCD_ON_GPIO_Port, LCD_ON_Pin, GPIO_PIN_SET);

  //ui_init();

  // Create the screen
  lv_disp_t * dispp = lv_display_get_default();
  lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
  lv_disp_set_theme(dispp, theme);

  // Add view(s)
  for( uint8_t idx = 0; idx < FordFocusSTRS.num_views; idx++) {
	  ui_view[idx] = lv_obj_create(NULL);
	  lv_obj_remove_flag(ui_view[idx], LV_OBJ_FLAG_SCROLLABLE);      /// Flags
  }

  // Add background(s)
  for( uint8_t idx = 0; idx < FordFocusSTRS.num_views; idx++) {

	  uint8_t is_image = 0;
	  const lv_image_dsc_t * img = NULL;
	  lv_color_t color = {0};

	  switch( FordFocusSTRS.view[idx].background )
	  {
		  case BACKGROUND_FLARE:
			  img = &ui_img_flare_png;
			  is_image = 1;
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

  for( uint8_t idx = 0; idx < FordFocusSTRS.num_views; idx++) {
	  int x_pos[MAX_GAUGES] = {0};

	  if( FordFocusSTRS.view[idx].num_gauges == 1) {
		  x_pos[0] = 0;
	  } else if( FordFocusSTRS.view[idx].num_gauges == 2) {
		  x_pos[0] = -200;
		  x_pos[1] = 200;
	  }if( FordFocusSTRS.view[idx].num_gauges == 3) {
		  x_pos[0] = -250;
		  x_pos[1] = 0;
		  x_pos[2] = 250;
	  }

	  for( uint8_t i = 0; i < FordFocusSTRS.view[idx].num_gauges; i++) {
		  FordFocusSTRS.view[idx].gauge[i].obj = add_stock_st_gauge(x_pos[i], 0, ui_view[0], FordFocusSTRS.view[0].gauge[i].pid);
	  }
  }

  lv_screen_load(ui_view[0]);


  uint32_t delay = HAL_GetTick();

  uint8_t gauge = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	lv_timer_handler();
	if( delay < HAL_GetTick() )
	{
		delay = HAL_GetTick() + 4;
		//lv_slider_set_value(ui_bar, i, LV_ANIM_OFF);
		//i = i + 5;
		iat.pid_value = iat.pid_value + 0.05;
		if( iat.pid_value > 150 )
			iat.pid_value = 30;

		boost.pid_value = boost.pid_value + 0.01;
		if( boost.pid_value > 25 )
			boost.pid_value = -14.6;

		oil.pid_value = oil.pid_value + 0.2;
		if( oil.pid_value > 198 )
			oil.pid_value = 32.5;

		log_minmax(&iat);
		log_minmax(&boost);
		log_minmax(&oil);


		/*
		if( HAL_GetTick() < 2750) {
			switch_screen(ui_splash);
		} else if( boost.pid_value > 10 ){
			switch_screen(ui_view2);
		} else if( boost.pid_value > 0 ) {
			switch_screen(ui_view3);
		} else {
			switch_screen(ui_view1);
		}
		*/


		/*
		if( oil.pid_value > 8000 )
		{
			if( alert_active == 0 ) {
				alert_active = 1;
				_ui_flag_modify(ui_alertContainer, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
			}
		} else {
			if( alert_active == 1 ) {
				alert_active = 0;
				_ui_flag_modify(ui_alertContainer, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
			}
		}
		*/

		switch( gauge )
		{
			case 0:
			//lv_arc_set_value(ui_gauge1, iat);
			if(screen_active(ui_view[0])) {
				lv_obj_send_event(FordFocusSTRS.view[0].gauge[0].obj, LV_EVENT_REFRESH, &iat);
			}
			break;

			case 1:
			//lv_arc_set_value(ui_Gauge2, boost);
			if(screen_active(ui_view[0])) {
				lv_obj_send_event(FordFocusSTRS.view[0].gauge[1].obj, LV_EVENT_REFRESH, &boost);
			}
			break;

			case 2:
			if(screen_active(ui_view[0])) {
				lv_obj_send_event(FordFocusSTRS.view[0].gauge[2].obj, LV_EVENT_REFRESH, &oil);
			}
			break;
		}

		gauge = (gauge >= 2) ? 0 : gauge + 1;
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x08000000;
  MPU_InitStruct.LimitAddress = 0x083FFFFF;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  MPU_AttributesInit.Number = MPU_REGION_NUMBER0;
  MPU_AttributesInit.Attributes = MPU_DEVICE_nGnRnE | MPU_WRITE_THROUGH
                              | MPU_TRANSIENT | MPU_R_ALLOCATE;

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.LimitAddress = 0x202EFFFF;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER1;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RW;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  MPU_AttributesInit.Number = MPU_REGION_NUMBER1;
  MPU_AttributesInit.Attributes = MPU_DEVICE_nGnRnE | MPU_WRITE_BACK
                              | MPU_TRANSIENT | MPU_RW_ALLOCATE;

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0xA0000000;
  MPU_InitStruct.LimitAddress = 0xA7FFFFFF;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER2;
  MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RW;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  MPU_AttributesInit.Number = MPU_REGION_NUMBER2;
  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
