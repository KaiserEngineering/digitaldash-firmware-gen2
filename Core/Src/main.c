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
#include "ui.h"
#include "demos/lv_demos.h"
#include "lvgl_port_display.h"
#include <string.h>
#include "stm32u5g9j_discovery_hspi.h"
#include "themes.h"
#include "lib_pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION "v1.0.0"
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

static void add_data(int32_t val)
{
	lv_chart_set_next_value(ui_Chart1,lv_chart_get_series_next(ui_Chart1, NULL), val);

    lv_chart_refresh(ui_Chart1);
}

PID_DATA iat;

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

  ui_init();

  //HAL_Delay(1000);

  //_ui_screen_change(&ui_View1, LV_SCR_LOAD_ANIM_FADE_IN, 500, 0, NULL);
  //lv_screen_load(ui_View1);
  uint8_t i = 0;
  uint32_t delay = HAL_GetTick();

	float boost = -10.2;
	float oil = 65.6;
	float slider = 50;
	uint8_t gauge = 0;
	uint8_t alert_active = 0;

	lv_obj_t * needle = NULL;

	lv_label_set_text(ui_version, FIRMWARE_VERSION);

	//_ui_flag_modify(ui_alertContainer, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);

	//lv_obj_add_event_cb(btn, event_cb, LV_EVENT_ALL, info_label);

	needle = add_stock_st_gauge(-250, 0, ui_view1, &iat);

    //lv_obj_send_event(mbox, LV_EVENT_VALUE_CHANGED, &btn_id);

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

		boost = boost + 0.01;
		if( boost > 25 )
			boost = -14.6;

		oil = oil + 0.2;
		if( oil > 198 )
			oil = 32.5;


		if( HAL_GetTick() < 2750) {
			switch_screen(ui_splash);
		} else if( boost > 10 ){
			switch_screen(ui_view2);
		} else if( boost > 0 ) {
			switch_screen(ui_view3);
		} else {
			switch_screen(ui_view1);
		}


		if( oil > 80 )
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

		switch( gauge )
		{
			case 0:
			//lv_arc_set_value(ui_gauge1, iat);
			if(screen_active(ui_view1)) {
				//lv_img_set_angle(needle, (iat*10)-600);
				lv_obj_send_event(needle, LV_EVENT_REFRESH, &iat);
				//lv_label_set_text_fmt(ui_value1, "%.1f", iat);
			}
			break;

			case 1:
			//lv_arc_set_value(ui_Gauge2, boost);
			if(screen_active(ui_view1)) {
				lv_img_set_angle(ui_needle2, (boost*10)-600);
				lv_label_set_text_fmt(ui_value2, "%.2f psi", boost);
			}
			break;

			case 2:
			if(screen_active(ui_view1)) {
				//lv_arc_set_value(ui_Gauge3, oil);
				lv_img_set_angle(ui_needle3, (oil*10)-600);
				lv_label_set_text_fmt(ui_value3, "%.1f F", oil);
			}
			break;

			case 3:
			if(screen_active(ui_view2)) {
				lv_slider_set_value(ui_linear1, boost*10, LV_ANIM_ON);
				lv_label_set_text_fmt(ui_value4, "%.2f psi", boost);
			}
			break;

			case 4:
			if(screen_active(ui_view3)) {
				lv_arc_set_value(ui_arc1, (boost+15)*10);
				lv_label_set_text_fmt(ui_value5, "%.2f psi", boost);
			}
			break;

			case 5:
			if(screen_active(ui_view3)) {
				add_data(oil);
				lv_label_set_text_fmt(ui_value6, "%.1f F", oil);
			}
			break;

			case 6:
			if(screen_active(ui_view3)) {
				lv_obj_set_style_shadow_color(ui_gauge4, lv_color_hex(oil*0x6EB3E), LV_PART_MAIN | LV_STATE_DEFAULT);
			}
			break;
		}

		gauge = (gauge >= 6) ? 0 : gauge + 1;
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
