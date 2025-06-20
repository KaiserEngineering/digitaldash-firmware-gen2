/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../ke_conf.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FORD_BKLT_Pin GPIO_PIN_5
#define FORD_BKLT_GPIO_Port GPIOE
#define GPIO1_Pin GPIO_PIN_13
#define GPIO1_GPIO_Port GPIOC
#define HSPI_NRST_Pin GPIO_PIN_0
#define HSPI_NRST_GPIO_Port GPIOB
#define ESP32_RESET_N_Pin GPIO_PIN_1
#define ESP32_RESET_N_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_0
#define LCD_EN_GPIO_Port GPIOG
#define GPIO2_Pin GPIO_PIN_1
#define GPIO2_GPIO_Port GPIOG
#define USB_SLEEP_N_Pin GPIO_PIN_8
#define USB_SLEEP_N_GPIO_Port GPIOA
#define DBG_LED1_Pin GPIO_PIN_11
#define DBG_LED1_GPIO_Port GPIOA
#define DBG_LED2_Pin GPIO_PIN_12
#define DBG_LED2_GPIO_Port GPIOA
#define CAN_STBY_Pin GPIO_PIN_2
#define CAN_STBY_GPIO_Port GPIOD
#define PWR_HOLD_Pin GPIO_PIN_7
#define PWR_HOLD_GPIO_Port GPIOD
#define PWR_VD_Pin GPIO_PIN_10
#define PWR_VD_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
