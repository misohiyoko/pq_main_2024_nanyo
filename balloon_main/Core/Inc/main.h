/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_dma.h"

#include "stm32l4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define SD_DET_Pin GPIO_PIN_13
#define SD_DET_GPIO_Port GPIOC
#define GPS_INT_Pin GPIO_PIN_2
#define GPS_INT_GPIO_Port GPIOC
#define GPS_RESET_Pin GPIO_PIN_3
#define GPS_RESET_GPIO_Port GPIOC
#define CTRL_SENSE_Pin GPIO_PIN_5
#define CTRL_SENSE_GPIO_Port GPIOA
#define CTRL_COMM_Pin GPIO_PIN_6
#define CTRL_COMM_GPIO_Port GPIOA
#define GPIO_PPS_Pin GPIO_PIN_1
#define GPIO_PPS_GPIO_Port GPIOB
#define GPIO_PPS_EXTI_IRQn EXTI1_IRQn
#define GPS_SAFE_Pin GPIO_PIN_6
#define GPS_SAFE_GPIO_Port GPIOC
#define RM92A_RESEST_Pin GPIO_PIN_7
#define RM92A_RESEST_GPIO_Port GPIOC
#define BNO055_RESET_Pin GPIO_PIN_8
#define BNO055_RESET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
