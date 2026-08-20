/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define LIMIT1_Pin GPIO_PIN_3
#define LIMIT1_GPIO_Port GPIOE
#define LIMIT2_Pin GPIO_PIN_4
#define LIMIT2_GPIO_Port GPIOE
#define LIMIT3_Pin GPIO_PIN_5
#define LIMIT3_GPIO_Port GPIOE
#define LIMIT4_Pin GPIO_PIN_6
#define LIMIT4_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define EN4_Pin GPIO_PIN_0
#define EN4_GPIO_Port GPIOA
#define EN3_Pin GPIO_PIN_1
#define EN3_GPIO_Port GPIOA
#define EN2_Pin GPIO_PIN_2
#define EN2_GPIO_Port GPIOA
#define EN1_Pin GPIO_PIN_3
#define EN1_GPIO_Port GPIOA
#define DIR4_Pin GPIO_PIN_4
#define DIR4_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_5
#define DIR2_GPIO_Port GPIOC
#define ENA2_Pin GPIO_PIN_0
#define ENA2_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_1
#define DIR1_GPIO_Port GPIOB
#define DIR3_Pin GPIO_PIN_8
#define DIR3_GPIO_Port GPIOE
#define STEP3_Pin GPIO_PIN_9
#define STEP3_GPIO_Port GPIOE
#define ENA3_Pin GPIO_PIN_10
#define ENA3_GPIO_Port GPIOE
#define STEP4_Pin GPIO_PIN_11
#define STEP4_GPIO_Port GPIOE
#define ENA4_Pin GPIO_PIN_12
#define ENA4_GPIO_Port GPIOE
#define STEP2_Pin GPIO_PIN_13
#define STEP2_GPIO_Port GPIOE
#define STEP1_Pin GPIO_PIN_14
#define STEP1_GPIO_Port GPIOE
#define ENA1_Pin GPIO_PIN_15
#define ENA1_GPIO_Port GPIOE
#define LASER_Pin GPIO_PIN_14
#define LASER_GPIO_Port GPIOD
#define MOTOR775_Pin GPIO_PIN_15
#define MOTOR775_GPIO_Port GPIOD
#define SPI1_CS_Pin GPIO_PIN_7
#define SPI1_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
