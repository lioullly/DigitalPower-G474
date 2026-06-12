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
#include "stm32g4xx_hal.h"

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
#define KEY2_Pin GPIO_PIN_0
#define KEY2_GPIO_Port GPIOC
#define il1_p_Pin GPIO_PIN_1
#define il1_p_GPIO_Port GPIOC
#define il1_n_Pin GPIO_PIN_2
#define il1_n_GPIO_Port GPIOC
#define il2_p_Pin GPIO_PIN_2
#define il2_p_GPIO_Port GPIOA
#define il2_n_Pin GPIO_PIN_3
#define il2_n_GPIO_Port GPIOA
#define KEY3_Pin GPIO_PIN_4
#define KEY3_GPIO_Port GPIOC
#define user_Pin GPIO_PIN_5
#define user_GPIO_Port GPIOC
#define Green_Pin GPIO_PIN_11
#define Green_GPIO_Port GPIOB
#define key_relay_Pin GPIO_PIN_12
#define key_relay_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_13
#define KEY1_GPIO_Port GPIOB
#define il3_p_Pin GPIO_PIN_14
#define il3_p_GPIO_Port GPIOB
#define il3_n_Pin GPIO_PIN_15
#define il3_n_GPIO_Port GPIOB
#define Red_Pin GPIO_PIN_15
#define Red_GPIO_Port GPIOA
#define KEY4_Pin GPIO_PIN_10
#define KEY4_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
