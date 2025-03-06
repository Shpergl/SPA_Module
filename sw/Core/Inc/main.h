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
#include "stm32f1xx_hal.h"

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
#define Parking_Button_Pin GPIO_PIN_1
#define Parking_Button_GPIO_Port GPIOA
#define Parking_Radar_Pin GPIO_PIN_2
#define Parking_Radar_GPIO_Port GPIOA
#define Right_Camera_Pin GPIO_PIN_3
#define Right_Camera_GPIO_Port GPIOA
#define Left_Camera_Pin GPIO_PIN_4
#define Left_Camera_GPIO_Port GPIOA
#define Front_Camera_Pin GPIO_PIN_5
#define Front_Camera_GPIO_Port GPIOA
#define Rear_Camera_Pin GPIO_PIN_7
#define Rear_Camera_GPIO_Port GPIOA
#define Video_Output_Pin GPIO_PIN_0
#define Video_Output_GPIO_Port GPIOB
#define TIM2_Pin GPIO_PIN_15
#define TIM2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
