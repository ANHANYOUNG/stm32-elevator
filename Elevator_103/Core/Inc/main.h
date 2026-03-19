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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define KEY_R1_Pin GPIO_PIN_0
#define KEY_R1_GPIO_Port GPIOC
#define KEY_R2_Pin GPIO_PIN_1
#define KEY_R2_GPIO_Port GPIOC
#define KEY_R3_Pin GPIO_PIN_2
#define KEY_R3_GPIO_Port GPIOC
#define KEY_R4_Pin GPIO_PIN_3
#define KEY_R4_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define KEY_C1_Pin GPIO_PIN_4
#define KEY_C1_GPIO_Port GPIOC
#define KEY_C2_Pin GPIO_PIN_5
#define KEY_C2_GPIO_Port GPIOC
#define SENSOR2_1F_Pin GPIO_PIN_1
#define SENSOR2_1F_GPIO_Port GPIOB
#define SENSOR2_2F_Pin GPIO_PIN_2
#define SENSOR2_2F_GPIO_Port GPIOB
#define SENSOR2_3F_Pin GPIO_PIN_10
#define SENSOR2_3F_GPIO_Port GPIOB
#define SENSOR_1F_Pin GPIO_PIN_12
#define SENSOR_1F_GPIO_Port GPIOB
#define SENSOR_2F_Pin GPIO_PIN_13
#define SENSOR_2F_GPIO_Port GPIOB
#define SENSOR_3F_Pin GPIO_PIN_14
#define SENSOR_3F_GPIO_Port GPIOB
#define KEY_C3_Pin GPIO_PIN_6
#define KEY_C3_GPIO_Port GPIOC
#define KEY_C4_Pin GPIO_PIN_7
#define KEY_C4_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SPEAKER_Pin GPIO_PIN_2
#define SPEAKER_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
