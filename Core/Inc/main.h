/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define LASER_GPIO_Pin GPIO_PIN_8
#define LASER_GPIO_GPIO_Port GPIOC
#define IST_RST_Pin GPIO_PIN_6
#define IST_RST_GPIO_Port GPIOG
#define IMU_HEAT_TIM_Pin GPIO_PIN_6
#define IMU_HEAT_TIM_GPIO_Port GPIOF
#define GREEN_LED_TIM_Pin GPIO_PIN_12
#define GREEN_LED_TIM_GPIO_Port GPIOH
#define IST_INT_Pin GPIO_PIN_3
#define IST_INT_GPIO_Port GPIOG
#define IST_INT_EXTI_IRQn EXTI3_IRQn
#define RED_LED_TIM_Pin GPIO_PIN_11
#define RED_LED_TIM_GPIO_Port GPIOH
#define BLUE_LED_TIM_Pin GPIO_PIN_10
#define BLUE_LED_TIM_GPIO_Port GPIOH
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOD
#define ACC_INT_Pin GPIO_PIN_4
#define ACC_INT_GPIO_Port GPIOC
#define ACC_INT_EXTI_IRQn EXTI4_IRQn
#define GYRO_INT_Pin GPIO_PIN_5
#define GYRO_INT_GPIO_Port GPIOC
#define GYRO_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
