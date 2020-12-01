/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define ROW1_Pin GPIO_PIN_0
#define ROW1_GPIO_Port GPIOC
#define ROW2_Pin GPIO_PIN_1
#define ROW2_GPIO_Port GPIOC
#define ROW3_Pin GPIO_PIN_2
#define ROW3_GPIO_Port GPIOC
#define ROW4_Pin GPIO_PIN_3
#define ROW4_GPIO_Port GPIOC
#define COLUMN1_Pin GPIO_PIN_0
#define COLUMN1_GPIO_Port GPIOA
#define COLUMN2_Pin GPIO_PIN_1
#define COLUMN2_GPIO_Port GPIOA
#define COLUMN3_Pin GPIO_PIN_2
#define COLUMN3_GPIO_Port GPIOA
#define COLUMN4_Pin GPIO_PIN_3
#define COLUMN4_GPIO_Port GPIOA
#define SLIDER4_Pin GPIO_PIN_6
#define SLIDER4_GPIO_Port GPIOA
#define SLIDER2_Pin GPIO_PIN_7
#define SLIDER2_GPIO_Port GPIOA
#define SLIDER3_Pin GPIO_PIN_4
#define SLIDER3_GPIO_Port GPIOC
#define SLIDER1_Pin GPIO_PIN_5
#define SLIDER1_GPIO_Port GPIOC
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOB
#define Demux_VCC_Pin GPIO_PIN_11
#define Demux_VCC_GPIO_Port GPIOA
#define Demux_GND_Pin GPIO_PIN_12
#define Demux_GND_GPIO_Port GPIOA
#define Demux_S4_Pin GPIO_PIN_10
#define Demux_S4_GPIO_Port GPIOC
#define Demux_S3_Pin GPIO_PIN_11
#define Demux_S3_GPIO_Port GPIOC
#define Demux_S2_Pin GPIO_PIN_12
#define Demux_S2_GPIO_Port GPIOC
#define Demux_S1_Pin GPIO_PIN_2
#define Demux_S1_GPIO_Port GPIOD
#define BLUE_PWM_Pin GPIO_PIN_5
#define BLUE_PWM_GPIO_Port GPIOB
#define GREEN_PWM_Pin GPIO_PIN_6
#define GREEN_PWM_GPIO_Port GPIOB
#define RED_PWM_Pin GPIO_PIN_7
#define RED_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
