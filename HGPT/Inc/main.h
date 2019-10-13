/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Process_100Hz();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_1
#define SW2_GPIO_Port GPIOC
#define MD_STBY_Pin GPIO_PIN_2
#define MD_STBY_GPIO_Port GPIOC
#define BAT_VOL_Pin GPIO_PIN_3
#define BAT_VOL_GPIO_Port GPIOC
#define ENL_CHB_Pin GPIO_PIN_1
#define ENL_CHB_GPIO_Port GPIOA
#define ENL_CHA_Pin GPIO_PIN_5
#define ENL_CHA_GPIO_Port GPIOA
#define ENR_CHA_Pin GPIO_PIN_6
#define ENR_CHA_GPIO_Port GPIOA
#define ENR_CHB_Pin GPIO_PIN_7
#define ENR_CHB_GPIO_Port GPIOA
#define MUX_RESET_Pin GPIO_PIN_5
#define MUX_RESET_GPIO_Port GPIOC
#define MUX1_Pin GPIO_PIN_13
#define MUX1_GPIO_Port GPIOB
#define MUX2_Pin GPIO_PIN_14
#define MUX2_GPIO_Port GPIOB
#define MUX3_Pin GPIO_PIN_15
#define MUX3_GPIO_Port GPIOB
#define DRL_IN1_Pin GPIO_PIN_6
#define DRL_IN1_GPIO_Port GPIOC
#define DRL_IN2_Pin GPIO_PIN_7
#define DRL_IN2_GPIO_Port GPIOC
#define DRR_IN1_Pin GPIO_PIN_8
#define DRR_IN1_GPIO_Port GPIOC
#define DRR_IN2_Pin GPIO_PIN_9
#define DRR_IN2_GPIO_Port GPIOC
#define PWM_L_Pin GPIO_PIN_8
#define PWM_L_GPIO_Port GPIOA
#define PWM_R_Pin GPIO_PIN_9
#define PWM_R_GPIO_Port GPIOA
#define LED_BAT_Pin GPIO_PIN_10
#define LED_BAT_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
