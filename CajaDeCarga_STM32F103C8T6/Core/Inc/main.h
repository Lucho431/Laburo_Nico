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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum{
	SIGNAL_OFF = 0,
	SIGNAL_ON,
}T_SIGNAL;

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
#define OUT_ENAMP1_Pin GPIO_PIN_3
#define OUT_ENAMP1_GPIO_Port GPIOA
#define OUT_ENAMP2_Pin GPIO_PIN_4
#define OUT_ENAMP2_GPIO_Port GPIOA
#define IN_P1CERO_Pin GPIO_PIN_5
#define IN_P1CERO_GPIO_Port GPIOA
#define IN_P2CERO_Pin GPIO_PIN_6
#define IN_P2CERO_GPIO_Port GPIOA
#define IN_PV1_Pin GPIO_PIN_7
#define IN_PV1_GPIO_Port GPIOA
#define IN_PV2_Pin GPIO_PIN_0
#define IN_PV2_GPIO_Port GPIOB
#define IN_PV3_Pin GPIO_PIN_1
#define IN_PV3_GPIO_Port GPIOB
#define IN_PA1_Pin GPIO_PIN_10
#define IN_PA1_GPIO_Port GPIOB
#define IN_PA2_Pin GPIO_PIN_11
#define IN_PA2_GPIO_Port GPIOB
#define IN_PA3_Pin GPIO_PIN_12
#define IN_PA3_GPIO_Port GPIOB
#define OUT_RS_Pin GPIO_PIN_15
#define OUT_RS_GPIO_Port GPIOB
#define OUT_EN_Pin GPIO_PIN_8
#define OUT_EN_GPIO_Port GPIOA
#define OUT_D4_Pin GPIO_PIN_9
#define OUT_D4_GPIO_Port GPIOA
#define OUT_D5_Pin GPIO_PIN_10
#define OUT_D5_GPIO_Port GPIOA
#define OUT_D6_Pin GPIO_PIN_11
#define OUT_D6_GPIO_Port GPIOA
#define OUT_D7_Pin GPIO_PIN_12
#define OUT_D7_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
