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

typedef enum{
	P_OK,
	P_TEMP,
	P_OL,
	REPONER_POTE,
}T_PROTEC;

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
#define LED_onBoard_Pin GPIO_PIN_13
#define LED_onBoard_GPIO_Port GPIOC
#define Pul_hab_Vo_Pin GPIO_PIN_14
#define Pul_hab_Vo_GPIO_Port GPIOC
#define Pul_hab_Io_Pin GPIO_PIN_15
#define Pul_hab_Io_GPIO_Port GPIOC
#define Rep_Pote_Vo_Pin GPIO_PIN_4
#define Rep_Pote_Vo_GPIO_Port GPIOA
#define Rep_Pote_Io_Pin GPIO_PIN_5
#define Rep_Pote_Io_GPIO_Port GPIOA
#define P_Temp_Vo_Pin GPIO_PIN_6
#define P_Temp_Vo_GPIO_Port GPIOA
#define P_Temp_Io_Pin GPIO_PIN_7
#define P_Temp_Io_GPIO_Port GPIOA
#define P_OL_Vo_Pin GPIO_PIN_0
#define P_OL_Vo_GPIO_Port GPIOB
#define P_OL_Io_Pin GPIO_PIN_1
#define P_OL_Io_GPIO_Port GPIOB
#define HAB_Vo_Pin GPIO_PIN_10
#define HAB_Vo_GPIO_Port GPIOB
#define HAB_Io_Pin GPIO_PIN_11
#define HAB_Io_GPIO_Port GPIOB
#define LED_Vo_Pin GPIO_PIN_12
#define LED_Vo_GPIO_Port GPIOB
#define LED_Io_Pin GPIO_PIN_13
#define LED_Io_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_15
#define LCD_RS_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_8
#define LCD_EN_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_9
#define LCD_D4_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_10
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_11
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_12
#define LCD_D7_GPIO_Port GPIOA
#define Rango_Io_A_Pin GPIO_PIN_3
#define Rango_Io_A_GPIO_Port GPIOB
#define Rango_Io_B_Pin GPIO_PIN_4
#define Rango_Io_B_GPIO_Port GPIOB
#define Rango_Io_C_Pin GPIO_PIN_5
#define Rango_Io_C_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
