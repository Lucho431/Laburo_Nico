/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "lcd_fer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define PROTEC_V ( (IN_PV1_GPIO_Port->IDR & IN_PV1_Pin) | (IN_PV2_GPIO_Port->IDR & IN_PV2_Pin) | (IN_PV3_GPIO_Port->IDR & IN_PV3_Pin) )
#define PROTEC_A ( (IN_PA1_GPIO_Port->IDR & IN_PA1_Pin) | (IN_PA2_GPIO_Port->IDR & IN_PA2_Pin) | (IN_PA3_GPIO_Port->IDR & IN_PA3_Pin) )

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t muestras[400];

uint32_t acumulaV = 0;
uint32_t acumulaA = 0;
uint32_t RMS_samplesV = 0;
uint32_t RMS_samplesA = 0;

uint8_t status_adc = 0;

uint8_t status_fase = 0;
uint8_t flag_faseNegativa = 0;
uint32_t acum_fase = 0;
uint8_t cuenta_fase = 0;
int32_t valor_fase = 0;

//uint32_t ic1, ic2, ic3, ic4;
uint32_t ventana_inicio = 0;
uint32_t ventana_fin = 0;

uint8_t flag_tim2 = 0;

uint8_t flag_protecV = 0;
uint8_t flag_protecA = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

  HAL_TIM_Base_Start(&htim3);

  HAL_ADC_Start_DMA(&hadc1, muestras, sizeof(muestras));

//  HAL_TIM_Base_Start_IT(&htim2); //desborda cada 10 ms.

  LCD_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  switch (status_adc) {
		  case 1:
			  for (uint16_t i = 0; i < 200; i+=2){
				  acumulaV += muestras[i]*muestras[i];
				  acumulaA += muestras[i+1]*muestras[i+1];
			  }

			  RMS_samplesV = (uint32_t) (sqrt( (double) acumulaV) / 10);
			  RMS_samplesA = (uint32_t) (sqrt( (double) acumulaA) / 10);

			  acumulaV = 0;
			  acumulaA = 0;

			  status_adc = 0;

		  break;
		  case 2:
			  for (uint16_t i = 200; i < 400; i+=2){
				  acumulaV += muestras[i]*muestras[i];
				  acumulaA += muestras[i+1]*muestras[i+1];
			  }

			  RMS_samplesV = (uint32_t) (sqrt( (double) acumulaV) / 10);
			  RMS_samplesA = (uint32_t) (sqrt( (double) acumulaA) / 10);

			  acumulaV = 0;
			  acumulaA = 0;

			  status_adc = 0;

		  default:
		  break;
	  } //fin switch(status_adc)


	  if (status_fase == 2){

		  acum_fase += (ventana_fin - ventana_inicio);
		  cuenta_fase++;

		  if (cuenta_fase == 5){

			  valor_fase = acum_fase / 5;

			  if (flag_faseNegativa != 0){
				  valor_fase *= -1;
			  }

			  acum_fase = 0;
			  cuenta_fase = 0;
		  } //fin if cuenta_fase

		  status_fase = 0;
	  } //fin if status_fase


	  switch (flag_protecV){
		  case 0:
			  if (PROTEC_V != 0){
				  HAL_GPIO_WritePin(OUT_ENAMP1_GPIO_Port, OUT_ENAMP1_Pin, SIGNAL_OFF);
				  flag_protecV = 1;
			  }
		  break;
		  case 1:
			  if (!PROTEC_V) break;

			  if (HAL_GPIO_ReadPin(IN_P1CERO_GPIO_Port, IN_P1CERO_Pin) == (GPIO_PinState)SIGNAL_ON){
				  HAL_GPIO_WritePin(OUT_ENAMP1_GPIO_Port, OUT_ENAMP1_Pin, SIGNAL_ON);
				  flag_protecV = 0;
			  }
		  default:
		  break;
	  } //fin switch flag_protecV

	  switch (flag_protecA){
		  case 0:
			  if (PROTEC_A != 0){
				  HAL_GPIO_WritePin(OUT_ENAMP2_GPIO_Port, OUT_ENAMP2_Pin, SIGNAL_OFF);
				  flag_protecA = 1;
			  }
		  break;
		  case 1:
			  if (!PROTEC_A) break;

			  if (HAL_GPIO_ReadPin(IN_P2CERO_GPIO_Port, IN_P2CERO_Pin) == (GPIO_PinState)SIGNAL_ON){
				  HAL_GPIO_WritePin(OUT_ENAMP2_GPIO_Port, OUT_ENAMP2_Pin, SIGNAL_ON);
				  flag_protecA = 0;
			  }
		  default:
		  break;
	  } //fin switch flag_protecA

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	status_adc = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	status_adc = 2;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){ // i.e.: PB6 ascendente
		//HAL_TIM_ReadCapturedValue(htim, HAL_TIM_ACTIVE_CHANNEL_3);
		//ic1 = htim->Instance->CCR1;

		//if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 1)
		if ((GPIOB->IDR & GPIO_PIN_8) != 0){

			if (!status_fase){
				status_fase = 1;
				ventana_inicio = htim->Instance->CCR1;
			}
		}else{
			flag_faseNegativa = 0;
		} //fin if GPIOB...


	} //fin if HAL_TIM_ACTIVE_CHANNEL_1

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){ // i.e.: PB6 descendente

		if (status_fase == 1){

		ventana_fin = htim->Instance->CCR2;
		status_fase = 2;
		}

	} //fin if HAL_TIM_ACTIVE_CHANNEL_2

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){ // i.e.: PB8 ascendente

		if ((GPIOB->IDR & GPIO_PIN_6) != 0){

			if (!status_fase){
				status_fase = 1;
				ventana_inicio = htim->Instance->CCR3;
			}
		}else{
			flag_faseNegativa = 1;
		} //fin if GPIOB...

	} //fin HAL_TIM_ACTIVE_CHANNEL_3

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){ //i.e.: PB8 descendente

		if (status_fase == 1){
			ventana_fin = htim->Instance->CCR4;
			status_fase = 2;
		}

	} //fin HAL_TIM_ACTIVE_CHANNEL_4


} //fin HAL_TIM_IC_CaptureCallback()

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM2){
		flag_tim2 = 1;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
