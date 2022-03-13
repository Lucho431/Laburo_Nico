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
#include "usart.h"
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

#define PROTEC_V ( (P_Temp_Vo_GPIO_Port->IDR & P_Temp_Vo_Pin) | (P_OL_Vo_GPIO_Port->IDR & P_OL_Vo_Pin) )
#define PROTEC_A ( (P_Temp_Io_GPIO_Port->IDR & P_Temp_Io_Pin) | (P_OL_Io_GPIO_Port->IDR & P_OL_Io_Pin) )

#define RANGO_I ( ((Rango_Io_A_GPIO_Port->IDR) >> 3) & 0b111 )
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
uint32_t muestras[400];

//variables de muestreo del ADC
uint32_t acumulaV = 0;
uint32_t acumulaI = 0;
uint32_t acum_RMS_samplesV = 0;
uint32_t acum_RMS_samplesI = 0;
uint8_t cuenta_RMS_samplesV = 0;
uint8_t cuenta_RMS_samplesI = 0;
uint32_t RMS_samplesV = 0;
uint32_t RMS_samplesI = 0;
uint8_t status_adc = 0;

//variables de selección de rango
uint8_t rango_Io = 1;
float Imax; //valor máximo de un rango
float coefRango_Io; //coeficioente de conversión de codigo ADC al rango de corriente
float valor_Io;
float valor_Vo;
float Vmax = 608.11; //valor máxiom del rango (Vp).
float coefRango_Vo = 0.297002;

//variables de detección de fase
uint8_t flag_faseNegativa = 0;
uint32_t acum_fase = 0;
uint8_t cuenta_fase = 0;
float valor_fase = 0;

//variables de timers
uint8_t flag_tim2 = 0;

//variables de protecciones
uint8_t flag_protecV = 0;
uint8_t flag_protecI = 0;

//variables de impresión de pantalla
char texto[30];
uint8_t refrescaPantalla = 25; //tiempo de refresco de pantalla en 10 * ms.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void valor_mediciones (void);
void imprimePantalla (void);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);

  HAL_TIM_Base_Start(&htim3);

  HAL_ADC_Start_DMA(&hadc1, muestras, sizeof(muestras));

  HAL_TIM_Base_Start_IT(&htim2); //desborda cada 10 ms.

  LCD_Init();

  LCD_GoToxy(8, 1);
  sprintf(texto, "OLEC");
  LCD_Print(texto);
  LCD_GoToxy(3, 2);
  sprintf(texto, "CAJA DE CARGA");
  LCD_Print(texto);

  HAL_Delay(5000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  switch (status_adc) {
		  case 1:
			  for (uint16_t i = 0; i < 200; i+=2){
				  acumulaV += muestras[i]*muestras[i];
				  acumulaI += muestras[i+1]*muestras[i+1];
			  }

			  acum_RMS_samplesV += (uint32_t) (sqrt( (double) acumulaV) / 10);
			  acum_RMS_samplesI += (uint32_t) (sqrt( (double) acumulaI) / 10);
			  cuenta_RMS_samplesV++;
			  cuenta_RMS_samplesI++;

			  if (cuenta_RMS_samplesV == 10){
				  RMS_samplesV = acum_RMS_samplesV / 10;
				  RMS_samplesI = acum_RMS_samplesI / 10;

				  cuenta_RMS_samplesV = 0;
				  acum_RMS_samplesV = 0;
				  cuenta_RMS_samplesI = 0;
				  acum_RMS_samplesI = 0;
			  }

			  acumulaV = 0;
			  acumulaI = 0;

			  status_adc = 0;

		  break;
		  case 2:
			  for (uint16_t i = 200; i < 400; i+=2){
				  acumulaV += muestras[i]*muestras[i];
				  acumulaI += muestras[i+1]*muestras[i+1];
			  }

			  acum_RMS_samplesV += (uint32_t) (sqrt( (double) acumulaV) / 10);
			  acum_RMS_samplesI += (uint32_t) (sqrt( (double) acumulaI) / 10);
			  cuenta_RMS_samplesV++;
			  cuenta_RMS_samplesI++;

			  if (cuenta_RMS_samplesV == 10){
				  RMS_samplesV = acum_RMS_samplesV / 10;
				  RMS_samplesI = acum_RMS_samplesI / 10;

				  valor_mediciones ();

				  cuenta_RMS_samplesV = 0;
				  acum_RMS_samplesV = 0;
				  cuenta_RMS_samplesI = 0;
				  acum_RMS_samplesI = 0;
			  }

			  acumulaV = 0;
			  acumulaI = 0;

			  status_adc = 0;

		  default:
		  break;
	  } //fin switch(status_adc)


	  if (cuenta_fase == 5){

		  if (acum_fase > 4500){ //mayor que 180 grados.
			  valor_fase = (float) (acum_fase * 0.04 - 360.0);
		  }else{
			  valor_fase = (float) (acum_fase * 0.04);
		  }

		  acum_fase = 0;
		  cuenta_fase = 0;
	  } //fin if cuenta_fase



	  switch (flag_protecV){
		  case 0:
			  if (HAL_GPIO_ReadPin(P_Temp_Vo_GPIO_Port, P_Temp_Vo_Pin) != 0){
				  HAL_GPIO_WritePin(HAB_Vo_GPIO_Port, HAB_Vo_Pin, SIGNAL_OFF);
				  flag_protecV = 1;
			  }else if (HAL_GPIO_ReadPin(P_OL_Vo_GPIO_Port, P_OL_Vo_Pin) != 0){
				  HAL_GPIO_WritePin(HAB_Vo_GPIO_Port, HAB_Vo_Pin, SIGNAL_OFF);
				  flag_protecV = 2;
			  }
		  break;
		  case 1:
		  case 2:
			  if (!PROTEC_V) break;

			  if (HAL_GPIO_ReadPin(Rep_Pote_Vo_GPIO_Port, Rep_Pote_Vo_Pin) == (GPIO_PinState)SIGNAL_ON){
				  HAL_GPIO_WritePin(HAB_Vo_GPIO_Port, HAB_Vo_Pin, SIGNAL_ON);
				  flag_protecV = 0;
			  }
		  default:
		  break;
	  } //fin switch flag_protecV



	  switch (flag_protecI){
		  case 0:
			  if (HAL_GPIO_ReadPin(P_Temp_Io_GPIO_Port, P_Temp_Io_Pin) != 0){
				  HAL_GPIO_WritePin(HAB_Io_GPIO_Port, HAB_Io_Pin, SIGNAL_OFF);
				  flag_protecI = 1;
			  }else if (HAL_GPIO_ReadPin(P_OL_Io_GPIO_Port, P_OL_Io_Pin) != 0){
				  HAL_GPIO_WritePin(HAB_Io_GPIO_Port, HAB_Io_Pin, SIGNAL_OFF);
				  flag_protecI = 1;
			  }
		  break;
		  case 1:
		  case 2:
			  if (!PROTEC_A) break;

			  if (HAL_GPIO_ReadPin(Rep_Pote_Io_GPIO_Port, Rep_Pote_Io_Pin) == (GPIO_PinState)SIGNAL_ON){
				  HAL_GPIO_WritePin(HAB_Io_GPIO_Port, HAB_Io_Pin, SIGNAL_ON);
				  flag_protecI = 0;
			  }
		  default:
		  break;
	  } //fin switch flag_protecI

	  if (flag_tim2 != 0){ //desborda cada 10 ms

		  imprimePantalla();

		  flag_tim2 = 0;

	  } //fin if flag_tim2


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
void valor_mediciones (void){

	switch (RANGO_I){

		case 1:
			//convierte a float 500mA
			coefRango_Io = 0.3272;
			Imax = 670.00;
		break;
		case 2:
			//convierte a float 1A
			coefRango_Io = 0.0006545;
			Imax = 1.34;
		break;
		case 3:
			//convierte a float 2A
			coefRango_Io = 0.001309;
			Imax = 2.68;
		break;
		case 4:
			//convierte a float 5A
			coefRango_Io = 0.003273;
			Imax = 6.7;
		break;
		case 5:
			//convierte a float 10A
			coefRango_Io = 0.006546;
			Imax = 13.4;
		break;
		case 6:
			//convierte a float 20A
			coefRango_Io = 0.01309;
			Imax = 26.8;
		break;
		case 7:
			//convierte a float 50A
			coefRango_Io = 0.032725;
			Imax = 67.0;
		default:
		break;

	} //fin switch RANG_I

	valor_Io = (float) (RMS_samplesI * coefRango_Io - Imax);
	valor_Vo = (float) (RMS_samplesV * coefRango_Vo - Vmax);

} //fin valor_mediciones()

void imprimePantalla(void) {

	if (refrescaPantalla != 0) {

		refrescaPantalla--;

	} else {

		valor_mediciones();

		LCD_GoToxy(0, 0);
		LCD_Print("Título");
		LCD_GoToxy(0, 1);

		switch (flag_protecV) {
			case 0:
				sprintf(texto, "Tension: %3.2f [V]", valor_Vo);
			break;
			case 1:
				sprintf(texto, "Sobretemp. Tension");
			break;
			case 2:
				sprintf(texto, "Tension Maxima");
			break;
		} //fin switch flag_protecV
		LCD_Print(texto);
		LCD_GoToxy(0, 2);

		switch (flag_protecI) {
			case 0:
				if (RANGO_I == 1)
					sprintf(texto, "Corriente: %5.2f[mA]", valor_Io);
				else
					sprintf(texto, "Corriente: %4.2f [A]", valor_Io);
			break;
			case 1:
				sprintf(texto, "Sobretemp. Corriente");
			break;
			case 2:
				sprintf(texto, "Corriente Maxima");
			break;
		} //fin switch flag_protecI

		LCD_Print(texto);
		LCD_GoToxy(0, 3);
		sprintf(texto, "Angulo: %4.1f [°]", valor_fase);
		LCD_Print(texto);

		refrescaPantalla = 25;

	} //fin if refrescaPantalla

} //fin imprimePantalla ()

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	status_adc = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	status_adc = 2;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){ // i.e.: PB6 ascendente

		__HAL_TIM_SET_COUNTER (&htim4, 0);

	} //fin if HAL_TIM_ACTIVE_CHANNEL_1

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){ // i.e.: PB8 descendente

		acum_fase += __HAL_TIM_GET_COUNTER (&htim4);
		cuenta_fase++;

	} //fin HAL_TIM_ACTIVE_CHANNEL_3

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
