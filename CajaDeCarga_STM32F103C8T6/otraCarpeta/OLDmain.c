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
#include "float.h"
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

	struct Prot Vo;
	struct Prot Io;

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
int32_t Imax; //valor máximo de un rango
int32_t coefRango_Io; //coeficioente de conversión de codigo ADC al rango de corriente
int32_t valor_Io;
int32_t valor_Vo;
int32_t Vmax = 60811; //608.11 //valor máxiom del rango (Vp).
int32_t coefRango_Vo = 297002; //0.297002

//variables de detección de fase
uint8_t flag_faseNegativa = 0;
uint32_t acum_fase = 0;
uint8_t cuenta_fase = 0;
uint32_t valor_fase = 0;

//variables de systick
uint16_t count_tick = 0;

//variables de protecciones
T_PROTEC flag_protecV = P_OK;
T_PROTEC flag_protecI = P_OK;

//variables de impresión de pantalla
char texto[30];
uint8_t refrescaPantalla = 25; //tiempo de refresco de pantalla en 10 * ms.

uint32_t rrr1= 0;
uint32_t rrr2= 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void valor_mediciones (void);
void imprimePantalla (void);
void protecciones (void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t estado_Vo=0;
uint8_t estado_Io=0;
uint8_t flag_relay=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	struct Prot Vo;
	struct Prot Io;

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

  HAL_TIM_Base_Start(&htim4);  //captura fase
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1); //captura fase
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3); //captura fase

  LCD_Init();

  LCD_GoToxy(7, 1);
  sprintf(texto, "OELEC");
  LCD_Print(texto);
  LCD_GoToxy(3, 2);
  sprintf(texto, "CAJA DE CARGA");
  LCD_Print(texto);

  HAL_Delay(3000);

  HAL_TIM_Base_Start(&htim3); //sincro ADC

  HAL_ADC_Start_DMA(&hadc1, muestras, sizeof(muestras)); //ADC por DMA

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

			  //HAL_ADC_Start_DMA(&hadc1, muestras, sizeof(muestras));

		  default:
		  break;
	  } //fin switch(status_adc)


	  if (cuenta_fase == 5){

		  if (acum_fase > 4500){ //mayor que 180 grados.
			  valor_fase = acum_fase * 4 / 10 - 3600; //acum_fase * 0.04 - 360.0
		  }else{
			  valor_fase = acum_fase * 4 / 10; //acum_fase * 0.04
		  }

		  acum_fase = 0;
		  cuenta_fase = 0;
	  } //fin if cuenta_fase

	  protecciones ();

	  if (count_tick > 10){ //refresca cada 10 ms

		  imprimePantalla();

		  count_tick = 0;

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

		default:
		case 1:
			//convierte a float 500mA
			coefRango_Io = 3272; //0.3272;
			Imax = 67000; //670.00;
			valor_Io = (RMS_samplesI * coefRango_Io / 100 - Imax);
		break;
		case 2:
			//convierte a float 1A
			coefRango_Io = 6545; //0.0006545;
			Imax = 134; //1.34;
			valor_Io = (RMS_samplesI * coefRango_Io / 100000 - Imax);
		break;
		case 3:
			//convierte a float 2A
			coefRango_Io = 1309; //0.001309;
			Imax = 268; //2.68;
			valor_Io = (RMS_samplesI * coefRango_Io / 10000 - Imax);
		break;
		case 4:
			//convierte a float 5A
			coefRango_Io = 3273; //0.003273;
			Imax = 670; //6.7;
			valor_Io = (RMS_samplesI * coefRango_Io / 10000 - Imax);
		break;
		case 5:
			//convierte a float 10A
			coefRango_Io = 6546; //0.006546;
			Imax = 1340; //13.4;
			valor_Io = (RMS_samplesI * coefRango_Io / 10000 - Imax);
		break;
		case 6:
			//convierte a float 20A
			coefRango_Io = 1309; //0.01309;
			Imax = 2680; //26.8;
			valor_Io = (RMS_samplesI * coefRango_Io / 1000 - Imax);
		break;
		case 7:
			//convierte a float 50A
			coefRango_Io = 32725; //0.032725;
			Imax = 6700; //67.0;
			valor_Io = (RMS_samplesI * coefRango_Io / 10000 - Imax);
		break;

	} //fin switch RANG_I

	valor_Vo = (RMS_samplesV * coefRango_Vo / 10000 - Vmax);

} //fin valor_mediciones()

void imprimePantalla(void) {


	if (refrescaPantalla != 0) {

		refrescaPantalla--;

	} else {

		valor_mediciones();

		LCD_GoToxy(5, 0);
		sprintf(texto, "MEDICIONES");
		//LCD_Print("Titulo");
		LCD_Print(texto);
		LCD_GoToxy(0, 1);

		LCD_Print("                    ");
		LCD_GoToxy(0, 1);
		switch (flag_protecV) {
			case P_OK:
//				sprintf(texto, "Tension: %3.2f [V]", valor_Vo);
				sprintf(texto, "Tension: %d.%d [V]", valor_Vo / 100, valor_Vo % 100);
			break;
			case P_TEMP:
				sprintf(texto, "Sobretemp. Tension");
			break;
			case P_OL:
				sprintf(texto, "Tension Maxima");
			break;
			case REPONER_POTE:
				sprintf(texto, "Reponer pote. V");
			break;
		} //fin switch flag_protecV
		LCD_Print(texto);
		LCD_GoToxy(0, 2);

		LCD_Print("                    ");
		LCD_GoToxy(0, 2);
		switch (flag_protecI) {
			case P_OK:
				if (RANGO_I == 1)
//					sprintf(texto, "Corriente: %5.2f[mA]", valor_Io);
					sprintf(texto, "Corriente:%d.%d[mA]", valor_Io / 100, valor_Io % 100);
				else
//					sprintf(texto, "Corriente: %4.2f [A]", valor_Io);
					sprintf(texto, "Corriente: %d.%d [A]", valor_Io / 100, valor_Io % 100);
			break;
			case P_TEMP:
				sprintf(texto, "Sobretemp. Corriente");
			break;
			case P_OL:
				sprintf(texto, "Corriente Maxima");
			break;
			case REPONER_POTE:
				sprintf(texto, "Reponer pote. I");
			break;

		} //fin switch flag_protecI
		LCD_Print(texto);
		LCD_GoToxy(0, 3);

		LCD_Print("                    ");
		LCD_GoToxy(0, 3);
		sprintf(texto, "Angulo: %d.%d [deg]", valor_fase / 10, valor_fase % 10);
		LCD_Print(texto);

		refrescaPantalla = 27;

	} //fin if refrescaPantalla

} //fin imprimePantalla ()

void protecciones (void){

	switch (flag_protecV) {
		case P_OK:

			if (HAL_GPIO_ReadPin(P_Temp_Vo_GPIO_Port, P_Temp_Vo_Pin) != 0) {

				flag_protecV = P_TEMP;
			} else if (HAL_GPIO_ReadPin(P_OL_Vo_GPIO_Port, P_OL_Vo_Pin) != 0) {
				flag_protecV = P_OL;
			}

			break;
		case P_TEMP:

			if (HAL_GPIO_ReadPin(P_Temp_Vo_GPIO_Port, P_Temp_Vo_Pin) != 0) {
				break;
			} else if (HAL_GPIO_ReadPin(P_OL_Vo_GPIO_Port, P_OL_Vo_Pin) != 0) {
				flag_protecV = P_OL;
				break;
			}

			flag_protecV = REPONER_POTE;

			break;
		case P_OL:

			if (HAL_GPIO_ReadPin(P_Temp_Vo_GPIO_Port, P_Temp_Vo_Pin) != 0) {
				flag_protecV = P_TEMP;
				break;
			} else if (HAL_GPIO_ReadPin(P_OL_Vo_GPIO_Port, P_OL_Vo_Pin) != 0) {
				break;
			}

			flag_protecV = REPONER_POTE;

		case REPONER_POTE:

			if (HAL_GPIO_ReadPin(P_Temp_Vo_GPIO_Port, P_Temp_Vo_Pin) != 0) {
				flag_protecV = P_TEMP;
				break;
			} else if (HAL_GPIO_ReadPin(P_OL_Vo_GPIO_Port, P_OL_Vo_Pin) != 0) {
				flag_protecV = P_OL;
				break;
			}

			if (HAL_GPIO_ReadPin(Rep_Pote_Vo_GPIO_Port, Rep_Pote_Vo_Pin) == (GPIO_PinState) SIGNAL_ON)
				break;

			flag_protecV = P_OK;

		default:
			break;
	} //fin switch flag_protecV

	switch (flag_protecI) {
		case P_OK:

			if (HAL_GPIO_ReadPin(P_Temp_Io_GPIO_Port, P_Temp_Io_Pin) != 0) {
				flag_protecI = P_TEMP;
			} else if (HAL_GPIO_ReadPin(P_OL_Io_GPIO_Port, P_OL_Io_Pin) != 0) {
				flag_protecI = P_OL;
			}

			break;
		case P_TEMP:

			if (HAL_GPIO_ReadPin(P_Temp_Io_GPIO_Port, P_Temp_Io_Pin) != 0) {
				break;
			} else if (HAL_GPIO_ReadPin(P_OL_Io_GPIO_Port, P_OL_Io_Pin) != 0) {
				flag_protecI = P_OL;
				break;
			}

			flag_protecI = REPONER_POTE;

			break;
		case P_OL:

			if (HAL_GPIO_ReadPin(P_Temp_Io_GPIO_Port, P_Temp_Io_Pin) != 0) {
				flag_protecI = P_TEMP;
				break;
			} else if (HAL_GPIO_ReadPin(P_OL_Io_GPIO_Port, P_OL_Io_Pin)	!= 0) {
				break;
			}

			flag_protecI = REPONER_POTE;

		case REPONER_POTE:

			if (HAL_GPIO_ReadPin(P_Temp_Io_GPIO_Port, P_Temp_Io_Pin) != 0) {
				flag_protecI = P_TEMP;
				break;
			} else if (HAL_GPIO_ReadPin(P_OL_Io_GPIO_Port, P_OL_Io_Pin) != 0) {
				flag_protecI = P_OL;
				break;
			}

			if (HAL_GPIO_ReadPin(Rep_Pote_Io_GPIO_Port, Rep_Pote_Io_Pin) == (GPIO_PinState) SIGNAL_ON)
				break;

			flag_protecI = P_OK;

		default:
			break;
	} //fin switch flag_protecI


	  Vo.P_temp	=HAL_GPIO_ReadPin(P_Temp_Vo_GPIO_Port, P_Temp_Vo_Pin);
	  Vo.P_OL	=HAL_GPIO_ReadPin(P_OL_Vo_GPIO_Port, P_OL_Vo_Pin);
	  Vo.Rep	=HAL_GPIO_ReadPin(Rep_Pote_Vo_GPIO_Port, Rep_Pote_Vo_Pin);
	  Vo.Pul	=HAL_GPIO_ReadPin(Pul_Hab_Vo_GPIO_Port, Pul_Hab_Vo_Pin);

	  Io.P_temp	=HAL_GPIO_ReadPin(P_Temp_Io_GPIO_Port, P_Temp_Io_Pin);
	  Io.P_OL	=HAL_GPIO_ReadPin(P_OL_Io_GPIO_Port, P_OL_Io_Pin);
	  Io.Rep	=HAL_GPIO_ReadPin(Rep_Pote_Io_GPIO_Port, Rep_Pote_Io_Pin);
	  Io.Pul	=HAL_GPIO_ReadPin(Pul_Hab_Io_GPIO_Port, Pul_Hab_Io_Pin);

	  switch(estado_Vo)
	  {
		  case 0: //prendo cuando pulso
		  {
			if(Vo.P_temp==0 && Vo.P_OL==0 && Vo.Rep==0 && Vo.Pul==0)
			{
				if(flag_relay==0)
				{
					HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 1);
					HAL_Delay(1000);
					flag_relay=1;
				}

				if(Vo.Rep==0)
				{
				HAL_GPIO_TogglePin(LED_Board_GPIO_Port, LED_Board_Pin);
				HAL_GPIO_WritePin(Hab_Vo_GPIO_Port, Hab_Vo_Pin, 0);
				HAL_GPIO_WritePin(LED_Vo_GPIO_Port, LED_Vo_Pin, 1);
				HAL_Delay(300);
				estado_Vo=1;
				}
			}
		  }
		  break;

		  case 1: //apago cuando pulso o actua protección
		  {
			  if(Vo.P_temp==1 || Vo.P_OL==1 || Vo.Pul==0)
			  {
				if(Vo.P_OL==1)
				{
					HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 0);
					flag_relay=0;
				}

				HAL_GPIO_TogglePin(LED_Board_GPIO_Port, LED_Board_Pin);
				HAL_GPIO_WritePin(Hab_Vo_GPIO_Port, Hab_Vo_Pin, 1);
				HAL_GPIO_WritePin(LED_Vo_GPIO_Port, LED_Vo_Pin, 0);
				HAL_Delay(300);
				estado_Vo=0;


			  }
		  }
		  break;
	  } //Fin switch Vo


	  switch(estado_Io)
	  {
		  case 0: //prendo cuando pulso
		  {
			if(Io.P_temp==0 && Io.P_OL==0 && Io.Rep==0 && Io.Pul==0)
			{
				if(flag_relay==0)
				{
					HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 1);
					HAL_Delay(3000);
					flag_relay=1;
				}

				if(Io.Rep==0)
				{
				HAL_GPIO_TogglePin(LED_Board_GPIO_Port, LED_Board_Pin);
				HAL_GPIO_WritePin(Hab_Io_GPIO_Port, Hab_Io_Pin, 0);
				HAL_GPIO_WritePin(LED_Io_GPIO_Port, LED_Io_Pin, 1);
				HAL_Delay(500);
				estado_Io=1;
				}
			}
		  }
		  break;

		  case 1: //apago cuando pulso o actua protección
		  {
			  if(Io.P_temp==1 || Io.P_OL==1 || Io.Pul==0)
			  {
				  if(Io.P_OL==1)
				  {
					  HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 0);
					  flag_relay=0;
				  }

				  HAL_GPIO_TogglePin(LED_Board_GPIO_Port, LED_Board_Pin);
				  HAL_GPIO_WritePin(Hab_Io_GPIO_Port, Hab_Io_Pin, 1);
				  HAL_GPIO_WritePin(LED_Io_GPIO_Port, LED_Io_Pin, 0);
				  HAL_Delay(500);
				  estado_Io=0;
			  }
		  }
		  break;
	  } //Fin switch Io

} //fin protecciones ()


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	status_adc = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//HAL_ADC_Stop_DMA(&hadc1);
	//HAL_TIM_Base_Stop(&htim3); //sincro ADC
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

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if (htim->Instance == TIM2){
//		flag_tim2 = 1;
//	}
//}


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
