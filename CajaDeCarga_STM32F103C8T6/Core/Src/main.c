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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "lcd_fer.h"
//#include "float.h"
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

#define PERIODO_BOARDLED 250
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t   Vo_P_temp	=0;
uint8_t	  Vo_P_OL	=0;
uint8_t	  Vo_Rep	=0;
uint8_t	  Vo_Pul	=0;

uint8_t	  Io_P_temp	=0;
uint8_t	  Io_P_OL	=0;
uint8_t	  Io_Rep	=0;
uint8_t	  Io_Pul	=0;

//Prot Vo;
//Prot Io;

int32_t muestras[400];

//variables INPUT
static T_INPUT pulsadorVo;
static T_INPUT pulsadorIo;
static T_INPUT P_OL_Vo;
static T_INPUT P_OL_Io;
static uint8_t read_pulsadorVo;
static uint8_t read_pulsadorIo;
static uint8_t read_OL_Vo;
static uint8_t read_OL_Io;

uint8_t last_pulsadorVo = 0;
uint8_t last_pulsadorIo = 0;
uint8_t last_OL_Vo = 0;
uint8_t last_OL_Io = 0;
uint8_t lectura_entradas = 100; //anti-rebote en ms.
uint8_t lectura_proteccion = 10;

//variables TIMERS
uint8_t flagTIM2 = 0;

//variables de muestreo del ADC
uint8_t flag_adc1 = 0;
uint8_t flag_adc2 = 0;
uint8_t num_muestraADC2 = 0; //0, 2, 4... 398
uint8_t num_muestraADC1 = 1; //1, 3, 5... 399
uint8_t flag_mitadADC1 = 0;
uint8_t flag_mitadADC2 = 0;
uint8_t flag_completoADC1 = 0;
uint8_t flag_completoADC2 = 0;

int32_t offset_adc = 1990; //2048;
int32_t aux_muestra_v = 0;
int32_t aux_muestra_i = 0;

int32_t acumulaV = 0;
int32_t acumulaI = 0;
int32_t acum_RMS_samplesV = 0;
int32_t acum_RMS_samplesI = 0;
uint8_t cuenta_RMS_samplesV = 0;
uint8_t cuenta_RMS_samplesI = 0;
int32_t RMS_samplesV = 0;
int32_t RMS_samplesI = 0;
uint8_t status_adc = 0;

//variables de selección de rango
uint8_t last_rango_Io;
uint8_t rango_Io = 1;
int32_t Imax; //valor máximo de un rango
int32_t coefRango_Io; //coeficioente de conversión de codigo ADC al rango de corriente
int32_t valor_Io;
int32_t valor_Vo;
int32_t Vmax = 60811; //608.11 //valor máxiom del rango (Vp).
int32_t coefRango_Vo = 2969; //2762; //0.297002

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
uint8_t flag_relay = 0;

uint16_t transitorioFunete = 3000; //en ms.

static T_PROTEC status_proteccionVo = P_APAGADO;
static T_PROTEC status_proteccionIo = P_APAGADO;

static uint8_t p_tempVo;
static uint8_t p_tempIo;
static uint8_t poteVo;
static uint8_t poteIo;


uint8_t demoraRelay = 100; //en 10 * ms.
uint8_t estadoDemoraRelay = 0;


//variables de impresión de pantalla
char texto[30];
uint16_t refrescaPantalla = 500; //tiempo de refresco de pantalla en ms.

//variables miscelaneas
uint16_t periodo_BoardLed = PERIODO_BOARDLED;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void valor_mediciones (void);
void imprimePantalla (void);
void protecciones (void);
void update_entradas (void);
void update_proteccion (void);
void check_entradas (void);
void check_proteccion (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t estado_Vo=0;
uint8_t estado_Io=0;

uint16_t i=0;

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim4);  //captura fase
//  HAL_TIM_Base_Start_IT(&htim2);  //refresco de 10 ms.
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1); //captura fase
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3); //captura fase

  LCD_Init();
  LCD_GoToxy(7, 1);
  sprintf(texto, "OELEC");
  LCD_Print(texto);
  LCD_GoToxy(6, 2);
  sprintf(texto, "GIM50v5");
  LCD_Print(texto);

  //Led de la placa
  HAL_GPIO_WritePin(LED_onBoard_GPIO_Port, LED_onBoard_Pin, 1);


  //Tiempo de arranque para la fuente de 48V, los leds actuan de testigo
  HAL_GPIO_WritePin(LED_Vo_GPIO_Port,LED_Vo_Pin, 1);
  HAL_GPIO_WritePin(LED_Io_GPIO_Port,LED_Io_Pin, 1);


  HAL_Delay(3000);


  HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 1);
  flag_relay=1;

  HAL_Delay(1000);

  LCD_Init();
  LCD_GoToxy(5, 0);
  sprintf(texto, "MEDICIONES");
  LCD_Print(texto);
  LCD_GoToxy(0, 1);
  sprintf(texto, "  Vo:");
  LCD_Print(texto);
  LCD_GoToxy(0, 2);
  sprintf(texto, "  Io:");
  LCD_Print(texto);
  LCD_GoToxy(0, 3);
  sprintf(texto, " Phi:");
  LCD_Print(texto);
  HAL_Delay(200);

  HAL_GPIO_WritePin(LED_Vo_GPIO_Port,LED_Vo_Pin, 0);
  HAL_GPIO_WritePin(LED_Io_GPIO_Port,LED_Io_Pin, 0);



  HAL_TIM_Base_Start(&htim3); //sincro ADC

  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*) muestras, sizeof(muestras)); //ADC por DMA
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_IT(&hadc2);

  last_rango_Io = RANGO_I;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*
	  if (HAL_GPIO_ReadPin(Pul_hab_Io_GPIO_Port, Pul_hab_Io_Pin)){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	  }else{
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	  }
*/
	  if (count_tick != 0){ //acciones del systickHandler

		  if (refrescaPantalla != 0){
			  refrescaPantalla--;
		  }else{
			  imprimePantalla();
		  }

		  if (periodo_BoardLed != 0){
			  periodo_BoardLed--;
		  }else{
//			  HAL_GPIO_TogglePin(LED_Board_GPIO_Port, LED_Board_Pin);
			  periodo_BoardLed = PERIODO_BOARDLED;
		  }

		  if (lectura_entradas != 0){
			  lectura_entradas--;
		  }else{
			  update_entradas();
		  }

		  if (lectura_proteccion != 0){
			  lectura_proteccion--;
		  }else{
			  update_proteccion();
		  }

		  if (transitorioFunete != 0)
			  transitorioFunete--;

		  count_tick = 0;
	  } //fin if (count_tick != 0)

	  check_entradas();
	  check_proteccion();

	  switch (status_adc) {

		  case 3:

			  if (flag_adc1 != 0){
				  muestras[num_muestraADC1] = HAL_ADC_GetValue(&hadc1);

				  if (num_muestraADC1 == 199)
					  flag_mitadADC1 = 1;

				  if (num_muestraADC1 < 399){
					  num_muestraADC1 += 2;
				  }else{
					  num_muestraADC1 = 1;
					  flag_completoADC1 = 1;
				  }

				  flag_adc1 = 0;
			  } //fin if (flag_adc1 != 0

			  if (flag_adc2 != 0){
				  muestras[num_muestraADC2] = HAL_ADC_GetValue(&hadc2);

				  if (num_muestraADC2 == 198)
					  flag_mitadADC2 = 1;

				  if (num_muestraADC2 < 398){
					  num_muestraADC2 += 2;
				  }else{
					  num_muestraADC2 = 0;
					  flag_completoADC2 = 1;
				  }

				  flag_adc2 = 0;
			  } //fin if (flag_adc2 != 0

			  if (flag_mitadADC1 != 0 && flag_mitadADC2 != 0){
				  status_adc = 1;
				  flag_mitadADC1 = 0;
				  flag_mitadADC2 = 0;
				  break;
			  }

			  if (flag_completoADC1 != 0 && flag_completoADC2 != 0){
				  status_adc = 2;
				  flag_completoADC1 = 0;
				  flag_completoADC2 = 0;
				  break;
			  }


			  if ( (!flag_adc1) && (!flag_adc2)) status_adc = 0;
		  break;
		  case 1:
			  for (uint16_t i = 0; i < 200; i+=2){
//				  muestras[i]	-=offset_adc;
//				  muestras[i+1]	-=offset_adc;
//
//				  acumulaV += muestras[i]*muestras[i];
//				  acumulaI += muestras[i+1]*muestras[i+1];

				  aux_muestra_v =muestras[i] - offset_adc;
				  aux_muestra_i =muestras[i+1] - offset_adc;

				  acumulaV += aux_muestra_v*aux_muestra_v;
				  acumulaI += aux_muestra_i*aux_muestra_i;
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

				  if (RMS_samplesV > 1432){
					  status_proteccionVo = P_MAXIMO;
					  HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 0); //apaga fuente
				  }
				  if (RMS_samplesI > 1392){
					  status_proteccionIo = P_MAXIMO;
					  HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 0); //apaga fuente
				  }
			  }

			  acumulaV = 0;
			  acumulaI = 0;

			  //status_adc = 0;
			  if (status_adc != 3) status_adc = 0;

		  break;
		  case 2:
			  for (uint16_t i = 200; i < 400; i+=2){

//				  muestras[i]	-=offset_adc;
//				  muestras[i+1]	-=offset_adc;

				  aux_muestra_v =muestras[i] - offset_adc;
				  aux_muestra_i =muestras[i+1] - offset_adc;

//				  acumulaV += muestras[i]*muestras[i];
//				  acumulaI += muestras[i+1]*muestras[i+1];

				  acumulaV += aux_muestra_v*aux_muestra_v;
				  acumulaI += aux_muestra_i*aux_muestra_i;
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

				  if (RMS_samplesV > 1432){
					  status_proteccionVo = P_MAXIMO;
					  HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 0); //apaga fuente
				  }
				  if (RMS_samplesI > 1432){
					  status_proteccionIo = P_MAXIMO;
					  HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 0); //apaga fuente
				  }
			  }

			  acumulaV = 0;
			  acumulaI = 0;

//			  status_adc = 0;
			  if (status_adc != 3) status_adc = 0;

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

	  if (last_rango_Io != RANGO_I) {

		  if (status_proteccionIo != P_APAGADO
				  || status_proteccionIo != P_DESHAB) {

			  if (!(status_proteccionIo == P_TEMP
					  || status_proteccionIo == P_OL
					  || status_proteccionIo == P_MAXIMO)) {

				  flag_relay = 1;
				  HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 0); //apaga fuente
			  }
		  }
		  status_proteccionIo = P_OL;
	  }

	  last_rango_Io = RANGO_I;


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
			coefRango_Io = 4847;
			valor_Io = (RMS_samplesI * coefRango_Io / 100);
		break;
		case 2:
			//convierte a float 1A
			coefRango_Io = 960;
			valor_Io = (RMS_samplesI * coefRango_Io / 1000);
		break;
		case 3:
			//convierte a float 2A
			coefRango_Io = 1902;
			valor_Io = (RMS_samplesI * coefRango_Io / 10000);
		break;
		case 4:
			//convierte a float 5A
			coefRango_Io = 4488;
			valor_Io = (RMS_samplesI * coefRango_Io / 10000);
		break;
		case 5:
			//convierte a float 10A
			coefRango_Io = 897;
			valor_Io = (RMS_samplesI * coefRango_Io / 1000);
		break;
		case 6:
			//convierte a float 20A
			coefRango_Io = 1795;
			valor_Io = (RMS_samplesI * coefRango_Io / 1000);
		break;
		case 7:
			//convierte a float 50A
			coefRango_Io = 4488;
			valor_Io = (RMS_samplesI * coefRango_Io / 1000);
		break;

	} //fin switch RANG_I

	valor_Vo = (RMS_samplesV * coefRango_Vo / 100);

} //fin valor_mediciones()

void imprimePantalla(void) {

	valor_mediciones();

//	LCD_GoToxy(5, 0);
//	sprintf(texto, "MEDICIONES");
	//LCD_Print("Titulo");
//	LCD_Print(texto);
	LCD_GoToxy(5, 1);

	LCD_Print("               ");
	LCD_GoToxy(5, 1);
	switch (status_proteccionVo) {
		default:
		break;
		case P_OK:
//				sprintf(texto, "Tension: %3.2f [V]", valor_Vo);
			if (valor_Vo % 100 < 10){
				sprintf(texto, " %d.0%d [V]", valor_Vo / 100, valor_Vo % 100);
			}else{
				sprintf(texto, " %d.%d [V]", valor_Vo / 100, valor_Vo % 100);
			}

		break;
		case P_TEMP:
			sprintf(texto, " Sobretemp.");
		break;
		case P_OL:
			sprintf(texto, " Sobrecarga");
		break;
		case P_APAGADO:
			sprintf(texto, " ---    ");
		break;
		case P_MAXIMO:
			sprintf(texto, " valor maximo");
		break;
	} //fin switch flag_protecV
	LCD_Print(texto);
	LCD_GoToxy(5, 2);
	LCD_Print("               ");
	LCD_GoToxy(5, 2);
	switch (status_proteccionIo) {
		default:
		break;
		case P_OK:
			if (RANGO_I == 1){
//					sprintf(texto, "Corriente: %5.2f[mA]", valor_Io);
				if (valor_Io % 100 < 10){
					sprintf(texto, " %d.0%d[mA]", valor_Io / 100, valor_Io % 100);
				}else{
					sprintf(texto, " %d.%d[mA]", valor_Io / 100, valor_Io % 100);
				}
			}else if (RANGO_I == 2){
//					sprintf(texto, "Corriente: %4.2f [A]", valor_Io);
				if (valor_Io % 1000 < 100){
					if (valor_Io % 1000 < 10){
						sprintf(texto, " %d.00%d [A]", valor_Io / 10, valor_Io % 1000);
					}else{
						sprintf(texto, " %d.0%d [A]", valor_Io / 10, valor_Io % 1000);
					}
				}else{
					sprintf(texto, " %d.%d [A]", valor_Io / 10, valor_Io % 1000);
				}
			}else{
				if (valor_Io % 100 < 10){
					sprintf(texto, " %d.0%d [A]", valor_Io / 100, valor_Io % 100);
				}else{
					sprintf(texto, " %d.%d [A]", valor_Io / 100, valor_Io % 100);
				}
			}
		break;
		case P_TEMP:
			sprintf(texto, " Sobretemp.");
		break;
		case P_OL:
			sprintf(texto, " Sobrecarga");
		break;
		case P_APAGADO:
			sprintf(texto, " ---    ");
		break;
		case P_MAXIMO:
			sprintf(texto, " valor maximo");
		break;

	} //fin switch flag_protecI
	LCD_Print(texto);
	LCD_GoToxy(5, 3);
	LCD_Print("               ");
	LCD_GoToxy(5, 3);
	sprintf(texto, " %d.%d [deg]", valor_fase / 10, valor_fase % 10);
	LCD_Print(texto);

	refrescaPantalla = 500;

} //fin imprimePantalla ()


void protecciones (void){

	//HAL_Delay(100);
//	HAL_GPIO_TogglePin(LED_Board_GPIO_Port, LED_Board_Pin);
	p_tempVo = HAL_GPIO_ReadPin(P_Temp_Vo_GPIO_Port, P_Temp_Vo_Pin);
	p_tempIo = HAL_GPIO_ReadPin(P_Temp_Io_GPIO_Port, P_Temp_Io_Pin);
	poteVo = HAL_GPIO_ReadPin(Rep_Pote_Vo_GPIO_Port, Rep_Pote_Vo_Pin);
	poteIo = HAL_GPIO_ReadPin(Rep_Pote_Io_GPIO_Port, Rep_Pote_Io_Pin);

	switch (status_proteccionVo) {
		case P_APAGADO:
		case P_DESHAB:

			HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 1); //fuente prendida
			HAL_GPIO_WritePin(HAB_Vo_GPIO_Port, HAB_Vo_Pin, 1); //apaga HAB
			HAL_GPIO_WritePin(LED_Vo_GPIO_Port, LED_Vo_Pin, 0); //y apaga el led

			if (poteVo != 0)
				break; //pote distinto de 0
			if (p_tempVo != 0)
				break; //hay alerta de temperatura
			if (P_OL_Vo == RISE || P_OL_Vo == HIGH_L)
				break; //hay alerta de sobre carga

			if (pulsadorVo == FALL)
				status_proteccionVo = P_OK;
		break;
		case P_OK:

			HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 1); //fuente prendida
			HAL_GPIO_WritePin(HAB_Vo_GPIO_Port, HAB_Vo_Pin, 0); //prende HAB
			HAL_GPIO_WritePin(LED_Vo_GPIO_Port, LED_Vo_Pin, 1); //y prende el led

			if (pulsadorVo == FALL) {
				LCD_Init();
				LCD_GoToxy(5, 0);
				sprintf(texto, "MEDICIONES");
				LCD_Print(texto);
				LCD_GoToxy(0, 1);
				sprintf(texto, "  Vo:");
				LCD_Print(texto);
				LCD_GoToxy(0, 2);
				sprintf(texto, "  Io:");
				LCD_Print(texto);
				LCD_GoToxy(0, 3);
				sprintf(texto, " Phi:");
				LCD_Print(texto);

				status_proteccionVo = P_APAGADO;
				//avisa por pantalla
				break;
			}

			if (p_tempVo != 0) {
				status_proteccionVo = P_TEMP;
				//aviso por pantalla
				break;
			}

			if (P_OL_Vo == RISE || P_OL_Vo == HIGH_L) {
				status_proteccionVo = P_OL;
				flag_relay = 1;
				//aviso por pantalla
				break;
			}

		break;
		case P_TEMP:
		case P_OL:
		case P_MAXIMO:

			HAL_GPIO_WritePin(HAB_Vo_GPIO_Port, HAB_Vo_Pin, 1); //apaga HAB
			HAL_GPIO_WritePin(LED_Vo_GPIO_Port, LED_Vo_Pin, 0); //y apaga el led
			if (flag_relay != 0)
				HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 0); //apaga fuente

			if (p_tempVo != 0)
				break; //hay alerta de temperatura

			if (P_OL_Vo == RISE || P_OL_Vo == HIGH_L) {
				break; //hay alerta de sobre tensión
			}

			if (poteVo != 0)
				break; // pote distinto de 0

			if (pulsadorVo == FALL) {

				if (flag_relay != 0) {
					status_proteccionVo = P_FUENTE;
					transitorioFunete = 1000; //en ms.
				} else {
					status_proteccionVo = P_OK;
				}
			}

		break;
		case P_FUENTE:

			HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 1); //prende fuente

			if (transitorioFunete != 0) {
				break; //fatal para el descuento de 1s.
			}

			if (p_tempVo != 0) {
				break; //hay alerta de temperatura
			}

			if (P_OL_Vo == RISE || P_OL_Vo == HIGH_L) {
				break; //hay alerta de sobre carga
			}

			if (poteVo != 0) {
				break; //pote distinto de 0.
			}

			if (pulsadorVo == FALL) {
				status_proteccionVo = P_OK;
			}

		break;
		default:
		break;

	} //end switch status_proteccionVo

	switch (status_proteccionIo) {
		case P_APAGADO:
		case P_DESHAB:

			HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 1); //fuente prendida
			HAL_GPIO_WritePin(HAB_Io_GPIO_Port, HAB_Io_Pin, 1); //apaga HAB
			HAL_GPIO_WritePin(LED_Io_GPIO_Port, LED_Io_Pin, 0); //y apaga el led

			if (poteIo != 0)
				break; //pote distinto de 0
			if (p_tempIo != 0)
				break; //hay alerta de temperatura
			if (P_OL_Io == RISE || P_OL_Io == HIGH_L)
				break; //hay alerta de sobre carga

			if (pulsadorIo == FALL)
				status_proteccionIo = P_OK;
		break;
		case P_OK:

			HAL_GPIO_WritePin(HAB_Io_GPIO_Port, HAB_Io_Pin, 0); //prende HAB
			HAL_GPIO_WritePin(LED_Io_GPIO_Port, LED_Io_Pin, 1); //y prende el led

			if (pulsadorIo == FALL) {
				LCD_Init();
				LCD_GoToxy(5, 0);
				sprintf(texto, "MEDICIONES");
				LCD_Print(texto);
				LCD_GoToxy(0, 1);
				sprintf(texto, "  Vo:");
				LCD_Print(texto);
				LCD_GoToxy(0, 2);
				sprintf(texto, "  Io:");
				LCD_Print(texto);
				LCD_GoToxy(0, 3);
				sprintf(texto, " Phi:");
				LCD_Print(texto);

				status_proteccionIo = P_APAGADO;
				//avisa por pantalla
				break;
			}

			if (p_tempIo != 0) {
				status_proteccionIo = P_TEMP;
				//aviso por pantalla
				break;
			}

			if (P_OL_Io == RISE || P_OL_Io == HIGH_L) {
				status_proteccionIo = P_OL;
				flag_relay = 1;
				//aviso por pantalla
				break;
			}

		break;
		case P_TEMP:
		case P_OL:
		case P_MAXIMO:

			HAL_GPIO_WritePin(HAB_Io_GPIO_Port, HAB_Io_Pin, 1); //apaga HAB
			HAL_GPIO_WritePin(LED_Io_GPIO_Port, LED_Io_Pin, 0); //y apaga el led
			if (flag_relay != 0)
				HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 0); //apaga fuente

			if (p_tempIo != 0)
				break; //hay alerta de temperatura

			if (P_OL_Io == RISE || P_OL_Io == HIGH_L) {
				break; //hay alerta de sobre tensión
			}

			if (poteIo != 0)
				break; // pote distinto de 0

			if (pulsadorIo == FALL) {

				if (flag_relay != 0) {
					status_proteccionIo = P_FUENTE;
					transitorioFunete = 1000; //en ms.
				} else {
					status_proteccionIo = P_OK;
				}
			}

		break;
		case P_FUENTE:

			HAL_GPIO_WritePin(Relay_Fte_GPIO_Port, Relay_Fte_Pin, 1); //prende fuente

			if (transitorioFunete != 0) {
				break; //fatal para el descuento de 1s.
			}

			if (p_tempIo != 0) {
				break; //hay alerta de temperatura
			}

			if (P_OL_Io == RISE || P_OL_Io == HIGH_L) {
				break; //hay alerta de sobre carga
			}

			if (poteIo != 0) {
				break; //pote distinto de 0.
			}

			if (pulsadorIo == FALL) {
				status_proteccionIo = P_OK;
			}

		break;
		default:
		break;

	} //end switch status_proteccionIo


} //fin protecciones ()

void  update_entradas(void){

	read_pulsadorVo = HAL_GPIO_ReadPin(Pul_hab_Vo_GPIO_Port, Pul_hab_Vo_Pin);
	read_pulsadorIo = HAL_GPIO_ReadPin(Pul_hab_Io_GPIO_Port, Pul_hab_Io_Pin);

	lectura_entradas = 100;

} //fin update_entradas()

void check_entradas (void){

	if (read_pulsadorVo) {
		if (last_pulsadorVo) {
			pulsadorVo = HIGH_L;
		} else {
			pulsadorVo = RISE;
		}
	} else {
		if (last_pulsadorVo) {
			pulsadorVo = FALL;
		} else {
			pulsadorVo = LOW_L;
		}
	} //end if pulsadorVo

	if (read_pulsadorIo) {
		if (last_pulsadorIo) {
			pulsadorIo = HIGH_L;
		} else {
			pulsadorIo = RISE;
		}
	} else {
		if (last_pulsadorIo) {
			pulsadorIo = FALL;
		} else {
			pulsadorIo = LOW_L;
		}
	} //end if pulsadorIo

	last_pulsadorIo = read_pulsadorIo;
	last_pulsadorVo = read_pulsadorVo;

} //fin check_entradas()


/*
 * Funcion: update_proteccion
 * Detalle: lee el las entradas de las protecciones de sobre carga, tras cumplirse
 * 			el tiempo de lectura (condicion externa a la funcion)
 *
 *
 */
void  update_proteccion(void){

	read_OL_Vo = HAL_GPIO_ReadPin(P_OL_Vo_GPIO_Port, P_OL_Vo_Pin);
	read_OL_Io = HAL_GPIO_ReadPin(P_OL_Io_GPIO_Port, P_OL_Io_Pin);

	lectura_proteccion = 10;

} //fin update_entradas()

void check_proteccion (void){

	if (read_OL_Vo) {
			if (last_OL_Vo) {
				P_OL_Vo = HIGH_L;
			} else {
				P_OL_Vo = RISE;
			}
		} else {
			if (last_OL_Vo) {
				P_OL_Vo = FALL;
			} else {
				P_OL_Vo = LOW_L;
			}
		} //end if P_OL_Vo

		if (read_OL_Io) {
			if (last_OL_Io) {
				P_OL_Io = HIGH_L;
			} else {
				P_OL_Io = RISE;
			}
		} else {
			if (last_OL_Io) {
				P_OL_Io = FALL;
			} else {
				P_OL_Io = LOW_L;
			}
		} //end if P_OL_Io

		last_OL_Io = read_OL_Io;
		last_OL_Vo = read_OL_Vo;
} //fin check_proteccion ()

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM2){
		flagTIM2 = 1;
		count_tick ++;
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	status_adc = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//HAL_ADC_Stop_DMA(&hadc1);
	//HAL_TIM_Base_Stop(&htim3); //sincro ADC
//	status_adc = 2;
	status_adc = 3;

	if (hadc->Instance == ADC1){
		flag_adc1 = 1;
	}
	if (hadc->Instance == ADC2){
			flag_adc2 = 1;
		}

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
