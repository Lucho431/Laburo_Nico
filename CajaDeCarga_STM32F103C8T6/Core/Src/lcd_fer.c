/*
===============================================================================
 Nombre      : lcd.c
 Autor	     : Cátedra: Técnicas Digitales II - UTN FRH
 Versión     : 2.0
 Fecha 		 : Junio 2014
 Descripción : Contiene las funciones básicas para controlar un display HD44780
===============================================================================
*/
#include "lcd_fer.h"

uint32_t usTicks;


/* Inicializa al display mediante instrucciones en modo 4-bit */
void LCD_Init(void)
{
	/* Se configuran a los 6 pines como salidas (en este caso están todos conectados en el puerto 2) */
	//LCD_PORT->DIR |= LCD_D4|LCD_D5|LCD_D6|LCD_D7|LCD_RS|LCD_EN;
	


	/* Se inicializa al Timer 1 */
	//LCD_Tim1Init();

	HAL_GPIO_WritePin(RS_Port, RS_Pin, 0);//LCD_PORT->CLR = LCD_RS;
	HAL_GPIO_WritePin(EN_Port, EN_Pin, 0);//LCD_PORT->CLR = LCD_EN;



	/* Se envían tres nibbles 0x03 con los delays correspondientes */
	HAL_Delay(16);//LCD_usDelay(16000);
	LCD_SendNibble(0x03);
	HAL_Delay(5);//LCD_usDelay(5000);
	LCD_SendNibble(0x03);
	HAL_Delay(1);//LCD_usDelay(200);
	LCD_SendNibble(0x03);

	/* Se cambia a modo 4-bit */
	LCD_SendNibble(0x02);
	HAL_Delay(1);//LCD_usDelay(1000);

	/* Se envían las instrucciones requeridas */
	LCD_SendInstruction(LCD_FUNCTION_SET_4BIT);
	LCD_SendInstruction(LCD_DISPLAY_OFF);
	LCD_SendInstruction(LCD_DISPLAY_CLEAR);
	LCD_SendInstruction(LCD_ENTRY_MODE_SET);
	LCD_SendInstruction(LCD_DISPLAY_ON);
}


/* Envía un nibble al display */
void LCD_SendNibble(uint8_t theNibble)
{
	/* Se coloca cada bit del nibble en el pin correspondiente */
	if (theNibble & 0x01) HAL_GPIO_WritePin(D4_Port, D4_Pin, 1);//(LCD_PORT->SET = LCD_D4);
	else			   	  HAL_GPIO_WritePin(D4_Port, D4_Pin, 0);//(LCD_PORT->CLR = LCD_D4);

	if (theNibble & 0x02) HAL_GPIO_WritePin(D5_Port, D5_Pin, 1);//(LCD_PORT->SET = LCD_D5);
	else			      HAL_GPIO_WritePin(D5_Port, D5_Pin, 0);//(LCD_PORT->CLR = LCD_D5);

	if (theNibble & 0x04) HAL_GPIO_WritePin(D6_Port, D6_Pin, 1);//(LCD_PORT->SET = LCD_D6);
	else			   	  HAL_GPIO_WritePin(D6_Port, D6_Pin, 0);//(LCD_PORT->CLR = LCD_D6);

	if (theNibble & 0x08) HAL_GPIO_WritePin(D7_Port, D7_Pin, 1);//(LCD_PORT->SET = LCD_D7);
	else			      HAL_GPIO_WritePin(D7_Port, D7_Pin, 0);//(LCD_PORT->CLR = LCD_D7);

	HAL_GPIO_WritePin(EN_Port, EN_Pin, 1);//LCD_PORT->SET = LCD_EN;
	for (uint16_t i = 0; i < 720; i++); //HAL_Delay(1);//LCD_usDelay(5);
	HAL_GPIO_WritePin(EN_Port, EN_Pin, 0);//LCD_PORT->CLR = LCD_EN;
	for (uint16_t i = 0; i < 720; i++); //HAL_Delay(1);//LCD_usDelay(5);
}


/* Envía un caracter al display */
void LCD_SendChar(char theChar)
{
	HAL_GPIO_WritePin(RS_Port, RS_Pin, 1);//LCD_PORT->SET = LCD_RS;

	LCD_SendByte(theChar);

	for (uint16_t i = 0; i < 7200; i++);
	//HAL_Delay(1);//LCD_usDelay(50);
}


/* Envía una instrucción al display */
void LCD_SendInstruction(uint8_t theInstruction)
{
	HAL_GPIO_WritePin(RS_Port, RS_Pin, 0);//LCD_PORT->CLR = LCD_RS;

	LCD_SendByte(theInstruction);

	if (theInstruction == LCD_DISPLAY_CLEAR)
		for (uint32_t i = 0; i < 288000; i++); //HAL_Delay(2);//LCD_usDelay(2000);
	else
		for (uint16_t i = 0; i < 7200; i++); //HAL_Delay(1);//LCD_usDelay(50);
}


/* Envía un byte al display */
void LCD_SendByte(uint8_t theByte)
{
	/* Primero se envía la parte alta */
	LCD_SendNibble(theByte >> 4);

	/* Luego se envía la parte baja */
	LCD_SendNibble(theByte);
}


/* Posiciona el cursor en la columna x - fila y */
void LCD_GoToxy(uint8_t x, uint8_t y)
{
	if (y == 0)
    	LCD_SendInstruction(LCD_DDRAM_ADDRESS + LCD_START_LINE1 + x);
    else if (y == 1)
    	LCD_SendInstruction(LCD_DDRAM_ADDRESS + LCD_START_LINE2 + x);
    else if (y == 2)
    	LCD_SendInstruction(LCD_DDRAM_ADDRESS + LCD_START_LINE3 + x);
    else if (y == 3)
    	LCD_SendInstruction(LCD_DDRAM_ADDRESS + LCD_START_LINE4 + x);
}


/* Envía un string al display */
void LCD_Print(char *p)
{
	while(*p != 0){
		LCD_SendChar(*p);
		p++;
	}
}



/* Delay us */
/*
void LCD_usDelay(uint32_t usec)
{
	Chip_TIMER_SetMatch(LPC_TIMER1, 0, usec);
	Chip_TIMER_Reset(LPC_TIMER1);
	Chip_TIMER_Enable(LPC_TIMER1);

	while (!usTicks) __WFI();
	usTicks = 0;

	Chip_TIMER_Disable(LPC_TIMER1);

}
*/

/* Inicializa al Timer 1 utilizado para realizar los delays de us */
/*
void LCD_Tim1Init(void)
{
	Chip_TIMER_Init(LPC_TIMER1);
	Chip_TIMER_PrescaleSet(LPC_TIMER1,
	Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER1) / 1000000 - 1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 0);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 0);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER1, 0);

	Chip_TIMER_SetMatch(LPC_TIMER1, 0, 1000);

	NVIC_EnableIRQ(TIMER1_IRQn);
}
*/

/* Desactiva al Timer 1 */
/*
void LCD_Tim1DeInit(void)
{
	Chip_TIMER_Disable(LPC_TIMER1);
	NVIC_DisableIRQ(TIMER1_IRQn);
}
*/

/* Handler de interrupción del Timer 1 */
/*
void TIMER1_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 0)) {
		usTicks++;
	}
	Chip_TIMER_ClearMatch(LPC_TIMER1, 0);
}

*/
