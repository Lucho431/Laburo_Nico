/*
===============================================================================
 Nombre      : lcd.h
 Autor	     : C�tedra T�cnicas Digitales II - UTN FRH
 Versi�n     : 2.0
 Fecha 		 : Junio 2014
 Descripci�n : Contiene la definici�n de constantes y prototipos de funciones
===============================================================================
*/
#ifndef LCD_H_
#define LCD_H_


#include "main.h"
#include "stdio.h"

	/* Puerto y pines a los que se conecta el display */
	#define D4_Pin 		OUT_D4_Pin
	#define D5_Pin		OUT_D5_Pin
	#define D6_Pin		OUT_D6_Pin
	#define D7_Pin		OUT_D7_Pin
	#define RS_Pin		OUT_RS_Pin
	#define EN_Pin		OUT_EN_Pin
	#define D4_Port		OUT_D4_GPIO_Port
	#define D5_Port		OUT_D5_GPIO_Port
	#define D6_Port		OUT_D6_GPIO_Port
	#define D7_Port		OUT_D7_GPIO_Port
	#define RS_Port		OUT_RS_GPIO_Port
	#define EN_Port		OUT_EN_GPIO_Port


	/* C�digos hexadecimales de las instrucciones utilizadas */
	#define LCD_FUNCTION_SET_4BIT 0x28 // 0b00101000 -> DL=0, N=1, F=0
	#define LCD_DISPLAY_OFF       0x08 // 0b00001000 -> D=0, C=0, B=0
	#define LCD_DISPLAY_CLEAR     0x01 // 0b00000001
	#define LCD_ENTRY_MODE_SET    0x06 // 0b00000110 -> I/D=1, S=0
	#define LCD_DISPLAY_ON        0x0C // 0b00001100 -> D=1 , C=0, B=0

	#define LCD_DDRAM_ADDRESS	  0x80 // 0b10000000
	#define LCD_START_LINE1 	  0x00
	#define LCD_START_LINE2 	  0x40
	#define LCD_START_LINE3 	  0x14
	#define LCD_START_LINE4 	  0x54

	/* Prototipos de funciones */
	void LCD_Init(void);
	void LCD_SendNibble(uint8_t theNibble);
	void LCD_SendChar(char theChar);
	void LCD_SendInstruction(uint8_t theInstruction);
	void LCD_SendByte(uint8_t theByte);
	void LCD_GoToxy(uint8_t x, uint8_t y);
	void LCD_Print(char *p);
	void LCD_usDelay(uint32_t usec);
	void LCD_Tim1Init(void);
	void LCD_Tim1DeInit(void);

#endif /* LCD_H_ */
