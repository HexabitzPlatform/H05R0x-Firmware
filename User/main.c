/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
uint16_t buffer , hello;
uint8_t times , SOC;
float batVolt11;

Module_Status ReadReg(uint16_t regAddress, uint16_t *Buffer, uint8_t NoBytes);
Module_Status WriteReg(uint16_t regAddress, uint16_t Data);

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){

//	WriteReg(0x01D0, 0XC88C);
//
//	WriteConfigsToNV();
//
//	ReadReg(0x01D0, &buffer, 2);

	// put your code here, to run repeatedly.
	while(1){

//		WriteReg(0x01D0, 0XC88C);

//		WriteConfigsToNV();

//		ReadReg(0x01D0, &buffer, 2);

	}
}

/*-----------------------------------------------------------*/
