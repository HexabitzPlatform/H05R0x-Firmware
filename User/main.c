/*
 BitzOS (BOS) V0.3.0 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
float picesSize=2.5;
uint8_t txDataEsp[4]={0};
uint16_t Z=400;
uint32_t q=40000000;

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

	// put your code here, to run repeatedly.
	while(1){

		Sampletoport(1, 1,batVolt);
		Sampletoport(2, 2,batVolt);
		Delay_ms(1000);
		Sampletoport(1, 1,batCurrent);
		Sampletoport(2, 2,batCurrent);
		Delay_ms(1000);
		Sampletoport(1, 1,batPower);
		Sampletoport(2, 2,batPower);
		Delay_ms(1000);
		Sampletoport(1, 1,Temp);
		Sampletoport(2, 2,Temp);
		Delay_ms(1000);
		Sampletoport(1, 1,batCapacity);
		Sampletoport(2, 2,batCapacity);
		Delay_ms(1000);
		Sampletoport(1, 1,batSOC);
		Sampletoport(2, 2,batSOC);
		Delay_ms(1000);
		Sampletoport(1, 1,batAge);
		Sampletoport(2, 2,batAge);
		Delay_ms(1000);
		Sampletoport(1, 1,batCycles);
		Sampletoport(2, 2,batCycles);
		Delay_ms(1000);

	}
}

/*-----------------------------------------------------------*/
