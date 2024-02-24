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

		SampletoPort(1, 1,batVolt);
		SampletoPort(2, 2,batVolt);
		Delay_ms(1000);
		SampletoPort(1, 1,batCurrent);
		SampletoPort(2, 2,batCurrent);
		Delay_ms(1000);
		SampletoPort(1, 1,batPower);
		SampletoPort(2, 2,batPower);
		Delay_ms(1000);
		SampletoPort(1, 1,Temp);
		SampletoPort(2, 2,Temp);
		Delay_ms(1000);
		SampletoPort(1, 1,batCapacity);
		SampletoPort(2, 2,batCapacity);
		Delay_ms(1000);
		SampletoPort(1, 1,batSOC);
		SampletoPort(2, 2,batSOC);
		Delay_ms(1000);
		SampletoPort(1, 1,batAge);
		SampletoPort(2, 2,batAge);
		Delay_ms(1000);
		SampletoPort(1, 1,batCycles);
		SampletoPort(2, 2,batCycles);
		Delay_ms(1000);

	}
}

/*-----------------------------------------------------------*/
