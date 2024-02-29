/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
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
//	StreamtoPort(1, 1,batVolt, 3, 1000);
//	StreamtoPort(2, 2,batVolt, 3, 1000);

	// put your code here, to run repeatedly.
	while(1){

//		SampletoPort(1, 1,batVolt);
//		SampletoPort(2, 2,batVolt);
//		Delay_ms(1000);


	}
}

/*-----------------------------------------------------------*/
