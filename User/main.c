/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Private variables *******************************************************/
float batVolt=0;
float batCurrent=0;
float batPower=0;
float batTemp=0;
float batCapacity=0;
uint8_t batSOC=0;
uint8_t batAge=0;
uint16_t batCycles=0;
float ChargerCurrent=0;
ChargingStatus StatusCharging;
float VBUSVolt =0;
AllMeasType batMeasurements;
int x=0;
/* Private Function Prototypes *********************************************/

/* Main Function ***********************************************************/
int main(void){

	/* Initialize Module &  BitzOS */
	Module_Init();

	/* Don't place your code here */
	for(;;){
	}
}

/***************************************************************************/
/* User Task */
void UserTask(void *argument){
	Enable3_3Output(1);
	EnableVBusOutput(1);
	/* put your code here, to run repeatedly. */
	while(1){


}
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
