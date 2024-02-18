/*
 BitzOS (BOS) V0.3.0 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
float batVolt;
float batCurrent;
float batPower;
float Temp;
float batCapacity;
uint8_t batSOC;
uint32_t batTTE;
uint32_t batTTF;
uint8_t batAge;
uint16_t batCycles;
float batIntResistance;
float setChargVolt;
float setChargCurrent;
AnalogMeasType analMeasurements;
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
//		ReadCellVoltage(&batVolt);
//		ReadCellCurrent(&batCurrent);
//		 ReadCellPower(&batPower);
//		ReadTemperature(&Temp);
//		ReadCellCapacity(&batCapacity);
//		 ReadCellStateOfCharge(&batSOC);
//		 ReadCellEstimatedTTE(&batTTE);
//		 ReadCellEstimatedTTF(&batTTF);
//		 ReadCellAge(&batAge);
//		 ReadCellCycles(&batCycles);
//		 ReadCellCalInterRes(&batIntResistance);
//		 ReadSetChargVoltage(&setChargVolt);
//		 ReadSetChargCurrent(&setChargCurrent);
//		ReadAllAnalogMeasurements(&analMeasurements);

	}
}

/*-----------------------------------------------------------*/
