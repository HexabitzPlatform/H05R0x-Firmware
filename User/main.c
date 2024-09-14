/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
uint16_t  buffer;
uint8_t times , SOC , Status;
extern AnalogMeasType AnalogMeasurement;
Module_Status ReadReg(uint16_t regAddress, uint16_t *Buffer, uint8_t NoBytes);
Module_Status WriteReg(uint16_t regAddress, uint16_t Data);
float Data;
IdType ID;
Module_Status ReadID(IdType *BatId);
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

//	WriteReg(PROTECT_CONFIGS_REG_ADD, 0x0000);
//
//	WriteReg(CONFIG_REG_ADD, 0x0080);

//	ReadCellVoltage(&Data);
	// put your code here, to run repeatedly.
	while(1){

//		CheckChargingStatus();
//
//		if (AnalogMeasurement.ChargingStatus == CHARGING )
//			HAL_Delay(200);
//		else
//			HAL_Delay(200);

//		ReadNumOfRemainingWrites(&times);
		ReadID(&ID);

	}
}

/*-----------------------------------------------------------*/
