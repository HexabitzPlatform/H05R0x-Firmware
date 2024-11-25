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
uint8_t times , SOC , Status,StateOfCharger=0;
extern AnalogMeasType AnalogMeasurement;
Module_Status ReadReg(uint16_t regAddress, uint16_t *Buffer, uint8_t NoBytes);
Module_Status WriteReg(uint16_t regAddress, uint16_t Data);
float Data,ChargingCurrent=0,ChargingVolt=0;
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

//	WriteConfigsToNV();



	// put your code here, to run repeatedly.
	while(1){
		ReadCellCurrent(&ChargingCurrent);
		ReadCellVoltage(&ChargingVolt);
//		CheckChargingStatus();
		ReadCellStateOfCharge(&StateOfCharger);
//        Delay_ms(1000);
//		if (times == 3){
//		ReadReg(PROTECT_CONFIGS_REG_ADD, &buffer, 2);
//		/* 1-  Write 0x0000 to the CommStat register (0x061) 2 times in a row to unlock Write Protection */
//		WriteReg(CMD_STAT_REG_ADD, 0x0000);
//
//		WriteReg(CMD_STAT_REG_ADD, 0x0000);
//
//		/* 2- Write 0x0300 to the CommStat register (0x061) to set DISOff & CHGOff: bits
//		 * to forcefully turn off DIS FET & CHG FET
//		 * this only work if if nProtCfg.CmOvrdEn is enabled in (1D7h) register
//		 *  */
//		WriteReg(CMD_STAT_REG_ADD, 0x0200);
//		}
//		times = 0;

//		CheckChargingStatus();
//
//		if (AnalogMeasurement.ChargingStatus == CHARGING )
//			HAL_Delay(200);
//		else
//			HAL_Delay(200);

//		ReadReg(0x1D0, &buffer, 2);
//
//		ReadReg(0x1D3, &buffer, 2);
//
//		ReadReg(0x1D8, &buffer, 2);

//		ReadReg(0x0000, &buffer, 2);
//		ReadCellVoltage(&Data);


	}
}

/*-----------------------------------------------------------*/
