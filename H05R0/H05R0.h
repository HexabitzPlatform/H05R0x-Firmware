/*
 BitzOS (BOS) V0.3.0 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H05R0.h
 Description   : Header file for module H05R0.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H05R0_H
#define H05R0_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H05R0_MemoryMap.h"
#include "H05R0_uart.h"
#include "H05R0_gpio.h"
#include "H05R0_dma.h"
#include "H05R0_inputs.h"
#include "H05R0_eeprom.h"
#include "H05R0_i2c.h"
#include "MAX17330_reg_address.h"

/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H05R0


/* Port-related definitions */
#define	NumOfPorts			5

#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 
//#define _P6

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
//#define _Usart4 1
#define _Usart5 1
#define _Usart6	1


/* Port-UART mapping */

#define P1uart &huart5
#define P2uart &huart2
#define P3uart &huart6
#define P4uart &huart3
#define P5uart &huart1
//#define P6uart &huart4


/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT		GPIOA
#define	USART6_RX_PORT		GPIOA
#define	USART6_AF			GPIO_AF3_USART6

/* Module-specific Definitions */

/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_15

#define NUM_MODULE_PARAMS		1

/* Module GPIO Pinout */
#define BAT_ALRT_Pin GPIO_PIN_0
#define BAT_ALRT_GPIO_Port GPIOB
#define BAT_ALRT_EXTI_IRQn EXTI0_1_IRQn
#define STATUS_LED_Pin GPIO_PIN_4
#define STATUS_LED_GPIO_Port GPIOB

/* Module Special I2C */
#define MCU_SDA_Pin GPIO_PIN_6
#define MCU_SDA_GPIO_Port GPIOA
#define MCU_SCL_Pin GPIO_PIN_7
#define MCU_SCL_GPIO_Port GPIOA

/* Module EEPROM Variables */
// Module Addressing Space 500 - 599
#define _EE_MODULE							500		

/* General purposes functions definitions */
#define _DELAY_MS(TimeOut)       		HAL_Delay(TimeOut)

/* Exported types ------------------------------------------------------------*/

/* Module_Status Type Definition */

typedef enum {
	H05R0_OK =0,				/* Battery charger/gauge OK */
	H05R0_ERR_UnknownMessage,
	H05R0_INV,
	H05R0_UNKMSG,
	H05R0_TMOUT,
	H05R0_MSG_ACK, // A special case used only inside MessageConstructor() to indicate that command has executed successfully.
	H05R0_COM_ERR,
	H05R0_ERROR =255 			/* Battery charger/gauge error */
} Module_Status;

/* Battery charger status type definitions */
typedef enum
{
	EXT_THERM_DIS = 0x00,				/* Thermistors channels are disabled */
	EXT_THERM_EN = 0x01,				/* Thermistor 1 is used as battery temperature, Thermistor 2 is used with DieTemp for calculating FETTemp. */
	EXT_THERM1_EN,						/* Thermistor 1 is enabled. FETTemp is copied from DieTemp. */
}ThermConfig;

typedef enum
{
	EXT_THERM_10K = 0,					/* External thermistor is 10K NTC */
	EXT_THERM_100K = 1,					/* External thermistor is 100K NTC */
}ThermType;

typedef enum
{
	AGE_FORCST_DIS = 0,					/* disable age forecasting */
	AGE_FORCST_EN = 1,					/* enable age forecasting */
}AgeForcast;

typedef enum
{
	VOLT_TEMP_BACK_DIS = 0,				/* disable voltage and temperature backup */
	VOLT_TEMP_BACK_EN = 1,				/* enable voltage and temperature backup */
}VoltTempBack;

typedef enum {
	FALSE = 0u,
	TRUE
} Bool_State;
/* Export Module typedef structure */
typedef struct {
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
}AnalogMeasType;

typedef struct {
	uint16_t ManId[MANFCTR_NAME_SIZE / 2];
	uint16_t DevId[DEVICE_NAME_SIZE / 2];
}IdType;


/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);
extern void ExecuteMonitor(void);



/* -----------------------------------------------------------------------
 |								  APIs							          |  																 	|
/* -----------------------------------------------------------------------
 */
Module_Status ReadID(IdType *BatId);
Module_Status ReadCellVoltage(float *batVolt);
Module_Status ReadCellCurrent(float *batCurrent);
Module_Status ReadCellPower(float *batPower);
Module_Status ReadTemperature(float *Temp);
Module_Status ReadCellCapacity(float *batCapacity);
Module_Status ReadCellStateOfCharge(uint8_t *batSOC);
Module_Status ReadCellEstimatedTTE(uint32_t *batTTE);
Module_Status ReadCellEstimatedTTF(uint32_t *batTTF);
Module_Status ReadCellAge(uint8_t *batAge);
Module_Status ReadCellCycles(uint16_t *batCycles);
Module_Status ReadCellCalInterRes(float *batIntResistance);
Module_Status ReadSetChargVoltage(float *setChargVolt);
Module_Status ReadSetChargCurrent(float *setChargCurrent);
Module_Status ReadAllAnalogMeasurements(AnalogMeasType *analMeasurements);
Module_Status WriteConfigsToNV(uint16_t *pConfigBuffer);
Module_Status ReadNumOfRemainingWrites(uint8_t *remWrites);
Module_Status LockNonVolatileMemory(void);


void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);


/* -----------------------------------------------------------------------
 |								Commands							      |															 	|
/* -----------------------------------------------------------------------
 */


#endif /* H05R0_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
