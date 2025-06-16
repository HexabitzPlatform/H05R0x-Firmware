/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H05R0.h
 Description: Header file for H05R0 module, a battery management module.
 Module: Manages LiPo battery charging and monitoring using MAX17330.
*/

/* Define to prevent recursive inclusion ***********************************/
#ifndef H05R0_H
#define H05R0_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H05R0_MemoryMap.h"
#include "H05R0_uart.h"
#include "H05R0_adc.h"
#include "H05R0_gpio.h"
#include "H05R0_dma.h"
#include "H05R0_inputs.h"
#include "H05R0_eeprom.h"
#include "MAX17330_reg_address.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H05R0

/* Port-related Definitions */
#define	NUM_OF_PORTS	5
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 

/* Define available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART4
#define _USART6

/* Port-UART Mapping */
#define UART_P1 &huart4
#define UART_P2 &huart2
#define UART_P3 &huart3
#define UART_P4 &huart1
#define UART_P5 &huart6

/* Module-specific Hardware Definitions ************************************/
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

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6

/* GPIO Pin Definition */
#define BAT_ALRT_PIN              GPIO_PIN_0
#define BAT_ALRT_GPIO_PORT        GPIOB
#define BAT_ALRT_EXTI_IRQN        EXTI0_1_IRQn
#define STATUS_LED_PIN            GPIO_PIN_8
#define STATUS_LED_GPIO_PORT      GPIOA
#define VBUS_OUT_EN_PIN           GPIO_PIN_4
#define VBUS_OUT_EN_GPIO_PORT     GPIOA
#define MCU_LDO_EN_PIN            GPIO_PIN_2
#define MCU_LDO_EN_GPIO_PORT      GPIOB
#define OUT_EN_3V3_PIN            GPIO_PIN_12
#define OUT_EN_3V3_GPIO_PORT      GPIOB
#define INPUT_3V3OUT_PG_PIN       GPIO_PIN_13
#define INPUT_3V3OUT_PG_GPIO_PORT GPIOB
#define INPUT_3V3OUT_PG_EXTI_IRQN EXTI4_15_IRQn

/* ADC Pin Definition */
#define CURRENT_SENSE_PIN         GPIO_PIN_0
#define CURRENT_SENSE_GPIO_PORT   GPIOB
#define VBUS_SENSE_PIN            GPIO_PIN_1
#define VBUS_SENSE_GPIO_PORT      GPIOB

/* I2C Pin Definition */
#define MCU_SDA_PIN               GPIO_PIN_6
#define MCU_SDA_GPIO_PORT         GPIOA
#define MCU_SCL_PIN               GPIO_PIN_7
#define MCU_SCL_GPIO_PORT         GPIOA
#define I2C_PORT				  &hi2c2

/* Timer Definition */
#define TIMER_CCR                 TIM1->CCR1

/* Indicator LED */
#define _IND_LED_PORT		      GPIOB
#define _IND_LED_PIN		      GPIO_PIN_15

/* Module-specific Macro Definitions ***************************************/
#define NUM_MODULE_PARAMS	      8
#define TIM_OUT_10MS         	  10u

#define UNSNGD_HALF_WORD_MAX_VAL  0xFFFF
#define UNSNGD_HALF_WORD_MIN_VAL  0x0000
#define TWO_COMPL_VAL_MASK		  0x7FFF

#define MAX_CCR_VALUE   		  20000

#define SENSE_CHARGER_VAL	      0.01
#define GAIN_CHARGER_VAL  		  50

/* Macros definitions */
#define MIN_PERIOD_MS		      100
#define MAX_TIMEOUT_MS		      0xFFFFFFFF
#define MIN_PERIOD_MS			  100
#define STREAM_MODE_TO_PORT       1
#define STREAM_MODE_TO_TERMINAL   2

/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H05R0_OK = 0,
	H05R0_ERR_UNKNOWNMESSAGE,
	H05R0_INV,
	H05R0_UNKMSG,
	H05R0_TMOUT,
	H05R0_MSG_ACK,
	H05R0_COM_ERR,
	H05R0_ERR_TERMINATED,
	H05R0_ERR_WRONGPARAMS,
	H05R0_ERROR = 255
} Module_Status;

/* */
typedef enum {
	BATTERY_VOLTAGE = 0,
	BATTERY_CURRENT,
	BATTERY_POWER,
	BATTERY_TEMP,
	BATTERY_CAPACITY,
	BATTERY_SOC,
	BATTERY_TTE,
	BATTERY_TTF,
	BATTERY_AGE,
	BATTERY_CYCLES,
	BATTERY_INT_RESISTANCE,
	SET_CHARGE_VOLT,
	SET_CHARGE_CURRENT,
} All_Data;

/* Thermistors channels status */
typedef enum {
	EXT_THERM_DIS = 0x00,
	EXT_THERM_EN = 0x01, /* Thermistor 1 is used as battery temperature, Thermistor 2 is used with DieTemp for calculating FETTemp. */
	EXT_THERM1_EN,      /* Thermistor 1 is enabled. FETTemp is copied from DieTemp. */
} ThermConfig;

/* External thermistor Values */
typedef enum {
	EXT_THERM_10K = 0,  /* 10K NTC  */
	EXT_THERM_100K = 1, /* 100K NTC */
} ThermType;

/* */
typedef enum {
	FALSE = 0u, TRUE
} BoolState;

/* Charging Status */
typedef enum {
	CHARGING = 0u, DISCHARGING
} ChargingStatus;

/* Battery Status */
typedef enum {
	BATTARY_EMPTY = 0u, BATTARY_FULL
} BatteryStatus;

/* LDO Enable:
 * To secure feeding for the processor after the charger is separated */
typedef enum {
	DISABLE_OUT = 0u, ENABLE_OUT
} LDOOutputState;

/* Export Module typedef structure */
typedef struct {
	ChargingStatus ChargingStatus;

	uint8_t BatSOC;
	uint8_t BatAge;

	uint16_t BatCycles;

	uint32_t BatTTE;
	uint32_t BatTTF;

	float BatVolt;
	float BatCurrent;
	float BatPower;
	float Temp;
	float BatCapacity;
	float BatIntResistance;
	float SetChargVolt;
	float SetChargCurrent;
} AnalogMeasType;

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

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
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
Module_Status ReadChargerCurrent(float *ChargerCurrent);
Module_Status CheckChargingStatus(void);

Module_Status ReadVBUSVoltage(float *VBUSVolt);
Module_Status EnableVBusOutput(LDOOutputState PinState);
Module_Status Enable3_3Output(LDOOutputState PinState);

Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction);
Module_Status StreamToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction, uint32_t numOfSamples,uint32_t streamTimeout);
Module_Status StreamToTerminal(uint8_t dstPort, All_Data dataFunction, uint32_t numOfSamples, uint32_t streamTimeout);
Module_Status StreamToBuffer(float *buffer, All_Data function, uint32_t Numofsamples, uint32_t timeout);

#endif /* H05R0_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
