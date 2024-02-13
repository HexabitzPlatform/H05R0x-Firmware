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
/* Module Special Timer */

/* Module Special ADC */

/* Module special parameters */

/* Module Special I2C */

#define MCU_SDA_Pin GPIO_PIN_6
#define MCU_SDA_GPIO_Port GPIOA
#define MCU_SCL_Pin GPIO_PIN_7
#define MCU_SCL_GPIO_Port GPIOA

/* Module EEPROM Variables */
// Module Addressing Space 500 - 599
#define _EE_MODULE							500		


/* Exported macros -----------------------------------------------------------*/
/* Battery Charger/Gauge I2C addresses */
#define I2C_6Ch_W_ADD				0x6C
#define I2C_6Ch_R_ADD				0x6D
#define I2C_16h_W_ADD				0x16
#define I2C_16h_R_ADD				0x17
#define FST_I2C_LMT_ADD				0xFF

/* Battery Charger/Gauge special macros */
#define SENSE_RES_VAL				0.01
#define SENSE_RES_REG_VAL			0x03E8
#define CAP_RESOL_VAL				0.000005/SENSE_RES_VAL
#define PERCENT_RESOL_VAL			256
#define VOLT_RESOL_VAL				0.000078125
#define CUR_RESOL_VAL				0.0000015625/SENSE_RES_VAL
#define TEMP_RESOL_VAL				256
#define POWER_RESOL_VAL				0.0008
#define RES_RESOL_VAL				4096
#define TIME_RESOL_VAL				5.625
#define THRMRES_BCONST_REF_VAL			34
#define THRMRES_CONFIG_REG_VAL		0x71E8
#define MANFCTR_NAME_SIZE			0x0C
#define DEVICE_NAME_SIZE			0x0A
#define MANFCTR_DATA_SIZE			0x1A
#define MANFCTR_INFO_SIZE			0x18
#define SERIAL_NUUM_SIZE			0x08
#define ID_BUF_SIZE					MANFCTR_NAME_SIZE + DEVICE_NAME_SIZE
#define BLOCK_TIME 					7500
#define UPDATE_TIME 				1300
#define RECALL_TIME 				5
#define NUM_1S_1					1
#define NUM_1S_2					3
#define NUM_1S_3					7
#define NUM_1S_4					15
#define NUM_1S_5					31
#define NUM_1S_6					63
#define NUM_1S_7					127
#define NUM_1S_8					255
#define ALRT_EN						4
#define CMD_STAT_ERR_CLEAR			4

/* Alert thresholds values */
#define MIN_VOLT_ALRT_THRE			0x64			/*  minimum alert voltage threshold */
#define MAX_VOLT_ALRT_THRE			0x64			/*  maximum alert voltage threshold */
#define MIN_TEMP_ALRT_THRE			0x10			/*  minimum alert temperature threshold */
#define MAX_TEMP_ALRT_THRE			0x64			/*  maximum alert temperature threshold */
#define MIN_SOC_ALRT_THRE			0x10			/*  minimum alert SOC threshold */
#define MAX_SOC_ALRT_THRE			0x64			/*  maximum alert SOC threshold */
#define MIN_CUR_ALRT_THRE			0x0A			/*  minimum alert current threshold */
#define MAX_CUR_ALRT_THRE			0x64			/*  maximum alert current threshold */

/*  Charger/Gauge registers addresses */
/* Charging status and configurations registers */
#define CHARGE_STATUS_REG_ADD		0x00A3
#define CHARGE_VOLTAGE_REG_ADD		0x002A
#define CHARGE_CURRENT_REG_ADD		0x0028

/* Protection Status and Configurations registers */
#define PROTECT_CONFIGS_REG_ADD		0x01D7
#define PROTECT_STATUS_REG_ADD		0x00D9
#define BATTERY_STATUS_REG_ADD		0x01A8
#define PROTECT_ALERT_REG_ADD		0x00AF

/* Status and Configurations registers */
#define SENSE_RES_REG_ADD			0x01CF			/* application sense resistor value register */
#define PACK_CONFIG_REG_ADD			0x01B5			/* pack configurations register */
#define THERM_CONFIG_REG_ADD		0x01CA			/* external thermistor configuration register */
#define NCONFIG_REG_ADD				0x01B0			/* configuration register */
#define CONFIG_REG_ADD				0x000B			/* configuration register */
#define CONFIG2_REG_ADD				0x00AB			/* configuration2 register */

/* Fuel gauge registers */
#define REP_SOC_REG_ADD				0x0006			/*  reported State Of Charge register */
#define REP_CAP_REG_ADD				0x0005			/*  reported capacity register */
#define FULL_CAP_REG_ADD			0x0010			/*  full capacity reported register */
#define TTE_REG_ADD					0x0011			/*  time to empty register */
#define TTF_REG_ADD					0x0020			/*  time to full register */
#define AGE_REG_ADD					0x0007			/*  age register */
#define CYCLES_REG_ADD				0x0017			/*  charge discharge cycles register */
#define NUM_CYCLES_REG_ADD			0x01A4			/*  number of cycles register */
#define INTERNAL_RES_REG_ADD		0x0014			/*  calculated internal resistance register */
#define VOLT_TEMP_REG_ADD			0x01AA			/* Voltage temperature register */
#define AVG_CAP_REG_ADD				0x001F			/*  average capacity register */
#define AVG_SOC_REG_ADD				0x000E			/*  average State Of Charge register */
#define AGE_FRCST_REG_ADD			0x00B9			/*  age forecast register */
#define NV_CONFIG0_REG_ADD			0x01B8			/* nNVCfg0 register */
#define NV_CONFIG2_REG_ADD			0x01BA			/* nNVCfg2 register */

/* Analog measurements registers */
#define CELL_VOLT_REG_ADD			0x001A			/*  cell voltage register */
#define VOLT_REG_ADD				0x00D7			/*  voltage register */
#define CURRENT_REG_ADD				0x001C			/*  current register */
#define TEMP_REG_ADD				0x001B			/*  temperature register */
#define DIE_TEMP_REG_ADD			0x001B			/*  die temperature register */
#define POWER_REG_ADD				0x00B1			/*  power register */

/* Alert thresholds registers */
#define VOLT_ALRT_THRE_REG_ADD		0x018C			/*  alert voltage thresholds register */
#define TEMP_ALRT_THRE_REG_ADD		0x018D			/*  alert temperature thresholds register */
#define SOC_ALRT_THRE_REG_ADD		0x018F			/*  alert SOC thresholds register */
#define CUR_ALRT_THRE_REG_ADD		0x018E			/*  alert current thresholds register */

/* IDs registers */
#define SERIAL_NUM_REG_ADD			0x011C			/*  charger/gauge serial number register */
#define MANFCTR_NAME_REG_ADD		0x0120			/*  charger/gauge manufacturer number register */
#define DEVICE_NAME_REG_ADD			0x0121			/*  charger/gauge device name register */
#define MANFCTR_DATA_REG_ADD		0x0123			/*  charger/gauge manufacturer data register */
#define MANFCTR_INFO_REG_ADD		0x0170			/*  charger/gauge manufacturer info register */

/* Commands registers */
#define CMD_REG_ADD					0x0060			/*  charger/gauge command register */
#define CMD_STAT_REG_ADD			0x0061			/*  charger/gauge command status register */
#define REM_UPDT_REG_ADD			0x01FD			/*  charger/gauge remaining updates register */


/* Module_Status Type Definition */
typedef enum {
	H05R0_OK =0,
	H05R0_ERR_UnknownMessage,
	H05R0_ERROR =255
} Module_Status;

/* Export Module typedef structure */


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
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);


/* -----------------------------------------------------------------------
 |								Commands							      |															 	|
/* -----------------------------------------------------------------------
 */


#endif /* H05R0_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
