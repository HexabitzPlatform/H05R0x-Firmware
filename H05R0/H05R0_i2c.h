/*
 BitzOS (BOS) V0.3.0 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H05R0_i2c.h
 Description   :This file contains all the functions prototypes for
 the i2c

 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern I2C_HandleTypeDef hi2c2;
extern SMBUS_HandleTypeDef hsmbus2;

/* USER CODE BEGIN Private defines */
/* I2C Typedef */
typedef I2C_HandleTypeDef 			I2C_HANDLE;
typedef SMBUS_HandleTypeDef			SMBUS_HANDLE;

/* I2C port definitions */
#define I2C_PORT					&hi2c2
#define SMBUS_PORT					&hsmbus2


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
void MX_I2C2_Init(void);
void MX_I2C2_SMBUS_Init(void);


/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
