/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H05R0_i2c.h
 Description   :This file contains all the functions prototypes for the i2c
 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Variables ******************************************************/
extern I2C_HandleTypeDef        hi2c2;

/* I2C Typedef *************************************************************/
typedef I2C_HandleTypeDef 		I2C_HANDLE;

/* Exported Functions ******************************************************/
void MX_I2C_Init(void);

extern Module_Status CheckI2C(I2C_HANDLE *xPort, uint8_t *addBuffer, uint8_t *rBuffer);
extern Module_Status WriteI2C(I2C_HANDLE *xPort, uint16_t sAddress, uint8_t *pData, uint16_t Size);
extern Module_Status ReadI2C(I2C_HANDLE *xPort, uint16_t sAddress, uint8_t *rBuffer, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
