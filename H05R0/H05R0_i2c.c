/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H05R0_i2c.c
 Description   : This file provides code for the configuration of the I2C instances.

 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Variables ******************************************************/
I2C_HandleTypeDef hi2c2;
SMBUS_HandleTypeDef hsmbus2;

/* Local Function Prototypes ***********************************************/
void MX_I2C2_Init(void);
void MX_I2C2_SMBUS_Init(void);

/***************************************************************************/
/* Configure I2C ***********************************************************/
/***************************************************************************/
void MX_I2C_Init(void) {
	MX_I2C2_Init();
}

/***************************************************************************/
void MX_I2C_SMBUS_Init(void) {
	MX_I2C2_SMBUS_Init();
}

/***************************************************************************/
/* I2C2 Initialization Function */
void MX_I2C2_Init(void) {

	/* Initialize I2C2 peripheral */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x10B17DB5; // Normal mode (100 kHz)
	hi2c2.Init.OwnAddress1 = 0; // No specific address required for master mode
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // 7-bit addressing mode
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // Disable dual address mode
	hi2c2.Init.OwnAddress2 = 0; // Not used, set to 0
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK; // No mask for second address
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Disable general call
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; // Disable clock stretching
	HAL_I2C_Init(&hi2c2);

	/** Configure Analogue filter */
	HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE); // Enable analog filter

	/** Configure Digital filter */
	HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0); // Digital filter set to 0 (disabled)
}

/***************************************************************************/
void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };
	if (i2cHandle->Instance == I2C2) {

		/* Initializes the peripherals clocks */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
		PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
		HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**I2C2 GPIO Configuration
		 PA6     ------> I2C2_SDA
		 PA7     ------> I2C2_SCL
		 */
		GPIO_InitStruct.Pin = MCU_SDA_Pin | MCU_SCL_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_I2C2;
		HAL_GPIO_Init(MCU_SDA_GPIO_Port, &GPIO_InitStruct);

		/* I2C2 clock enable */
		__HAL_RCC_I2C2_CLK_ENABLE();
	}
}

/***************************************************************************/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2cHandle) {

	if (i2cHandle->Instance == I2C2) {
		/* Peripheral clock disable */
		__HAL_RCC_I2C2_CLK_DISABLE();

		/**I2C2 GPIO Configuration
		 PA6     ------> I2C2_SDA
		 PA7     ------> I2C2_SCL
		 */
		HAL_GPIO_DeInit(MCU_SDA_GPIO_Port, MCU_SDA_Pin);

		HAL_GPIO_DeInit(MCU_SDA_GPIO_Port, MCU_SCL_Pin);

	}
}

/***************************************************************************/
/* I2C2 init function */
void MX_I2C2_SMBUS_Init(void) {

	hsmbus2.Instance = I2C2;
	hsmbus2.Init.Timing = 0x20303E5D;
	hsmbus2.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
	hsmbus2.Init.OwnAddress1 = 2;
	hsmbus2.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
	hsmbus2.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
	hsmbus2.Init.OwnAddress2 = 0;
	hsmbus2.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
	hsmbus2.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
	hsmbus2.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
	hsmbus2.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
	hsmbus2.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_HOST;
	hsmbus2.Init.SMBusTimeout = 0x00008019; //0x00008249
	HAL_SMBUS_Init(&hsmbus2);

	HAL_SMBUS_ConfigDigitalFilter(&hsmbus2, 0);

}

/***************************************************************************/
void HAL_SMBUS_MspInit(SMBUS_HandleTypeDef *smbusHandle) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };
	if (smbusHandle->Instance == I2C2) {

		/* Initializes the peripherals clocks */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
		PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
		HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**I2C2 GPIO Configuration
		 PA6     ------> I2C2_SDA
		 PA7     ------> I2C2_SCL
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_I2C2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = GPIO_PIN_12;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF8_I2C2;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* I2C2 clock enable */
		__HAL_RCC_I2C2_CLK_ENABLE();

	}
}

/***************************************************************************/
void HAL_SMBUS_MspDeInit(SMBUS_HandleTypeDef *smbusHandle) {

	if (smbusHandle->Instance == I2C2) {
		/* Peripheral clock disable */
		__HAL_RCC_I2C2_CLK_DISABLE();

		/**I2C2 GPIO Configuration
		 PA6     ------> I2C2_SDA
		 PA7     ------> I2C2_SCL
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);

	}
}

/***************************************************************************/
/* write n bytes to an I2C register
 * @param1: I2C port handle
 * @param2: Device address to be written
 * @param3: Data to be sent
 * @param4: Data size
 */
Module_Status WriteI2C(I2C_HANDLE *xPort, uint16_t sAddress, uint8_t *pData, uint16_t Size) {
	Module_Status Status = H05R0_ERROR;

	if (NULL != xPort && NULL != pData) {
		taskENTER_CRITICAL();

		if (HAL_OK == HAL_I2C_Master_Transmit(xPort, (uint16_t) sAddress, pData, Size, TIM_OUT_10MS))
			Status = H05R0_OK;
		taskEXIT_CRITICAL();

	} else
		Status = H05R0_ERROR;

	return Status;
}

/***************************************************************************/
/* read n bytes from an I2C device
 * @param1: I2C port handle
 * @param2: Device address to be read
 * @param3: Pointer to buffer to store received data in
 * @param4: Data size
 */
Module_Status ReadI2C(I2C_HANDLE *xPort, uint16_t sAddress, uint8_t *rBuffer, uint16_t Size) {
	Module_Status Status;

	if (NULL != xPort && NULL != rBuffer) {
		taskENTER_CRITICAL();
		if (HAL_OK == HAL_I2C_Master_Receive(xPort, (uint16_t) sAddress, rBuffer, Size, TIM_OUT_10MS))
			Status = H05R0_OK;
		taskEXIT_CRITICAL();

	} else
		Status = H05R0_ERROR;

	return Status;
}

/***************************************************************************/
/* check available devices on I2C bus
 * @param1: I2C port handle
 * @param2: Pointer to buffer of addresses sequence
 * @param3: Pointer to buffer to return devices response in correspond to the addresses
 */
Module_Status CheckI2C(I2C_HANDLE *xPort, uint8_t *addBuffer, uint8_t *rBuffer) {
	uint8_t addCounter = 0;
	uint8_t sAddress = 0;

	for (addCounter = 0; addCounter <= 127; addCounter++) {
		sAddress = (addCounter << 1) | 0;
		addBuffer [addCounter] = sAddress;
		if (HAL_OK == HAL_I2C_Master_Transmit(xPort, sAddress, 0, 1, TIM_OUT_10MS))
			rBuffer [addCounter] = TRUE;

		else
			rBuffer [addCounter] = FALSE;
	}

	return H05R0_OK;
}

/***************************************************************************/
/* write n bytes to an SMBUS register
 * @param1: SMBUS port handle
 * @param2: Device address to be written
 * @param3: Data to be sent
 * @param4: Data size
 */
Module_Status WriteSMBUS(SMBUS_HANDLE *xPort, uint16_t sAddress, uint8_t *pData, uint16_t Size) {
	Module_Status Status = H05R0_ERROR;

	if (NULL != xPort && NULL != pData) {
		if (HAL_OK == HAL_SMBUS_Master_Transmit_IT(xPort, (uint16_t) sAddress, pData, Size,
		SMBUS_FIRST_AND_LAST_FRAME_NO_PEC))
			Status = H05R0_OK;
	} else
		Status = H05R0_ERROR;

	return Status;
}

/***************************************************************************/
/* read n bytes from an SMBUS device
 * @param1: SMBUS port handle
 * @param2: Device address to be read
 * @param3: Pointer to buffer to store received data in
 * @param4: Data size
 */
Module_Status ReadSMBUS(SMBUS_HANDLE *xPort, uint16_t sAddress, uint8_t *rBuffer, uint16_t Size) {
	Module_Status Status;

	if (NULL != xPort && NULL != rBuffer) {
		if (HAL_OK == HAL_SMBUS_Master_Receive_IT(xPort, (uint16_t) sAddress, rBuffer, Size,
		SMBUS_FIRST_AND_LAST_FRAME_NO_PEC))
			Status = H05R0_OK;
	} else
		Status = H05R0_ERROR;

	return Status;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
