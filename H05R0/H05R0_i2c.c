/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H05R0_i2c.c
 Description: Configures and manages I2C communication for module H05R0.
 I2C: Initializes I2C2 with 100 kHz, 7-bit addressing, and GPIO pins PA6 (SDA), PA7 (SCL).
 Data: Provides functions for reading, writing, and checking devices on the I2C bus.
*/

/* Includes ****************************************************************/
#include "BOS.h"
#include "H05R0_i2c.h"

/* Exported Variables ******************************************************/
I2C_HandleTypeDef hi2c2;

/* Local Function Prototypes ***********************************************/
void MX_I2C2_Init(void);

/***************************************************************************/
/* Configure I2C ***********************************************************/
/***************************************************************************/
void MX_I2C_Init(void) {
	MX_I2C2_Init();
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
		GPIO_InitStruct.Pin = MCU_SDA_PIN | MCU_SCL_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_I2C2;
		HAL_GPIO_Init(MCU_SDA_GPIO_PORT, &GPIO_InitStruct);

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
		HAL_GPIO_DeInit(MCU_SDA_GPIO_PORT, MCU_SDA_PIN);

		HAL_GPIO_DeInit(MCU_SDA_GPIO_PORT, MCU_SCL_PIN);
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
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
