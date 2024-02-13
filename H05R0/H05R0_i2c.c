/*
 BitzOS (BOS) V0.3.0 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H05R0_i2c.c
 Description   : This file provides code for the configuration
 of the I2C instances.

 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* USER CODE BEGIN 0 */
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN 1 */
SMBUS_HandleTypeDef hsmbus2;

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing =0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
 HAL_I2C_Init(&hi2c2);

  /** Configure Analogue filter
  */
   HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);


  /** Configure Digital filter
  */
 HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);

  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PA6     ------> I2C2_SDA
    PA7     ------> I2C2_SCL
    */
    GPIO_InitStruct.Pin = MCU_SDA_Pin|MCU_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_I2C2;
    HAL_GPIO_Init(MCU_SDA_GPIO_Port, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PA6     ------> I2C2_SDA
    PA7     ------> I2C2_SCL
    */
    HAL_GPIO_DeInit(MCU_SDA_GPIO_Port, MCU_SDA_Pin);

    HAL_GPIO_DeInit(MCU_SDA_GPIO_Port, MCU_SCL_Pin);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* I2C2 init function */
void MX_I2C2_SMBUS_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hsmbus2.Instance = I2C2;
  hsmbus2.Init.Timing =0x20303E5D;
  hsmbus2.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
  hsmbus2.Init.OwnAddress1 = 2;
  hsmbus2.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
  hsmbus2.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
  hsmbus2.Init.OwnAddress2 = 0;
  hsmbus2.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
  hsmbus2.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
  hsmbus2.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
  hsmbus2.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
  hsmbus2.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
  hsmbus2.Init.SMBusTimeout =0x00008019;//0x00008249
  HAL_SMBUS_Init(&hsmbus2);


  /** configuration Alert Mode
  */
  HAL_SMBUS_EnableAlert_IT(&hsmbus2);

  /** Configure Digital filter
  */
  HAL_SMBUS_ConfigDigitalFilter(&hsmbus2, 0);

  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

void HAL_SMBUS_MspInit(SMBUS_HandleTypeDef* smbusHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(smbusHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);


    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PA6     ------> I2C2_SDA
    PA7     ------> I2C2_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_I2C2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_SMBUS_MspDeInit(SMBUS_HandleTypeDef* smbusHandle)
{

  if(smbusHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PA6     ------> I2C2_SDA
    PA7     ------> I2C2_SCL
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);


  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}




/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
