/*
 BitzOS (BOS) V0.3.0 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H05R0_adc.c
 Description   : This file provides code for the configuration
 of the ADC instances.

 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H05R0_adc.h"

/* USER CODE BEGIN 0 */
ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.LowPowerAutoPowerOff = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
    hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
    HAL_ADC_Init(&hadc1);




  /** Configure Regular Channel
  */
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_MspInit(void)
{
	  /* USER CODE BEGIN MspInit 0 */

	  /* USER CODE END MspInit 0 */

	  __HAL_RCC_SYSCFG_CLK_ENABLE();
	  __HAL_RCC_PWR_CLK_ENABLE();

	  /* System interrupt init*/

	  /** Configure the internal voltage reference buffer voltage scale
	  */
	  HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE1);

	  /** Enable the Internal Voltage Reference buffer
	  */
	  HAL_SYSCFG_EnableVREFBUF();

	  /** Configure the internal voltage reference buffer high impedance mode
	  */
	  HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);

	  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
	  */
	  HAL_SYSCFG_StrobeDBattpinsConfig(SYSCFG_CFGR1_UCPD1_STROBE | SYSCFG_CFGR1_UCPD2_STROBE);

	  /* USER CODE BEGIN MspInit 1 */

	  /* USER CODE END MspInit 1 */
}
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
    GPIO_InitStruct.Pin = CHARGER_CURRENT_SENSE_Pin|VBUS_SENSE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
    HAL_GPIO_DeInit(GPIOB, CHARGER_CURRENT_SENSE_Pin|VBUS_SENSE_Pin);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void SelectADCChannel(ADC_Channel ADC_Channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	if (ADC_Channel == 8) {
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	}
	if (ADC_Channel == 9) {

		sConfig.Channel = ADC_CHANNEL_9;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	}

}

Module_Status ReadADCValue(ADC_HandleTypeDef *hadc,ADC_Channel ADC_Channel,uint32_t *ADC_Value, uint32_t Timeout)
{
	Module_Status Status=H05R0_ERROR;

	  SelectADCChannel(ADC_Channel);
	   // Calibrate The ADC On Power-Up For Better Accuracy
	  HAL_ADCEx_Calibration_Start(hadc);
	  // Start ADC Conversion
	  if (HAL_OK ==HAL_ADC_Start(hadc))
	  {
	      Status=H05R0_OK;
	  }
	  else
	  {
	      Status=H05R0_ERROR;
	  }
	  // Poll ADC1 Perihperal & TimeOut
	  HAL_ADC_PollForConversion(hadc, Timeout);
	  // Read The ADC Conversion Result
	  *ADC_Value = HAL_ADC_GetValue(hadc);
	  // Stop ADC Conversion
	  HAL_ADC_Stop(hadc);

	  return Status;
}
/* USER CODE END 1 */



/* ADC init function */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
