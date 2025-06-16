/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H05R0_adc.c
 Description: Configures and manages ADC instances for module H05R0.
 ADC: Initializes ADC1 with 12-bit resolution for channels 8 and 9 on pins PB0 and PB1.
 Data: Provides functions for channel selection and reading ADC values.
*/

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Variables ******************************************************/
ADC_HandleTypeDef hadc1;

/* Local Function Prototypes ***********************************************/
void MX_ADC1_Init(void);

/***************************************************************************/
/* Configure ADC ***********************************************************/
/***************************************************************************/
void MX_ADC_Init(void) {
	MX_ADC1_Init();
}

/***************************************************************************/
/* ADC init function */
void MX_ADC1_Init(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

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

	/* Configure Regular Channel */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/***************************************************************************/
void HAL_MspInit(void) {

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	/* System interrupt init*/
	/* Configure the internal voltage reference buffer voltage scale */
	HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE1);

	/* Enable the Internal Voltage Reference buffer */
	HAL_SYSCFG_EnableVREFBUF();

	/* Configure the internal voltage reference buffer high impedance mode */
	HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);

	/* Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral */
	HAL_SYSCFG_StrobeDBattpinsConfig(SYSCFG_CFGR1_UCPD1_STROBE | SYSCFG_CFGR1_UCPD2_STROBE);

}

/***************************************************************************/
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	if (adcHandle->Instance == ADC1) {
		/* ADC1 clock enable */
		__HAL_RCC_ADC_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/**ADC1 GPIO Configuration
		 PB0     ------> ADC1_IN8
		 PB1     ------> ADC1_IN9
		 */
		GPIO_InitStruct.Pin = CURRENT_SENSE_PIN | VBUS_SENSE_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(VBUS_SENSE_GPIO_PORT, &GPIO_InitStruct);

	}
}

/***************************************************************************/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle) {
	if (adcHandle->Instance == ADC1) {
		/* Peripheral clock disable */
		__HAL_RCC_ADC_CLK_DISABLE();

		/**ADC1 GPIO Configuration
		 PB0     ------> ADC1_IN8
		 PB1     ------> ADC1_IN9
		 */
		HAL_GPIO_DeInit(CURRENT_SENSE_GPIO_PORT, CURRENT_SENSE_PIN);
		HAL_GPIO_DeInit(VBUS_SENSE_GPIO_PORT, VBUS_SENSE_PIN);
	}
}

/***************************************************************************/
void SelectADCChannel(ADC_Channel ADC_Channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/* Configure for the selected ADC regular channel its corresponding rank
	 * in the sequencer and its sample time.
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

/***************************************************************************/
BOS_Status ReadADCValue(ADC_HandleTypeDef *hadc, ADC_Channel ADC_Channel, uint32_t *ADC_Value, uint32_t Timeout) {
	BOS_Status Status = BOS_ERROR;

	SelectADCChannel(ADC_Channel);
	/* Calibrate The ADC On Power-Up For Better Accuracy */
	HAL_ADCEx_Calibration_Start(hadc);

	/* Start ADC Conversion */
	if (HAL_OK == HAL_ADC_Start(hadc))
		Status = BOS_OK;
	 else
		return BOS_ERROR;

	/* Poll ADC1 Perihperal & TimeOut */
	HAL_ADC_PollForConversion(hadc, Timeout);

	/* Read The ADC Conversion Result */
	*ADC_Value = HAL_ADC_GetValue(hadc);

	/* Stop ADC Conversion */
	HAL_ADC_Stop(hadc);

	return Status;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
