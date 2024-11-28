/*
 BitzOS (BOS) V0.3.0 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H05R0_adc.h
 Description   :This file contains all the functions prototypes for
 the adc

 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

//* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;
/* Exported types ------------------------------------------------------------*/
typedef enum {
	ADC_CHANNEL8=8,
	ADC_CHANNEL9=9,

}ADC_Channel;

/* USER CODE BEGIN Private defines */
void SelectADCChannel(uint8_t ADC_Channel);
void ReadADCValue(ADC_HandleTypeDef *hadc,uint8_t ADC_Channel,uint32_t *ADC_Value, uint32_t Timeout);

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
