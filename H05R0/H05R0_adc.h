/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H05R0_adc.h
 Description: Header file for ADC configuration function prototypes.
 ADC: Declares ADC1 handle and functions for initialization and channel reading.
 Types: Defines ADC_Channel enum for channel selection.
*/

/* Define to prevent recursive inclusion ***********************************/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Variables ******************************************************/
extern ADC_HandleTypeDef hadc1;

/* Exported types **********************************************************/
typedef enum {
	ADC_CHANNEL8 = 8,
	ADC_CHANNEL9 = 9,
} ADC_Channel;

/* Exported Functions ******************************************************/
void MX_ADC_Init(void);

void SelectADCChannel(ADC_Channel ADC_Channel);
BOS_Status ReadADCValue(ADC_HandleTypeDef *hadc,ADC_Channel ADC_Channel,uint32_t *ADC_Value, uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
