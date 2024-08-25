/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H01R0_timers.c
 Description   : Peripheral timers setup source file.

 Required MCU resources :

 >> Timer 16 for micro-sec delay.
 >> Timer 17 for milli-sec delay.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*----------------------------------------------------------------------------*/
/* Configure Timers                                                              */
/*----------------------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStruct = {0};
TIM_HandleTypeDef htim16; /* micro-second delay counter */
TIM_HandleTypeDef htim17; /* milli-second delay counter */

TIM_HandleTypeDef htim3;  /* PWM Special Timer - Charging Indicator */
TIM_HandleTypeDef htim14; /* Special Timer Interrupt every 30 mS */

//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void MX_TIM3_Init(void);

/*  Micro-seconds timebase init function - TIM14 (16-bit)
 */
void TIM_USEC_Init(void){
	/* Peripheral clock enable */
	  __TIM16_CLK_ENABLE();
	  
      /* Peripheral configuration */
	  htim16.Instance = TIM16;
	  htim16.Init.Prescaler = 47;
	  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim16.Init.Period = 0XFFFF;
	  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim16.Init.RepetitionCounter = 0;
	  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  HAL_TIM_Base_Init(&htim16);

	  HAL_TIM_Base_Start(&htim16);

}

/*-----------------------------------------------------------*/

/*  Milli-seconds timebase init function - TIM15 (16-bit)
 */
void TIM_MSEC_Init(void){
	
	/* Peripheral clock enable */
	  __TIM17_CLK_ENABLE();
	  
	  /* Peripheral configuration */
	  htim17.Instance = TIM17;
	  htim17.Init.Prescaler = 47999;
	  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim17.Init.Period = 0xFFFF;
	  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim17.Init.RepetitionCounter = 0;
	  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  HAL_TIM_Base_Init(&htim17);

	  HAL_TIM_Base_Start(&htim17);
}

/*-----------------------------------------------------------*/

/* TIM3 init function */
void MX_TIM3_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 24000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim3);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

	HAL_TIM_PWM_Init(&htim3);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**TIM3 GPIO Configuration
	 PB4     ------> TIM3_CH1
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

}

/*-----------------------------------------------------------*/

/* TIM14 init function */
void MX_TIM14_Init(void) {

	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 1000 - 1;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 1600 - 1; // 1600-->30ms  960-->20ms  14400-->300ms
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim14);

	HAL_TIM_Base_Start_IT(&htim14);

}

/*-----------------------------------------------------------*/

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
  else if(tim_baseHandle->Instance==TIM14)
    {
      /* TIM14 clock enable */
      __HAL_RCC_TIM14_CLK_ENABLE();

      /* TIM14 interrupt Init */
      HAL_NVIC_SetPriority(TIM14_IRQn, 3, 0);
      HAL_NVIC_EnableIRQ(TIM14_IRQn);

    }
}

/*-----------------------------------------------------------*/

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  else if(tim_baseHandle->Instance==TIM14)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM14_CLK_DISABLE();

    /* TIM14 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM14_IRQn);
  }
}
/*-----------------------------------------------------------*/

/* --- Load and start micro-second delay counter --- 
 */
void StartMicroDelay(uint16_t Delay){
	uint32_t t0 =0;
	
	portENTER_CRITICAL();
	
	if(Delay){
		t0 =htim16.Instance->CNT;
		
		while(htim16.Instance->CNT - t0 <= Delay){};
	}

	portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

/* --- Load and start milli-second delay counter --- 
 */
void StartMilliDelay(uint16_t Delay){
	uint32_t t0 =0;
	
	portENTER_CRITICAL();
	
	if(Delay){
		t0 =htim17.Instance->CNT;
		
		while(htim17.Instance->CNT - t0 <= Delay){};
	}

	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
