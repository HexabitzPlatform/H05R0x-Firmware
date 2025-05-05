/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H05R0.c
 Description   : Source code for module H05R0.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "H05R0_inputs.h"
#include "H05R0_i2c.h"

/* Exported Typedef ******************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

All_Data PortFunction;
All_Data TerminalFunction;
AnalogMeasType AnalogMeasurement;

TimerHandle_t xTimerStream = NULL;
TaskHandle_t LipoChargerTaskHandle = NULL;

/* Private Variables *******************************************************/
uint8_t FullCharge = 0;
uint8_t ChargStatus = 1u;
uint8_t StateOfCharger = 0;
float ChargingVolt = 0.0f;
float ChargingCurrent = 0.0f;

/* Private Variables *******************************************************/
/* Streaming variables */
static bool stopStream = false;         /* Flag to indicate whether to stop streaming process */
uint8_t PortModule = 0u;                /* Module ID for the destination port */
uint8_t PortNumber = 0u;                /* Physical port number used for streaming */
uint8_t StreamMode = 0u;                /* Current active streaming mode (to port, terminal, etc.) */
uint8_t TerminalPort = 0u;              /* Port number used to output data to a terminal */
uint8_t StopeCliStreamFlag = 0u;        /* Flag to request stopping a CLI stream operation */
uint32_t SampleCount = 0u;              /* Counter to track the number of samples streamed */
uint32_t PortNumOfSamples = 0u;         /* Total number of samples to be sent through the port */
uint32_t TerminalNumOfSamples = 0u;     /* Total number of samples to be streamed to the terminal */

/* Global variables for sensor data used in ModuleParam */


float H05R0_batVolt = 0.0f;
float H05R0_batCurrent = 0.0f;
float H05R0_batPower = 0.0f;
float H05R0_Temp = 0.0f;
float H05R0_batCapacity = 0.0f;
uint8_t H05R0_batAge = 0;
uint16_t H05R0_batCycles = 0;/* Module exported parameters ------------------------------------------------*/
/* Exported Typedef */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] = {
    {.ParamPtr = &H05R0_batVolt, .ParamFormat = FMT_FLOAT, .ParamName = "batvolt"},
    {.ParamPtr = &H05R0_batCurrent, .ParamFormat = FMT_FLOAT, .ParamName = "batcurrent"},
    {.ParamPtr = &H05R0_batPower, .ParamFormat = FMT_FLOAT, .ParamName = "batpower"},
    {.ParamPtr = &H05R0_Temp, .ParamFormat = FMT_FLOAT, .ParamName = "temperature"},
    {.ParamPtr = &H05R0_batCapacity, .ParamFormat = FMT_FLOAT, .ParamName = "batcapacity"},
    {.ParamPtr = &H05R0_batAge, .ParamFormat = FMT_UINT8, .ParamName = "batage"},
    {.ParamPtr = &H05R0_batCycles, .ParamFormat = FMT_UINT16, .ParamName = "batcycles"},
};

/* Local Typedef related to stream functions */
typedef void (*SampleToString)(char *, size_t);
typedef void (*SampleToBuffer)(float *buffer);

/* Private Function Prototypes *********************************************/
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
uint8_t ClearROtopology(void);
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift);

/* Local Function Prototypes ***********************************************/
void MX_TIM1_Init(void);
void LipoChargerTask(void *argument);

Module_Status Init_MAX17330(void);
Module_Status WriteConfigsToNV(void);
Module_Status LockNonVolatileMemory(void);
Module_Status MCULDOEnable(LDOOutputState PinState);
Module_Status ReadNumOfRemainingWrites(uint8_t *remWrites);
Module_Status ConvertTwosComplToDec(uint16_t twosComplVal, int16_t *sgnDecimalVal);
Module_Status BAT_ReadIdReg(uint16_t regAddress, uint16_t *Buffer, uint8_t NoBytes);

/* Stream Functions */
void StreamTimeCallback(TimerHandle_t xTimerStream);

Module_Status SampleToTerminal(uint8_t dstPort, All_Data dataFunction) ;

static Module_Status POLLINGSLEEPCLISAFE(uint32_t period, long Numofsamples);
static Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout, SampleToString function);


/* Create CLI commands *****************************************************/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* CLI command structure ***************************************************/
/* CLI command structure : sample */
const CLI_Command_Definition_t SampleCommandDefinition = {
    (const int8_t*)"sample",
    (const int8_t*)"sample:\r\n Syntax: sample [volt]/[curr]/[pow]/[temp]/[cap]/[age]/[cycles].\r\n\r\n",
    SampleSensorCommand,
    1
};

/***************************************************************************/
/* CLI command structure : stream */
const CLI_Command_Definition_t StreamCommandDefinition = {
    (const int8_t*)"stream",
    (const int8_t*)"stream:\r\n Syntax: stream [volt]/[curr]/[pow]/[temp]/[cap]/[age]/[cycles] (Numofsamples) (timeout) [port] [module].\r\n\r\n",
    StreamSensorCommand,
    -1
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct ={0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct ={0};

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
	RCC_OscInitStruct.PLL.PLLN =16; // Multiplication factor for PLL
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port){

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart =GetUart(port);

	if((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)){

		/* make sure that no UART transfer is on-going */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to receive */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart,WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins){

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch(wakeupPins){

		case PA0_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins){

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status StopeCliStreamFlag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the StopeCliStreamFlag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch(wakeupPins){

			case PA0_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
				break;

			case PA2_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
				break;

			case PB5_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
				break;

			case PC13_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
				break;

			case NRST_PIN:
				/* do no thing*/
				break;
		}

		IND_blink(1000);

	}
	else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void){

	HAL_StatusTypeDef flashStatus =HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd =8;
	uint16_t temp =0;

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MAX_NUM_OF_PORTS; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(Array[row - 1][0]){
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,Array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd +=8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
	/* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index =0; index < NumOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j * 8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j =0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j * 4));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	/* Clear the Array */
	memset(Array,0,sizeof(Array));
	N =1;
	myID =0;
	
	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){

	UART_HandleTypeDef *huart =GetUart(port);
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);

}

/***************************************************************************/
/* H05R0 module initialization */
void Module_Peripheral_Init(void) {

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART6_UART_Init();

	LipoGPIOInit();
	MX_I2C_Init();
	MX_ADC_Init();
	MX_TIM1_Init();
	Init_MAX17330();

	MCULDOEnable(ENABLE_OUT);

	/* Circulating DMA Channels ON All Module */
	for (int i = 1; i <= NUM_OF_PORTS; i++) {
		if (GetUart(i) == &huart1) {
			dmaIndex [i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex [i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex [i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex [i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			dmaIndex [i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			dmaIndex [i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

	/* Create module special task (if needed) */
	if (LipoChargerTaskHandle == NULL)
		xTaskCreate(LipoChargerTask, (const char*) "LipoChargerTask", configMINIMAL_STACK_SIZE, NULL,
				osPriorityNormal - osPriorityIdle, &LipoChargerTaskHandle);

	/* Create a timeout software timer StreamSamplsToPort() API */
	xTimerStream = xTimerCreate("StreamTimer", pdMS_TO_TICKS(1000), pdTRUE, (void*) 1, StreamTimeCallback);
}

/***************************************************************************/
/* H05R0 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift) {
	Module_Status result = H05R0_OK;

	switch (code) {
	case CODE_H05R0_CELLVOLTAGE:
		SampleToPort(cMessage [port - 1] [shift], cMessage [port - 1] [1 + shift], BATTERY_VOLTAGE);
		break;

	case CODE_H05R0_CELLCURRENT:
		SampleToPort(cMessage [port - 1] [shift], cMessage [port - 1] [1 + shift], BATTERY_CURRENT);
		break;

	case CODE_H05R0_CELLPOWER:
		SampleToPort(cMessage [port - 1] [shift], cMessage [port - 1] [1 + shift], BATTERY_POWER);
		break;

	case CODE_H05R0_CELLTEMPERATURE:
		SampleToPort(cMessage [port - 1] [shift], cMessage [port - 1] [1 + shift], BATTERY_TEMP);
		break;

	case CODE_H05R0_CELLCAPACITY:
		SampleToPort(cMessage [port - 1] [shift], cMessage [port - 1] [1 + shift], BATTERY_CAPACITY);
		break;

	case CODE_H05R0_CELLAGE:
		SampleToPort(cMessage [port - 1] [shift], cMessage [port - 1] [1 + shift], BATTERY_AGE);
		break;

	case CODE_H05R0_CELLCYCLES:
		SampleToPort(cMessage [port - 1] [shift], cMessage [port - 1] [1 + shift], BATTERY_CYCLES);
		break;

	default:
		result = H05R0_ERR_UNKNOWNMESSAGE;
		break;
	}

	return result;
}

/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART4)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART3)
		return P3;
	else if (huart->Instance == USART1)
		return P4;
	else if (huart->Instance == USART6)
		return P5;
	else if (huart->Instance == USART5)
		return P6;

	return 0;
}

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&StreamCommandDefinition);
}

/***************************************************************************/
/* This function is useful only for input (sensors) modules.
 * @brief: Samples a module parameter value based on parameter index.
 * @param paramIndex: Index of the parameter (1-based index).
 * @param value: Pointer to store the sampled float value.
 * @retval: Module_Status indicating success or failure.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
    Module_Status status = BOS_OK;

    switch (paramIndex) {
        /* Sample battery voltage */
        case 1:
            status = ReadCellVoltage(value);
            *value=20 ;
            break;

        /* Sample battery current */
        case 2:
            status = ReadCellCurrent(value);
            break;

        /* Sample battery power */
        case 3:
            status = ReadCellPower(value);
            break;

        /* Sample temperature */
        case 4:
            status = ReadTemperature(value);
            break;

        /* Sample battery capacity */
        case 5:
            status = ReadCellCapacity(value);
            break;

        /* Sample battery age (convert uint8_t to float) */
        case 6: {
            uint8_t temp = 0;
            status = ReadCellAge(&temp);
            if (status == BOS_OK)
                *value = (float)temp;
            break;
        }

        /* Sample battery cycles (convert uint16_t to float) */
        case 7: {
            uint16_t temp = 0;
            status = ReadCellCycles(&temp);
            if (status == BOS_OK)
                *value = (float)temp;
            break;
        }

        /* Invalid parameter index */
        default:
            status = BOS_ERR_WrongParam;
            break;
    }

    return status;
}


/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/* Formats voltage data into a string for CLI output.
 * cstring: Pointer to the string buffer where data will be stored.
 * maxLen: Maximum length of the string buffer.
 */
void SampleVoltageToString(char *cstring, size_t maxLen) {
    float voltage;
    ReadCellVoltage(&voltage);
    snprintf(cstring, maxLen, "Voltage(V) | %.2f\r\n", voltage);
}

/***************************************************************************/
/* Formats current data into a string for CLI output.
 * cstring: Pointer to the string buffer where data will be stored.
 * maxLen: Maximum length of the string buffer.
 */
void SampleCurrentToString(char *cstring, size_t maxLen) {
    float current;
    ReadCellCurrent(&current);
    snprintf(cstring, maxLen, "Current(A) | %.2f\r\n", current);
}

/***************************************************************************/
/* Formats power data into a string for CLI output.
 * cstring: Pointer to the string buffer where data will be stored.
 * maxLen: Maximum length of the string buffer.
 */
void SamplePowerToString(char *cstring, size_t maxLen) {
    float power;
    ReadCellPower(&power);
    snprintf(cstring, maxLen, "Power(W) | %.2f\r\n", power);
}

/***************************************************************************/
/* Formats temperature data into a string for CLI output.
 * cstring: Pointer to the string buffer where data will be stored.
 * maxLen: Maximum length of the string buffer.
 */
void SampleTemperatureToString(char *cstring, size_t maxLen) {
    float temperature;
    ReadTemperature(&temperature);
    snprintf(cstring, maxLen, "Temp(Celsius) | %.2f\r\n", temperature);
}

/***************************************************************************/
/* Formats capacity data into a string for CLI output.
 * cstring: Pointer to the string buffer where data will be stored.
 * maxLen: Maximum length of the string buffer.
 */
void SampleCapacityToString(char *cstring, size_t maxLen) {
    float capacity;
    ReadCellCapacity(&capacity);
    snprintf(cstring, maxLen, "Capacity(mAh) | %.2f\r\n", capacity);
}

/***************************************************************************/
/* Formats age data into a string for CLI output.
 * cstring: Pointer to the string buffer where data will be stored.
 * maxLen: Maximum length of the string buffer.
 */
void SampleAgeToString(char *cstring, size_t maxLen) {
    uint8_t age;
    ReadCellAge(&age);
    snprintf(cstring, maxLen, "Age(%%) | %u\r\n", age);
}

/***************************************************************************/
/* Formats cycles data into a string for CLI output.
 * cstring: Pointer to the string buffer where data will be stored.
 * maxLen: Maximum length of the string buffer.
 */
void SampleCyclesToString(char *cstring, size_t maxLen) {
    uint16_t cycles;
    ReadCellCycles(&cycles);
    snprintf(cstring, maxLen, "Cycles | %u\r\n", cycles);
}

/* Samples cell voltage data into a buffer.
 * buffer: Pointer to the buffer where voltage data will be stored.
 */
void SampleVoltageBuf(float *buffer) {
    float voltage;
    ReadCellVoltage(&voltage);
    voltage =50 ;
    *buffer = voltage;
}

/***************************************************************************/
/* Samples cell current data into a buffer.
 * buffer: Pointer to the buffer where current data will be stored.
 */
void SampleCurrentBuf(float *buffer) {
    float current;
    ReadCellCurrent(&current);
    *buffer = current;
}

/***************************************************************************/
/* Samples cell power data into a buffer.
 * buffer: Pointer to the buffer where power data will be stored.
 */
void SamplePowerBuf(float *buffer) {
    float power;
    ReadCellPower(&power);
    *buffer = power;
}

/***************************************************************************/
/* Samples temperature data into a buffer.
 * buffer: Pointer to the buffer where temperature data will be stored.
 */
void SampleTemperatureBuf(float *buffer) {
    float temperature;
    ReadTemperature(&temperature);
    *buffer = temperature;
}

/***************************************************************************/
/* Samples cell capacity data into a buffer.
 * buffer: Pointer to the buffer where capacity data will be stored.
 */
void SampleCapacityBuf(float *buffer) {
    float capacity;
    ReadCellCapacity(&capacity);
    *buffer = capacity;
}

/***************************************************************************/
/* Samples cell age data into a buffer.
 * buffer: Pointer to the buffer where age data will be stored.
 */
void SampleAgeBuf(float *buffer) {
    uint8_t age;
    ReadCellAge(&age);
    *buffer = (float)age;
}

/***************************************************************************/
/* Samples cell cycles data into a buffer.
 * buffer: Pointer to the buffer where cycles data will be stored.
 */
void SampleCyclesBuf(float *buffer) {
    uint16_t cycles;
    ReadCellCycles(&cycles);
    *buffer = (float)cycles;
}

/***************************************************************************/
/* Streams sensor data to a buffer.
 * buffer: Pointer to the buffer where data will be stored.
 * Numofsamples: Number of samples to take.
 * timeout: Timeout period for the operation.
 * function: Function pointer to the sampling function (e.g., SampleVoltageBuf, SampleCurrentBuf).
 */
static Module_Status StreamToBuf(float *buffer, uint32_t Numofsamples, uint32_t timeout, SampleToBuffer function) {
    Module_Status status = H05R0_OK;
    uint16_t StreamIndex = 0;
    uint32_t period = timeout / Numofsamples;

    /* Check if the calculated period is valid */
    if (period < MIN_PERIOD_MS)
        return H05R0_ERR_WRONGPARAMS;

    stopStream = false;

    /* Stream data to buffer */
    while ((Numofsamples-- > 0) || (timeout >= MAX_TIMEOUT_MS)) {
        float sample;
        function(&sample);
        buffer[StreamIndex] = sample;
        StreamIndex++;

        /* Delay for the specified period */
        vTaskDelay(pdMS_TO_TICKS(period));

        /* Check if streaming should be stopped */
        if (stopStream) {
            status = H05R0_ERR_TERMINATED;
            break;
        }
    }

    return status;
}
/* Module special task function (if needed) */
void LipoChargerTask(void *argument) {

	int static timer = 0, StopeCliStreamFlag = 0;

	/* Infinite loop */
	for (;;) {

		/* Read Charging Current */
		CheckChargingStatus();

		/* in case the battery is charging */
		if (AnalogMeasurement.ChargingStatus == 0) {
			ReadCellCurrent(&ChargingCurrent);
			ReadCellVoltage(&ChargingVolt);
			ReadCellStateOfCharge(&StateOfCharger);

			if (StateOfCharger == 100) {
				FullCharge = BATTARY_FULL;
				ChargStatus = CHARGING;
				TIMER_CCR = MAX_CCR_VALUE;
			}

			if (StateOfCharger < 99) {
				ChargStatus = CHARGING;
				FullCharge = BATTARY_EMPTY;
			}

			if ((ChargingVolt * ChargingCurrent) >= 0.1 && FullCharge == BATTARY_EMPTY) {
				ChargStatus = CHARGING;
				if (timer < MAX_CCR_VALUE && StopeCliStreamFlag == 0) {
					timer += 200;
					if (timer >= MAX_CCR_VALUE)
						StopeCliStreamFlag = 1;
				} else if (StopeCliStreamFlag == 1) {
					timer -= 200;
					if (timer <= 0)
						StopeCliStreamFlag = 0;
				}
				/* increase CCR */
				TIMER_CCR = timer;
			}

			if ((ChargingVolt * ChargingCurrent) < 0.1 && FullCharge == BATTARY_EMPTY) {
				ChargStatus = DISCHARGING;		//Stop the loads and wait for the current to increase
				TIMER_CCR = 0;
			}

		} else if (AnalogMeasurement.ChargingStatus == 1 && ChargingCurrent < 0) {
			ChargStatus = DISCHARGING;
			FullCharge = BATTARY_EMPTY;
			TIMER_CCR = 0;
		}

		taskYIELD();
	}
}

/***************************************************************************/
/*
 * brief: Callback function triggered by a timer to manage data streaming.
 * param xTimerStream: Handle of the timer that triggered the callback.
 * retval: None
 */
void StreamTimeCallback(TimerHandle_t xTimerStream) {
	/* Increment sample counter */
	++SampleCount;

	/* Stream mode to port: Send samples to port */
	if (STREAM_MODE_TO_PORT == StreamMode) {
		if ((SampleCount <= PortNumOfSamples) || (0 == PortNumOfSamples)) {
			SampleToPort(PortModule, PortNumber, PortFunction);

		} else {
			SampleCount = 0;
			xTimerStop(xTimerStream, 0);
		}
	}
	/* Stream mode to terminal: Export to terminal */
	else if (STREAM_MODE_TO_TERMINAL == StreamMode) {
		if ((SampleCount <= TerminalNumOfSamples) || (0 == TerminalNumOfSamples)) {
			SampleToTerminal(TerminalPort, TerminalFunction);
		} else {
			SampleCount = 0;
			xTimerStop(xTimerStream, 0);
		}
	}
}

/***************************************************************************/
/* write 16-bit data to a Battery charger/gauge register
 * regAddress: register's address to write data at
 * Data: data to be written
 */
Module_Status WriteReg(uint16_t regAddress, uint16_t Data) {
	Module_Status Status = H05R0_OK;
	uint8_t tempBuffer [4] = { 0 };
	uint8_t i2cSize = 0;
	uint8_t i2cSlaveAddress = 0;

	if (FST_I2C_LMT_ADD >= regAddress) {
		tempBuffer [0] = (uint8_t) regAddress;
		tempBuffer [1] = (uint8_t) Data;
		tempBuffer [2] = (uint8_t) (Data >> 8);
		i2cSize = 3;
		i2cSlaveAddress = I2C_6Ch_W_ADD;
	} else {
		tempBuffer [0] = (uint8_t) regAddress;
		tempBuffer [1] = (uint8_t) Data;
		tempBuffer [2] = (uint8_t) (Data >> 8);
		i2cSize = 3;
		i2cSlaveAddress = I2C_16h_W_ADD;
	}

	if (H05R0_OK != WriteI2C(I2C_PORT, i2cSlaveAddress, tempBuffer, i2cSize))
		return H05R0_ERROR;

	return Status;
}

/***************************************************************************/
/* read 16-bit data from a Battery charger/gauge register
 * regAddress: register's address to read data from
 * Buffer: pointer to a buffer to store received data
 * NoBytes: Number of data to be read
 */
Module_Status ReadReg(uint16_t regAddress, uint16_t *Buffer, uint8_t NoBytes) {
	Module_Status Status = H05R0_OK;
	uint8_t tempBuffer [2] = { 0 };
	uint8_t i2cSize = 0;
	uint8_t i2cSlaveWriteAdd = 0;
	uint8_t i2cSlaveReadAdd = 0;

	if (Buffer == NULL)
		return Status;

	if (FST_I2C_LMT_ADD >= regAddress) {
		tempBuffer [0] = (uint8_t) regAddress;
		i2cSize = 1;
		i2cSlaveWriteAdd = I2C_6Ch_W_ADD;
		i2cSlaveReadAdd = I2C_6Ch_R_ADD;
	} else {
		tempBuffer [0] = (uint8_t) regAddress;
		tempBuffer [1] = (uint8_t) (regAddress >> 8);
		i2cSize = 2;
		i2cSlaveWriteAdd = I2C_16h_W_ADD;
		i2cSlaveReadAdd = I2C_16h_R_ADD;
	}

	if (H05R0_OK != WriteI2C(I2C_PORT, i2cSlaveWriteAdd, tempBuffer, i2cSize))
		return H05R0_ERROR;

	if (H05R0_OK != ReadI2C(I2C_PORT, i2cSlaveReadAdd, (uint8_t*) Buffer, NoBytes))
		return H05R0_ERROR;

	return Status;
}

/***************************************************************************/
/* initialize Battery charger/gauge */
Module_Status Init_MAX17330(void) {
	Module_Status Status = H05R0_OK;
	uint16_t tempVar = 0u;
	uint16_t tempBuffer [3] = { 0 };

	/* setting temperature sensing source from external thermistors and their type */
	tempVar = (EXT_THERM_10K << 11) | (EXT_THERM_EN << 12);
	if (H05R0_OK != WriteReg(PACK_CONFIG_REG_ADD, tempVar))
		Status = H05R0_OK;

	/* write sense resistor value to nRSense register */
	if (H05R0_OK != WriteReg(SENSE_RES_REG_ADD, SENSE_RES_REG_VAL))
		return H05R0_ERROR;

	/* write sense resistor B_Const to nThermCfg register */
	if (H05R0_OK != WriteReg(THERM_CONFIG_REG_ADD, THRMRES_CONFIG_REG_VAL))
		return H05R0_ERROR;

	/* enable battery ALRT, write configs to shadow memory */
	/* set the alert voltage thresholds */
	tempVar = MIN_VOLT_ALRT_THRE | (MAX_VOLT_ALRT_THRE << 8);
	if (H05R0_OK != WriteReg(VOLT_ALRT_THRE_REG_ADD, tempVar))
		return H05R0_ERROR;

	tempVar = MIN_CUR_ALRT_THRE | (MAX_CUR_ALRT_THRE << 8);
	if (H05R0_OK != WriteReg(CUR_ALRT_THRE_REG_ADD, tempVar))
		return H05R0_ERROR;

	tempVar = MIN_TEMP_ALRT_THRE | (MAX_TEMP_ALRT_THRE << 8);
	if (H05R0_OK != WriteReg(TEMP_ALRT_THRE_REG_ADD, tempVar))
		Status = H05R0_OK;

	tempVar = MIN_SOC_ALRT_THRE | (MAX_SOC_ALRT_THRE << 8);
	if (H05R0_OK != WriteReg(SOC_ALRT_THRE_REG_ADD, tempVar))
		return H05R0_ERROR;

	/* enable alerts by setting Aen bit */
	tempVar = ALRT_EN;
	if (H05R0_OK != WriteReg(CONFIG_REG_ADD, tempVar))
		return H05R0_ERROR;

	if (H05R0_OK != ReadReg(PACK_CONFIG_REG_ADD, &tempBuffer [0], sizeof(tempBuffer [0])))
		return H05R0_ERROR;

	if (H05R0_OK != ReadReg(SENSE_RES_REG_ADD, &tempBuffer [1], sizeof(tempBuffer [0])))
		return H05R0_ERROR;

	if (H05R0_OK != ReadReg(THERM_CONFIG_REG_ADD, &tempBuffer [2], sizeof(tempBuffer [0])))
		return H05R0_ERROR;

	return Status;
}

/***************************************************************************/
/* read battery cell calculated internal resistance
 * twosComplVal: the raw value to be processed
 * sgnDecimalVal: pointer to a buffer to store converted signed value
 */
Module_Status ConvertTwosComplToDec(uint16_t twosComplVal, int16_t *sgnDecimalVal) {

	if (sgnDecimalVal == NULL
			&& ((UNSNGD_HALF_WORD_MAX_VAL < twosComplVal) || (UNSNGD_HALF_WORD_MIN_VAL > twosComplVal)))
		return H05R0_INV;

	if (TWO_COMPL_VAL_MASK < twosComplVal)
		*sgnDecimalVal = -((~twosComplVal) + 1);
	else
		*sgnDecimalVal = twosComplVal;

	return H05R0_OK;
}


/***************************************************************************/
/*
 * @brief: write configurations to non-volatile memory registers
 * @param1: pointer to a buffer storing the configurations to be written
 * @retval: status
 */
Module_Status WriteConfigsToNV(void) {
	Module_Status Status = H05R0_OK;
	uint16_t tempVar = 0u;
	uint16_t POR_CMD = 1u;

	/* 1-  Write 0x0000 to the CommStat register (0x061) 2 times in a row to unlock Write Protection */
	if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x0000))
		return H05R0_COM_ERR;

	if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x0000))
		return H05R0_COM_ERR;

	/* 2- write the desired configs to their addresses at the shadow RAM */
	/* Set the under voltage protection to: 3.3 v */
	WriteReg(0x01D0, 0XDC8C); // Under voltage Protection Threshold = 3.33 v

	/* Set the maximum discharging current to: 8 A default value = 0x4BB5  */
//	WriteReg(0x01D3, 0x4B9C); // 0x9c --> 8 A
	WriteReg(0x01D3, 0x4B83); // 0x83 --> 10 A

	/* Set the maximum charging current to: 2 A default value = 0x7F4B */
//	WriteReg(0x01D8, 0X64FF);  // R = 5 m OM
	WriteReg(0x01D8, 0X3CFF);  // R = 3.33 m OM

	/* Set CmOvrdEn.bit = 1 to allows the ChgOff and DisOff bits in CommStat to be
	 * set by I2C communication to turn off the protection FETs
	 * default value = 0x2800 .
	 * Set the value to 0x2C00 */
//	WriteReg(PROTECT_CONFIGS_REG_ADD, 0x2C00);
	/* 3- Write 0x0000 to the CommStat register (0x061) to clear CommStat.NVError bit */
	if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x0000))
		return H05R0_COM_ERR;

	/* 4- Write 0xE904 to the Command register 0x060 to initiate a block copy  */
	if (H05R0_OK != WriteReg(CMD_REG_ADD, 0xE904))
		return H05R0_COM_ERR;

	/* 5- Wait tBLOCK for the copy to complete.  */
	Delay_ms(7000);

	/* 6- Check the CommStat.NVBusy bit. Continue to wait while CommStat.NVBusy = 1
	 * If NVBusy is 1, keep waiting; if it becomes 0, proceed */
	if (H05R0_OK != ReadReg(CMD_STAT_REG_ADD, &tempVar, sizeof(tempVar)))
		return H05R0_COM_ERR;

	/* 7- Check the CommStat.NVError bit. If set, return to Step 2 to repeat the process.
	 *  If clear, continue */
	if (H05R0_OK != ReadReg(CMD_STAT_REG_ADD, &tempVar, sizeof(tempVar)))
		return H05R0_COM_ERR;

	/* 8- Write 0x000F to the Command register 0x060 to POR the IC */
	if (H05R0_OK != WriteReg(CMD_REG_ADD, 0x000F))
		return H05R0_COM_ERR;

	/* 9- wait 10msec */
	Delay_ms(10);

	/* 10- Verify all of the nonvolatile memory locations are recalled correctly
	 * Check that all nonvolatile memory locations contain the expected values. */
	ReadReg(0x01D0, &tempVar, 2);

	ReadReg(0x01D3, &tempVar, 2);

	ReadReg(0x01D8, &tempVar, 2);

	/* 11- Write 0x0000 to the CommStat register (0x061) 3 times in a row to
	 *  unlock Write Protection and clear NVError bit */
	if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x0000))
		return H05R0_COM_ERR;

	if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x0000))
		return H05R0_COM_ERR;

	if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x0000))
		return H05R0_COM_ERR;

	/* 12- Write 0x8000 to Config2 register 0x0AB to reset firmware */
	if (H05R0_OK != WriteReg(CONFIG2_REG_ADD, 0x8000))
		return H05R0_COM_ERR;

	/* 13- Wait for POR_CMD bit (bit 15) of the Config2 register to
	 *  be cleared to indicate POR sequence is complete */
	while (POR_CMD != 0) {

		if (H05R0_OK != ReadReg(CONFIG2_REG_ADD, &tempVar, sizeof(tempVar)))
			return H05R0_COM_ERR;

		POR_CMD = (tempVar | 0X00000) >> 15;
	}

	/* 14- Write 0x00F9 to the CommStat register (0x061) 2 times in a row to lock Write Protection */
	if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x00F9))
		return H05R0_COM_ERR;

	if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x00F9))
		return H05R0_COM_ERR;

	return Status;
}

/***************************************************************************/
/* read number of remaining non-volatile memory writes
 * @param1: pointer to a buffer to store received data
 */
Module_Status ReadNumOfRemainingWrites(uint8_t *updatesRemaining) {
	Module_Status Status = H05R0_OK;
	uint16_t tempVar = 0u;
	uint8_t updatesUsed = 0, index = 0, combinedData = 0;
	uint8_t totalUpdates = 7;

	/* the following steps are implemented as stated in the data sheet */
	/* 1- write 0xE29B to the Command register 0x060 to initiate a block copy */
	if (H05R0_OK != WriteReg(CMD_REG_ADD, 0xE29B))
		return H05R0_COM_ERR;

	/* 2- wait t_BLOCK for the copy to complete */
	Delay_ms(RECALL_TIME);

	/* 3- read memory address 1FDh */
	if (H05R0_OK != ReadReg(REM_UPDT_REG_ADD, &tempVar, 2))
		return H05R0_COM_ERR;

	/* 4- decode the received data */
	/* Logical OR of upper and lower bytes:
	 * (data >> 8): This effectively discards the lower byte, leaving only the upper byte.
	 * (data & 0xFF): This operation retains only the lower byte and sets the upper byte to zero.
	 * The result is an 8-bit value (combinedData) where the upper byte comes from the original data
	 * (shifted right) and the lower byte comes from the original data (masked with 0xFF).
	 */
	combinedData = (tempVar >> 8) | (tempVar & 0xFF);

	// Count the number of 1s (updates used)
	for (index = 0; index < 7; ++index) {
		if (combinedData & (1 << index)) {
			updatesUsed++;
		}
	}

	/* Calculate updates remaining */
	*updatesRemaining = totalUpdates - updatesUsed;

	return Status;
}

/***************************************************************************/
/* read number of remaining non-volatile memory writes */
Module_Status LockNonVolatileMemory(void) {
	bool csErrorBit = 0;
	uint16_t tempVar = 0u;

	/* read command status register */
	if (H05R0_OK != ReadReg(CMD_STAT_REG_ADD, &tempVar, 1))
		return H05R0_COM_ERR;

	//csErrorBit = tempVar & 32;

	/* repeat the process till CommStat.NVError bit is zero */
	while (TRUE == csErrorBit) {
		/* write 0x0000 to the CommStat register (0x61) two times in a row to unlock write protection */
		if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x0000))
			return H05R0_COM_ERR;

		if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x0000))
			return H05R0_COM_ERR;

		/* write 0x0000 to the CommStat register (0x61) one more time to clear CommStat.NVError bit */
		if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x0000))
			return H05R0_COM_ERR;

		/* write 0x6AXX to the Command register 0x060 to lock desired blocks */
		if (H05R0_OK != WriteReg(CMD_REG_ADD, 0x6A00))
			return H05R0_COM_ERR;

		/* wait tUPDATE for the copy to complete */
		Delay_ms(UPDATE_TIME);

		/* check the CommStat.NVError bit. If set, repeat the process */
		if (H05R0_OK != ReadReg(CMD_STAT_REG_ADD, &tempVar, 1))
			return H05R0_COM_ERR;

		csErrorBit = tempVar & 32;
	}

	/* write 0x00F9 to the CommStat register (0x61) two times in a row to lock write protection */
	if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x00F9))
		return H05R0_COM_ERR;

	if (H05R0_OK != WriteReg(CMD_STAT_REG_ADD, 0x00F9))
		return H05R0_COM_ERR;

	return 0;
}

/***************************************************************************/
/* MCU LDO Enable To secure feeding for the processor after the charger is separated */
Module_Status MCULDOEnable(LDOOutputState PinState) {
	Module_Status Status = H05R0_OK;

	if (PinState == ENABLE_OUT)
		HAL_GPIO_WritePin(MCU_LDO_EN_GPIO_PORT, MCU_LDO_EN_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(MCU_LDO_EN_GPIO_PORT, MCU_LDO_EN_PIN, GPIO_PIN_RESET);

	return Status;
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
/* read battery cell voltage
 * batVolt: pointer to a buffer to store received data
 */
Module_Status ReadCellVoltage(float *batVolt) {
	Module_Status Status = H05R0_OK;
	uint16_t tempVar = 0u;

	if (batVolt == NULL)
		return H05R0_INV;

	if (H05R0_OK != ReadReg(CELL_VOLT_REG_ADD, &tempVar, sizeof(tempVar)))
		return H05R0_ERROR;

	*batVolt = (float) (VOLT_RESOL_VAL * tempVar);

	return Status;
}

/***************************************************************************/
/* read battery cell current
 * batCurrent: pointer to a buffer to store received data
 */
Module_Status ReadCellCurrent(float *batCurrent) {
	Module_Status Status = H05R0_OK;
	uint16_t tempVar = 0u;
	int16_t tempSingedVar = 0;

	if (batCurrent == NULL)
		return H05R0_INV;

	if (H05R0_OK != ReadReg(CURRENT_REG_ADD, &tempVar, sizeof(tempVar)))
		return H05R0_ERROR;

	/* convert the received value from two's complementary to signed decimal value */
	if (H05R0_OK != ConvertTwosComplToDec(tempVar, &tempSingedVar))
		return H05R0_ERROR;
	else {
		tempSingedVar = tempVar;
		*batCurrent = (float) (CUR_RESOL_VAL * tempSingedVar);
	}

	return Status;
}
/***************************************************************************/
/* Read battery cell power
 * batPower: pointer to a buffer to store received data
 */
Module_Status ReadCellPower(float *batPower)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;
	int16_t tempSingedVar = 0;

	if (batPower == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(POWER_REG_ADD, &tempVar, sizeof(tempVar)))
			Status = H05R0_OK;

	/* convert the received value from two's complementary to signed decimal value */
	if (H05R0_OK == ConvertTwosComplToDec(tempVar, &tempSingedVar))
		*batPower = (float) (POWER_RESOL_VAL * tempSingedVar);

	return Status;
}

/***************************************************************************/
/* read battery cell temperature
 * Temp: pointer to a buffer to store received data
 */
Module_Status ReadTemperature(float *Temp) {
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;
	int16_t tempSingedVar = 0;

	if (Temp == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(TEMP_REG_ADD, &tempVar, sizeof(tempVar)))
		Status = H05R0_OK;

	/* convert the received value from two's complementary to signed decimal value */
	if (H05R0_OK == ConvertTwosComplToDec(tempVar, &tempSingedVar))
		*Temp = (float) (tempSingedVar / TEMP_RESOL_VAL);

	return Status;
}

/***************************************************************************/
/* read battery cell capacity
 * batCapacity: pointer to a buffer to store received data
 */
Module_Status ReadCellCapacity(float *batCapacity) {
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batCapacity == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(FULL_CAP_REG_ADD, &tempVar, sizeof(tempVar)))
		Status = H05R0_OK;

	*batCapacity = (float) (CAP_RESOL_VAL * tempVar);

	return Status;
}

/***************************************************************************/
/* read battery cell state of charge
 * batSOC: pointer to a buffer to store received data
 */
Module_Status ReadCellStateOfCharge(uint8_t *batSOC) {
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batSOC == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(REP_SOC_REG_ADD, &tempVar, sizeof(tempVar)))
		Status = H05R0_OK;

	*batSOC = (uint8_t) (tempVar / PERCENT_RESOL_VAL);

	return Status;
}

/***************************************************************************/
/* read battery time to empty
 * batTTE: pointer to a buffer to store received data
 */
Module_Status ReadCellEstimatedTTE(uint32_t *batTTE) {
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batTTE == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(TTE_REG_ADD, &tempVar, sizeof(tempVar)))
		Status = H05R0_OK;

	*batTTE = (uint32_t) (TIME_RESOL_VAL * tempVar);
	return Status;
}

/***************************************************************************/
/* read battery cell time to full
 * batTTF: pointer to a buffer to store received data
 */
Module_Status ReadCellEstimatedTTF(uint32_t *batTTF) {
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batTTF == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(TTF_REG_ADD, &tempVar, sizeof(tempVar)))
		Status = H05R0_OK;

	*batTTF = (uint32_t) (TIME_RESOL_VAL * tempVar);

	return Status;
}

/***************************************************************************/
/* read battery cell age
 * batAge: pointer to a buffer to store received data
 */
Module_Status ReadCellAge(uint8_t *batAge) {
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batAge == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(AGE_REG_ADD, &tempVar, sizeof(tempVar)))
		Status = H05R0_OK;

	*batAge = (uint8_t) (tempVar / PERCENT_RESOL_VAL);

	return Status;
}

/***************************************************************************/
/* read battery cell charge discharge cycles
 * batCycles: pointer to a buffer to store received data
 */
Module_Status ReadCellCycles(uint16_t *batCycles) {
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batCycles == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(CYCLES_REG_ADD, &tempVar, sizeof(tempVar)))
		Status = H05R0_OK;

	*batCycles = (tempVar >> 3);
	//*batCycles = BAT_PERCENT_RESOL_VAL * tempVar;

	return Status;
}

/***************************************************************************/
/* read battery cell calculated internal resistance
 * batIntResistance: pointer to a buffer to store received data
 */
Module_Status ReadCellCalInterRes(float *batIntResistance) {
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batIntResistance == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(INTERNAL_RES_REG_ADD, &tempVar, sizeof(tempVar)))
		Status = H05R0_OK;

	*batIntResistance = (float) tempVar / RES_RESOL_VAL;

	return Status;
}

/***************************************************************************/
Module_Status ReadAllAnalogMeasurements(AnalogMeasType *batMeasurements) {
	Module_Status Status = H05R0_OK;
	uint8_t cntStatus = 0u;

	Status = ReadCellVoltage(&batMeasurements->BatVolt);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellCurrent(&batMeasurements->BatCurrent);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellPower(&batMeasurements->BatPower);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadTemperature(&batMeasurements->Temp);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellCapacity(&batMeasurements->BatCapacity);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellStateOfCharge(&batMeasurements->BatSOC);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellEstimatedTTE(&batMeasurements->BatTTE);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellEstimatedTTF(&batMeasurements->BatTTF);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellAge(&batMeasurements->BatAge);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellCycles(&batMeasurements->BatCycles);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellCalInterRes(&batMeasurements->BatIntResistance);
	if (H05R0_OK != Status)
		cntStatus++;
	Status = ReadSetChargVoltage(&batMeasurements->SetChargVolt);
	if (H05R0_OK != Status)
		cntStatus++;
	Status = ReadSetChargCurrent(&batMeasurements->SetChargCurrent);
	if (H05R0_OK != Status)
		cntStatus++;

	if (FALSE != cntStatus)
		Status = H05R0_ERROR;

	return Status;
}

/***************************************************************************/
/* read battery previously set charging voltage
 * setChargVolt: pointer to a buffer to store received data
 */
Module_Status ReadSetChargVoltage(float *setChargVolt) {
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (setChargVolt == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(CHARGE_VOLTAGE_REG_ADD, &tempVar, sizeof(tempVar)))
		Status = H05R0_OK;

	*setChargVolt = (float) (VOLT_RESOL_VAL * tempVar);

	return Status;
}

/***************************************************************************/
/* read battery previously set charging current
 * setChargCurrent: pointer to a buffer to store received data
 */
Module_Status ReadSetChargCurrent(float *setChargCurrent) {
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (setChargCurrent == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(CHARGE_CURRENT_REG_ADD, &tempVar, sizeof(tempVar)))
		Status = H05R0_OK;

	/* convert the received value from two's complementary to signed decimal value */
	*setChargCurrent = (float) (CUR_RESOL_VAL * tempVar);

	return Status;
}

/***************************************************************************/
/* read  charger Current measurement
 * ChargerCurrent: pointer to a buffer to store received data
 */
Module_Status ReadChargerCurrent(float *ChargerCurrent) {
	Module_Status Status = H05R0_ERROR;
	uint32_t tempVar = 0u;

	if (H05R0_OK == ReadADCValue(&hadc1, ADC_CHANNEL8, &tempVar, 100))
		Status = H05R0_OK;

	*ChargerCurrent = (((tempVar * 2.5) / (4096)) / (GAIN_CHARGER_VAL) / SENSE_CHARGER_VAL);

	return Status;
}

/***************************************************************************/
/* Read VBUS Voltage
 * Note: VBus refers to the battery or charger voltage.
 * When this is enabled, the battery voltage becomes available
 * through the green connector.
 * VBUSVolt: pointer to a buffer to store received data
 */
Module_Status ReadVBUSVoltage(float *VBUSVolt) {
	Module_Status Status = H05R0_ERROR;
	uint32_t tempVar = 0u;

	if (H05R0_OK == ReadADCValue(&hadc1, ADC_CHANNEL9, &tempVar, 100))
		Status = H05R0_OK;

	*VBUSVolt = (((tempVar * 2.5) / (4096)) * 3.35);

	return Status;
}

/***************************************************************************/
/* MCU Out Volt Enable To secure 3.3V for other Modules */
Module_Status Enable3_3Output(LDOOutputState PinState) {
	Module_Status Status = H05R0_OK;

	if (PinState == ENABLE_OUT)
		HAL_GPIO_WritePin(OUT_EN_3V3_GPIO_PORT, OUT_EN_3V3_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(OUT_EN_3V3_GPIO_PORT, OUT_EN_3V3_PIN, GPIO_PIN_RESET);

	return Status;
}

/***************************************************************************/
/* VBUS Output Switch Enable */
Module_Status EnableVBusOutput(LDOOutputState PinState) {
	Module_Status Status = H05R0_OK;

	if (PinState == ENABLE_OUT)
		HAL_GPIO_WritePin(VBUS_OUT_EN_GPIO_PORT, VBUS_OUT_EN_PIN, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(VBUS_OUT_EN_GPIO_PORT, VBUS_OUT_EN_PIN, GPIO_PIN_SET);

	return Status;
}

/***************************************************************************/
Module_Status CheckChargingStatus(void) {
	uint16_t tempVar = 0u;
	uint16_t sixthBit = 0u;
	Module_Status status = H05R0_OK;

	if (H05R0_OK != ReadReg(FPROT_STAT_REG_ADD, &tempVar, 2))
		return H05R0_COM_ERR;

	// Shift the value right by 5 bits to bring the sixth bit to the least significant position
	sixthBit = (tempVar >> 5) & 1;
	if (sixthBit == 0)
		AnalogMeasurement.ChargingStatus = CHARGING;
	else
		AnalogMeasurement.ChargingStatus = DISCHARGING;

	return H05R0_OK;
}
/***************************************************************************/
/* Streams a single sensor data sample to the terminal.
 * dstPort: Port number to stream data to.
 * dataFunction: Function to sample data (e.g., VOLTAGE, CURRENT, POWER, TEMPERATURE, CAPACITY, AGE, CYCLES).
 */
Module_Status SampleToTerminal(uint8_t dstPort, All_Data dataFunction) {
    Module_Status Status = H05R0_OK; /* Initialize operation status as success */
    int8_t *PcOutputString = NULL; /* Pointer to CLI output buffer */
    char CString[100] = {0}; /* Buffer for formatted output string */
    float value = 0.0f; /* Variable for float-based sensor data */
    uint8_t age = 0; /* Variable for age data */
    uint16_t cycles = 0; /* Variable for cycles data */

    /* Get the CLI output buffer for writing */
    PcOutputString = FreeRTOS_CLIGetOutputBuffer();

    /* Process data based on the requested sensor function */
    switch (dataFunction) {
        case BATTERY_VOLTAGE:
            /* Sample voltage data in volts */
            if (ReadCellVoltage(&value) != H05R0_OK) {
                return H05R0_ERROR; /* Return error if sampling fails */
            }


            /* Format voltage data into a string */
            snprintf(CString, 50, "Voltage(V) | %.2f\r\n", value);
            break;

        case BATTERY_CURRENT:
            /* Sample current data in amps */
            if (ReadCellCurrent(&value) != H05R0_OK) {
                return H05R0_ERROR; /* Return error if sampling fails */
            }
            /* Format current data into a string */
            snprintf(CString, 50, "Current(A) | %.2f\r\n", value);
            break;

        case BATTERY_POWER:
            /* Sample power data in watts */
            if (ReadCellPower(&value) != H05R0_OK) {
                return H05R0_ERROR; /* Return error if sampling fails */
            }
            /* Format power data into a string */
            snprintf(CString, 50, "Power(W) | %.2f\r\n", value);
            break;

        case BATTERY_TEMP:
            /* Sample temperature data in Celsius */
            if (ReadTemperature(&value) != H05R0_OK) {
                return H05R0_ERROR; /* Return error if sampling fails */
            }
            /* Format temperature data into a string */
            snprintf(CString, 50, "Temp(Celsius) | %.2f\r\n", value);
            break;

        case BATTERY_CAPACITY:
            /* Sample capacity data in mAh */
            if (ReadCellCapacity(&value) != H05R0_OK) {
                return H05R0_ERROR; /* Return error if sampling fails */
            }
            /* Format capacity data into a string */
            snprintf(CString, 50, "Capacity(mAh) | %.2f\r\n", value);
            break;

        case BATTERY_AGE:
            /* Sample age data in percentage */
            if (ReadCellAge(&age) != H05R0_OK) {
                return H05R0_ERROR; /* Return error if sampling fails */
            }
            /* Format age data into a string */
            snprintf(CString, 50, "Age(%%) | %u\r\n", age);
            break;

        case BATTERY_CYCLES:
            /* Sample cycles data as a number */
            if (ReadCellCycles(&cycles) != H05R0_OK) {
                return H05R0_ERROR; /* Return error if sampling fails */
            }
            /* Format cycles data into a string */
            snprintf(CString, 50, "Cycles | %u\r\n", cycles);
            break;

        default:
            /* Return error for invalid sensor function */
            return H05R0_ERR_WRONGPARAMS;
    }

    /* Send the formatted string to the specified port */
    writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);

    /* Return final status indicating success or prior error */
    return Status;
}

/***************************************************************************/
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples) {
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay = period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream
		for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf [pcPort - 1] [chr] == '\r' && Numofsamples > 0) {
				UARTRxBuf [pcPort - 1] [chr] = 0;
				StopeCliStreamFlag = 1;
				return H05R0_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H05R0_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H05R0_OK;
}

/***************************************************************************/
/*
 * brief: Samples data and exports it to a specified port.
 * param dstModule: The module number to export data from.
 * param dstPort: The port number to export data to.
 * param dataFunction: Function to sample data (e.g., VOLTAGE, CURRENT, POWER, TEMPERATURE, CAPACITY, AGE, CYCLES).
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction) {
    Module_Status Status = H05R0_OK;
    static uint8_t Temp[12] = {0}; /* Buffer for data transmission */
    float value = 0.0f;
    uint8_t age = 0;
    uint16_t cycles = 0;

    /* Check if the port and module ID are valid */
    if ((dstPort == 0) && (dstModule == myID)) {
        return H05R0_ERR_WRONGPARAMS;
    }

    /* Sample and export data based on function type */
    switch (dataFunction) {
        case BATTERY_VOLTAGE:
            if (ReadCellVoltage(&value) != H05R0_OK) {
                return H05R0_ERROR;
            }
            break;

        case BATTERY_CURRENT:
            if (ReadCellCurrent(&value) != H05R0_OK) {
                return H05R0_ERROR;
            }
            break;

        case BATTERY_POWER:
            if (ReadCellPower(&value) != H05R0_OK) {
                return H05R0_ERROR;
            }
            break;

        case BATTERY_TEMP:
            if (ReadTemperature(&value) != H05R0_OK) {
                return H05R0_ERROR;
            }
            break;

        case BATTERY_CAPACITY:
            if (ReadCellCapacity(&value) != H05R0_OK) {
                return H05R0_ERROR;
            }
            break;

        case BATTERY_AGE:
            if (ReadCellAge(&age) != H05R0_OK) {
                return H05R0_ERROR;
            }
            value = (float)age;
            break;

        case BATTERY_CYCLES:
            if (ReadCellCycles(&cycles) != H05R0_OK) {
                return H05R0_ERROR;
            }
            value = (float)cycles;
            break;

        default:
            return H05R0_ERR_WRONGPARAMS;
    }

    /* Transmit the sampled data */
    if (dstModule == myID || dstModule == 0) {
        /* LSB first */
        Temp[0] = (uint8_t)((*(uint32_t*)&value) >> 0);
        Temp[1] = (uint8_t)((*(uint32_t*)&value) >> 8);
        Temp[2] = (uint8_t)((*(uint32_t*)&value) >> 16);
        Temp[3] = (uint8_t)((*(uint32_t*)&value) >> 24);

        writePxITMutex(dstPort, (char*)&Temp[0], 4 * sizeof(uint8_t), 10);
    } else {
        /* LSB first */
        MessageParams[1] = (H05R0_OK == Status) ? BOS_OK : BOS_ERROR;
        MessageParams[0] = FMT_FLOAT;
        MessageParams[2] = 1;
        MessageParams[3] = (uint8_t)((*(uint32_t*)&value) >> 0);
        MessageParams[4] = (uint8_t)((*(uint32_t*)&value) >> 8);
        MessageParams[5] = (uint8_t)((*(uint32_t*)&value) >> 16);
        MessageParams[6] = (uint8_t)((*(uint32_t*)&value) >> 24);

        SendMessageToModule(dstModule, CODE_READ_RESPONSE, (sizeof(float) * 1) + 3);
    }

    /* Clear the temp buffer */
    memset(&Temp[0], 0, sizeof(Temp));

    return Status;
}

/***************************************************************************/
static Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout, SampleToString function) {
	Module_Status status = H05R0_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_PERIOD_MS)
		return H05R0_ERR_WRONGPARAMS;

	// TODO: Check if CLI is enable or not
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
		if (UARTRxBuf [pcPort - 1] [chr] == '\r') {
			UARTRxBuf [pcPort - 1] [chr] = 0;
		}
	}
	if (1 == StopeCliStreamFlag) {
		StopeCliStreamFlag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(pcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char*) pcOutputString, 100);

		writePxMutex(pcPort, (char*) pcOutputString, strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
			break;
	}

	memset((char*) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char*) pcOutputString, "\r\n");

	return status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified port and module with a given number of samples.
 * param targetModule: The target module to which data will be streamed.
 * param portNumber: The port number on the module.
 * param portFunction: Type of data that will be streamed (VOLTAGE, CURRENT, POWER, TEMPERATURE, CAPACITY, AGE, CYCLES).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamtoPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction, uint32_t numOfSamples, uint32_t streamTimeout) {
    Module_Status Status = H05R0_OK;
    uint32_t SamplePeriod = 0u;

    /* Check timer handle and timeout validity */
    if ((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples))
        return H05R0_ERROR; /* Assuming H05R0_ERROR is defined in Module_Status */

    /* Set streaming parameters */
    StreamMode = STREAM_MODE_TO_PORT;
    PortModule = dstModule;
    PortNumber = dstPort;
    PortFunction = dataFunction;
    PortNumOfSamples = numOfSamples;

    /* Calculate the period from timeout and number of samples */
    SamplePeriod = streamTimeout / numOfSamples;

    /* Stop (Reset) the TimerStream if it's already running */
    if (xTimerIsTimerActive(xTimerStream)) {
        if (pdFAIL == xTimerStop(xTimerStream, 100))
            return H05R0_ERROR;
    }

    /* Start the stream timer */
    if (pdFAIL == xTimerStart(xTimerStream, 100))
        return H05R0_ERROR;

    /* Update timer timeout - This also restarts the timer */
    if (pdFAIL == xTimerChangePeriod(xTimerStream, SamplePeriod, 100))
        return H05R0_ERROR;

    return Status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified terminal port with a given number of samples.
 * param targetPort: The port number on the terminal.
 * param dataFunction: Type of data that will be streamed (VOLTAGE, CURRENT, POWER, TEMPERATURE, CAPACITY, AGE, CYCLES).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToTerminal(uint8_t dstPort, All_Data dataFunction, uint32_t numOfSamples, uint32_t streamTimeout) {
    Module_Status Status = H05R0_OK;
    uint32_t SamplePeriod = 0u;

    /* Check timer handle and timeout validity */
    if ((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples))
        return H05R0_ERROR; /* Assuming H05R0_ERROR is defined in Module_Status */

    /* Set streaming parameters */
    StreamMode = STREAM_MODE_TO_TERMINAL;
    TerminalPort = dstPort;
    TerminalFunction = dataFunction;
    TerminalNumOfSamples = numOfSamples;

    /* Calculate the period from timeout and number of samples */
    SamplePeriod = streamTimeout / numOfSamples;

    /* Stop (Reset) the TimerStream if it's already running */
    if (xTimerIsTimerActive(xTimerStream)) {
        if (pdFAIL == xTimerStop(xTimerStream, 100))
            return H05R0_ERROR;
    }

    /* Start the stream timer */
    if (pdFAIL == xTimerStart(xTimerStream, 100))
        return H05R0_ERROR;

    /* Update timer timeout - This also restarts the timer */
    if (pdFAIL == xTimerChangePeriod(xTimerStream, SamplePeriod, 100))
        return H05R0_ERROR;

    return Status;
}
/***************************************************************************/
/*
 * @brief: Streams data to a buffer.
 * @param buffer: Pointer to the buffer where data will be stored.
 * @param function: Function to sample data (e.g., VOLTAGE, CURRENT, POWER, TEMPERATURE, CAPACITY, AGE, CYCLES).
 * @param Numofsamples: Number of samples to take.
 * @param timeout: Timeout period for the operation.
 * @retval: Module status indicating success or error.
 */
Module_Status StreamToBuffer(float *buffer, All_Data function, uint32_t Numofsamples, uint32_t timeout) {
    switch (function) {
        case BATTERY_VOLTAGE:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleVoltageBuf);
            break;
        case BATTERY_CURRENT:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleCurrentBuf);
            break;
        case BATTERY_POWER:
            return StreamToBuf(buffer, Numofsamples, timeout, SamplePowerBuf);
            break;
        case BATTERY_TEMP:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleTemperatureBuf);
            break;
        case BATTERY_CAPACITY:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleCapacityBuf);
            break;
        case BATTERY_AGE:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleAgeBuf);
            break;
        case BATTERY_CYCLES:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleCyclesBuf);
            break;
        default:
            return H05R0_ERR_WRONGPARAMS;
    }
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
/***************************************************************************/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
    const char *const VoltCmdName = "volt";
    const char *const CurrCmdName = "curr";
    const char *const PowCmdName = "pow";
    const char *const TempCmdName = "temp";
    const char *const CapCmdName = "cap";
    const char *const AgeCmdName = "age";
    const char *const CyclesCmdName = "cycles";

    const char *pSensName = NULL;
    portBASE_TYPE sensNameLen = 0;

    // Make sure we return something
    *pcWriteBuffer = '\0';

    pSensName = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 1, &sensNameLen);

    if (pSensName == NULL) {
        snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
        return pdFALSE;
    }

    do {
        if (!strncmp(pSensName, VoltCmdName, strlen(VoltCmdName))) {
            SampleToTerminal(pcPort, BATTERY_VOLTAGE);
        }
        else if (!strncmp(pSensName, CurrCmdName, strlen(CurrCmdName))) {
            SampleToTerminal(pcPort, BATTERY_CURRENT);
        }
        else if (!strncmp(pSensName, PowCmdName, strlen(PowCmdName))) {
            SampleToTerminal(pcPort, BATTERY_POWER);
        }
        else if (!strncmp(pSensName, TempCmdName, strlen(TempCmdName))) {
            SampleToTerminal(pcPort, BATTERY_TEMP);
        }
        else if (!strncmp(pSensName, CapCmdName, strlen(CapCmdName))) {
            SampleToTerminal(pcPort, BATTERY_CAPACITY);
        }
        else if (!strncmp(pSensName, AgeCmdName, strlen(AgeCmdName))) {
            SampleToTerminal(pcPort, BATTERY_AGE);
        }
        else if (!strncmp(pSensName, CyclesCmdName, strlen(CyclesCmdName))) {
            SampleToTerminal(pcPort, BATTERY_CYCLES);
        }
        else {
            snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
        }

        return pdFALSE;
    } while (0);

    snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
    return pdFALSE;
}

/***************************************************************************/
// Port Mode => false and CLI Mode => true
static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
                                bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule) {
    const char *pPeriodMSStr = NULL;
    const char *pTimeoutMSStr = NULL;

    portBASE_TYPE periodStrLen = 0;
    portBASE_TYPE timeoutStrLen = 0;

    const char *pPortStr = NULL;
    const char *pModStr = NULL;

    portBASE_TYPE portStrLen = 0;
    portBASE_TYPE modStrLen = 0;

    *ppSensName = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 1, pSensNameLen);
    pPeriodMSStr = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 2, &periodStrLen);
    pTimeoutMSStr = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 3, &timeoutStrLen);

    // At least 3 Parameters are required!
    if ((*ppSensName == NULL) || (pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
        return false;

    // TODO: Check if Period and Timeout are integers or not!
    *pPeriod = atoi(pPeriodMSStr);
    *pTimeout = atoi(pTimeoutMSStr);
    *pPortOrCLI = true;

    pPortStr = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 4, &portStrLen);
    pModStr = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 5, &modStrLen);

    if ((pModStr == NULL) && (pPortStr == NULL))
        return true;
    if ((pModStr == NULL) || (pPortStr == NULL)) // If user has provided 4 Arguments.
        return false;

    *pPort = atoi(pPortStr);
    *pModule = atoi(pModStr);
    *pPortOrCLI = false;

    return true;
}

/***************************************************************************/
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
    const char *const VoltCmdName = "volt";
    const char *const CurrCmdName = "curr";
    const char *const PowCmdName = "pow";
    const char *const TempCmdName = "temp";
    const char *const CapCmdName = "cap";
    const char *const AgeCmdName = "age";
    const char *const CyclesCmdName = "cycles";

    uint32_t Numofsamples = 0;
    uint32_t timeout = 0;
    uint8_t port = 0;
    uint8_t module = 0;

    bool portOrCLI = true; // Port Mode => false and CLI Mode => true

    const char *pSensName = NULL;
    portBASE_TYPE sensNameLen = 0;

    // Make sure we return something
    *pcWriteBuffer = '\0';

    if (!StreamCommandParser(pcCommandString, &pSensName, &sensNameLen, &portOrCLI, &Numofsamples, &timeout, &port, &module)) {
        snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
        return pdFALSE;
    }

    do {
        if (!strncmp(pSensName, VoltCmdName, strlen(VoltCmdName))) {
            if (portOrCLI) {
                StreamToCLI(Numofsamples, timeout, SampleVoltageToString);
            } else {
                StreamtoPort(module, port, BATTERY_VOLTAGE, Numofsamples, timeout);
            }
        }
        else if (!strncmp(pSensName, CurrCmdName, strlen(CurrCmdName))) {
            if (portOrCLI) {
                StreamToCLI(Numofsamples, timeout, SampleCurrentToString);
            } else {
                StreamtoPort(module, port, BATTERY_CURRENT, Numofsamples, timeout);
            }
        }
        else if (!strncmp(pSensName, PowCmdName, strlen(PowCmdName))) {
            if (portOrCLI) {
                StreamToCLI(Numofsamples, timeout, SamplePowerToString);
            } else {
                StreamtoPort(module, port, BATTERY_POWER, Numofsamples, timeout);
            }
        }
        else if (!strncmp(pSensName, TempCmdName, strlen(TempCmdName))) {
            if (portOrCLI) {
                StreamToCLI(Numofsamples, timeout, SampleTemperatureToString);
            } else {
                StreamtoPort(module, port, BATTERY_TEMP, Numofsamples, timeout);
            }
        }
        else if (!strncmp(pSensName, CapCmdName, strlen(CapCmdName))) {
            if (portOrCLI) {
                StreamToCLI(Numofsamples, timeout, SampleCapacityToString);
            } else {
                StreamtoPort(module, port, BATTERY_CAPACITY, Numofsamples, timeout);
            }
        }
        else if (!strncmp(pSensName, AgeCmdName, strlen(AgeCmdName))) {
            if (portOrCLI) {
                StreamToCLI(Numofsamples, timeout, SampleAgeToString);
            } else {
                StreamtoPort(module, port, BATTERY_AGE, Numofsamples, timeout);
            }
        }
        else if (!strncmp(pSensName, CyclesCmdName, strlen(CyclesCmdName))) {
            if (portOrCLI) {
                StreamToCLI(Numofsamples, timeout, SampleCyclesToString);
            } else {
                StreamtoPort(module, port, BATTERY_CYCLES, Numofsamples, timeout);
            }
        }
        else {
            snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
        }

        snprintf((char*)pcWriteBuffer, xWriteBufferLen, "\r\n");
        return pdFALSE;
    } while (0);

    snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
    return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
