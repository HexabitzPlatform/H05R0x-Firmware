/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H05R0.c
 Description   : Source code for module H05R0.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H05R0_inputs.h"
#include "H05R0_i2c.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;
/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;

/* Local functions */
Module_Status WriteReg(uint16_t regAddress, uint16_t Data);
Module_Status ReadReg(uint16_t regAddress, uint16_t *Buffer, uint8_t Size);
Module_Status ReadIdReg(uint16_t regAddress, uint16_t *Buffer, uint8_t NoBytes);
Module_Status ReadID(IdType *BatId);
Module_Status Init_MAX17330(void);
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples);

typedef void (*SampleMemsToPort)(uint8_t, uint8_t);
typedef void (*SampleMemsToString)(char *, size_t);
typedef void (*SampleMemsToBuffer)(float *buffer);

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};

/* Private variables ---------------------------------------------------------*/
TaskHandle_t LipoChargerTaskHandle = NULL;
uint8_t port1, module1;
uint8_t port2 ,module2,mode2,mode1;
uint32_t Numofsamples1 ,timeout1;
uint8_t port3 ,module3,mode3;
uint32_t Numofsamples3 ,timeout3;
uint8_t flag ;
uint8_t tofMode ;
static bool stopStream = false;
/* Private function prototypes -----------------------------------------------*/
Module_Status Exporttoport(uint8_t module,uint8_t port,All_Data function);
Module_Status Exportstreamtoport (uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout);
Module_Status Exportstreamtoterminal(uint32_t Numofsamples, uint32_t timeout,uint8_t Port,All_Data function);
void LipoChargerTask(void *argument);
void ExecuteMonitor(void);
void FLASH_Page_Eras(uint32_t Addr );
Module_Status ConvertTwosComplToDec(uint16_t twosComplVal, int16_t *sgnDecimalVal);
Module_Status BAT_ReadIdReg(uint16_t regAddress, uint16_t *Buffer, uint8_t NoBytes);
static Module_Status StreamMemsToCLI(uint32_t Numofsamples, uint32_t timeout, SampleMemsToString function);

/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellVoltageCommandstream( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellVoltageCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellCurrentCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellPowerCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadTemperatureCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellCapacityCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellStateOfChargeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellEstimatedTTECommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellEstimatedTTFCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellAgeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellCyclesCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellCalInterResCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadSetChargVoltageCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadSetChargCurrentCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadAllAnalogMeasurementsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellcurrentCommandstream( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellPowrCommandstream( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellsocCommandstream( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCellcapacityCommandstream( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_ReadCelltempCommandstream( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : demo */

/*-----------------------------------------------------------*/
/* CLI command structure : ReadCellVoltage */
const CLI_Command_Definition_t CLI_ReadCellVoltageCommandDefinition =
{
	( const int8_t * ) "cellvoltage", /* The command string to type. */
	( const int8_t * ) "cellvoltage:\r\n Parameters required to execute a readcellvoltage \r\n\r\n",
	CLI_ReadCellVoltageCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/* CLI command structure : ReadCellVoltage(stream) */
const CLI_Command_Definition_t CLI_ReadCellVoltageCommandDefinitionstream =
{
	( const int8_t * ) "streamvoltage", /* The command string to type. */
	( const int8_t * ) "streamvoltage:\r\n Parameters required to execute a streamvoltage \r\n\r\n",
	CLI_ReadCellVoltageCommandstream, /* The function to run. */
	2 /* tow parameters are expected. */
};
/* CLI command structure : ReadCellcurren(stream) */
/*-----------------------------------------------------------*/
const CLI_Command_Definition_t CLI_ReadCellCurrentCommandDefinitionstream =
{
	( const int8_t * ) "streamcurrent", /* The command string to type. */
	( const int8_t * ) "streamcurrent:\r\n Parameters required to execute a streamcurrent \r\n\r\n",
	CLI_ReadCellcurrentCommandstream, /* The function to run. */
	2 /* tow parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadCellCurrent */
const CLI_Command_Definition_t CLI_ReadCellCurrentCommandDefinition =
{
	( const int8_t * ) "cellcurrent", /* The command string to type. */
	( const int8_t * ) "cellcurrent:\r\n Parameters required to execute a readcellcurrent \r\n\r\n",
	CLI_ReadCellCurrentCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadCellPower */
const CLI_Command_Definition_t CLI_ReadCellPowerCommandDefinition =
{
	( const int8_t * ) "cellpower", /* The command string to type. */
	( const int8_t * ) "cellpower:\r\n Parameters required to execute a readcellpower \r\n\r\n",
	CLI_ReadCellPowerCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/* CLI command structure : ReadCellPower(stream) */
const CLI_Command_Definition_t CLI_ReadCellPowerCommandDefinitionstream =
{
	( const int8_t * ) "streampower", /* The command string to type. */
	( const int8_t * ) "streampower:\r\n Parameters required to execute a streampower \r\n\r\n",
	CLI_ReadCellPowrCommandstream, /* The function to run. */
	2 /* tow parameters are expected. */
};
/* CLI command structure : ReadCellTemp (stream) */
const CLI_Command_Definition_t CLI_ReadCellTempCommandDefinitionstream =
{
	( const int8_t * ) "streamtemp", /* The command string to type. */
	( const int8_t * ) "streamtemp:\r\n Parameters required to execute a streamtemp \r\n\r\n",
	CLI_ReadCelltempCommandstream, /* The function to run. */
	2 /* tow parameters are expected. */
};
/* CLI command structure : ReadCellbatCapacity (stream) */
const CLI_Command_Definition_t CLI_ReadCellCapacityCommandDefinitionstream =
{
	( const int8_t * ) "streamcapacity", /* The command string to type. */
	( const int8_t * ) "streamcapacity:\r\n Parameters required to execute a streamcapacity \r\n\r\n",
	CLI_ReadCellcapacityCommandstream, /* The function to run. */
	2 /* tow parameters are expected. */
};
/* CLI command structure : ReadSOC (stream) */
const CLI_Command_Definition_t CLI_ReadCellSOCCommandDefinitionstream =
{
	( const int8_t * ) "streamsoc", /* The command string to type. */
	( const int8_t * ) "streamsoc:\r\n Parameters required to execute a streamsoc \r\n\r\n",
	CLI_ReadCellsocCommandstream, /* The function to run. */
	2 /* tow parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadTemperature */
const CLI_Command_Definition_t CLI_ReadTemperatureCommandDefinition =
{
	( const int8_t * ) "temperature", /* The command string to type. */
	( const int8_t * ) "temperature:\r\n Parameters required to execute a readtemperature \r\n\r\n",
	CLI_ReadTemperatureCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadCellCapacity */
const CLI_Command_Definition_t CLI_ReadCellCapacityCommandDefinition =
{
	( const int8_t * ) "cellcapacity", /* The command string to type. */
	( const int8_t * ) "cellcapacity:\r\n Parameters required to execute a readcellcapacity \r\n\r\n",
	CLI_ReadCellCapacityCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadCellStateOfCharge */
const CLI_Command_Definition_t CLI_ReadCellStateOfChargeCommandDefinition =
{
	( const int8_t * ) "cellstateofcharge", /* The command string to type. */
	( const int8_t * ) "cellstateofcharge:\r\n Parameters required to execute a readcellstateofcharge \r\n\r\n",
	CLI_ReadCellStateOfChargeCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadCellEstimatedTTE */
const CLI_Command_Definition_t CLI_ReadCellEstimatedTTECommandDefinition =
{
	( const int8_t * ) "cellestimatedtte", /* The command string to type. */
	( const int8_t * ) "cellestimatedtte:\r\n Parameters required to execute a readcellestimatedtte \r\n\r\n",
	CLI_ReadCellEstimatedTTECommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadCellEstimatedTTF */
const CLI_Command_Definition_t CLI_ReadCellEstimatedTTFCommandDefinition =
{
	( const int8_t * ) "cellestimatedttf", /* The command string to type. */
	( const int8_t * ) "cellestimatedttf:\r\n Parameters required to execute a readcellestimatedttf \r\n\r\n",
	CLI_ReadCellEstimatedTTFCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadCellAge */
const CLI_Command_Definition_t CLI_ReadCellAgeCommandDefinition =
{
	( const int8_t * ) "cellage", /* The command string to type. */
	( const int8_t * ) "cellage:\r\n Parameters required to execute a readcellage \r\n\r\n",
	CLI_ReadCellAgeCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadCellCycles */
const CLI_Command_Definition_t CLI_ReadCellCyclesCommandDefinition =
{
	( const int8_t * ) "cellcycles", /* The command string to type. */
	( const int8_t * ) "cellcycles:\r\n Parameters required to execute a readcellcycles \r\n\r\n",
	CLI_ReadCellCyclesCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadCellCalInterRes */
const CLI_Command_Definition_t CLI_ReadCellCalInterResCommandDefinition =
{
	( const int8_t * ) "cellcalinterres", /* The command string to type. */
	( const int8_t * ) "cellcalinterres:\r\n Parameters required to execute a readcellcalinterres \r\n\r\n",
	CLI_ReadCellCalInterResCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadSetChargVoltage */
const CLI_Command_Definition_t CLI_ReadSetChargVoltageCommandDefinition =
{
	( const int8_t * ) "setchargvoltage", /* The command string to type. */
	( const int8_t * ) "setchargvoltage:\r\n Parameters required to execute a readsetchargvoltage \r\n\r\n",
	CLI_ReadSetChargVoltageCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadSetChargCurrent */
const CLI_Command_Definition_t CLI_ReadSetChargCurrentCommandDefinition =
{
	( const int8_t * ) "setchargcurrent", /* The command string to type. */
	( const int8_t * ) "setchargcurrent:\r\n Parameters required to execute a readsetchargcurrent \r\n\r\n",
	CLI_ReadSetChargCurrentCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ReadAllAnalogMeasurements */
const CLI_Command_Definition_t CLI_ReadAllAnalogMeasurementsCommandDefinition =
{
	( const int8_t * ) "allanalogmeasurements", /* The command string to type. */
	( const int8_t * ) "allanalogmeasurements:\r\n Parameters required to execute a readallanalogmeasurements \r\n\r\n",
	CLI_ReadAllAnalogMeasurementsCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};
/*-----------------------------------------------------------*/


/* ---------------------------------------------------------------------
 |							 Private Functions	                	   |
 ----------------------------------------------------------------------- 
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 12;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLKSOURCE_PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	__SYSCFG_CLK_ENABLE();

	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =8;
    uint16_t temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};
	
	HAL_FLASH_Unlock();
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	FLASH_PageErase(FLASH_BANK_1,RO_MID_ADDRESS);
	//TOBECHECKED
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}
	
	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS,temp);
		//TOBECHECKED
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}
		
		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){

          	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS + add,array[i - 1][j]);
				 //HALFWORD 	//TOBECHECKED
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						add +=8;
					}
				}
			}
		}
	}
	
	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
				//HALFWORD
				//TOBECHECKED
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
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[s].cmd + j*4 ));
				//HALFWORD
				//TOBECHECKED
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
	
	HAL_FLASH_Lock();
	
	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
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

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}

/* --- H05R0 module initialization.
 */
void Module_Peripheral_Init(void){

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();
	MX_I2C2_Init();
	MX_GPIO_Init();
	Init_MAX17330();

	 //Circulating DMA Channels ON All Module
	for (int i = 1; i <= NumOfPorts; i++) {
		if (GetUart(i) == &huart1) {
			index_dma[i - 1] = &(DMA1_Channel1->CNDTR);}
		else if (GetUart(i) == &huart2) {
			index_dma[i - 1] = &(DMA1_Channel2->CNDTR);}
		else if (GetUart(i) == &huart3) {
			index_dma[i - 1] = &(DMA1_Channel3->CNDTR);}
		else if (GetUart(i) == &huart4) {
			index_dma[i - 1] = &(DMA1_Channel4->CNDTR);}
		else if (GetUart(i) == &huart5) {
			index_dma[i - 1] = &(DMA1_Channel5->CNDTR);}
		else if (GetUart(i) == &huart6) {
			index_dma[i - 1] = &(DMA1_Channel6->CNDTR);}
	}


	/* Create module special task (if needed) */
	if(LipoChargerTaskHandle == NULL)
	xTaskCreate(LipoChargerTask,(const char* ) "LipoChargerTask",configMINIMAL_STACK_SIZE,NULL,osPriorityNormal - osPriorityIdle,&LipoChargerTaskHandle);

}

/*-----------------------------------------------------------*/
/* --- H05R0 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H05R0_OK;


	switch(code){
	case CODE_H05R0_CELLVOLTAGE:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batVolt);
		break;
	}

	case CODE_H05R0_CELLCURRENT:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batCurrent);
		break;
	}

	case CODE_H05R0_CELLPOWER:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batPower);
		break;
	}

	case CODE_H05R0_CELLTEMPERATURE:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],Temp);
		break;
	}

	case CODE_H05R0_CELLCAPACITY:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batCapacity);
		break;
	}

	case CODE_H05R0_CELLSTATEOFCHARGE:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batSOC);
		break;
	}

	case CODE_H05R0_CELLESTIMATEDTTE:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batTTE);
		break;
	}

	case CODE_H05R0_CELLESTIMATEDTTF:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batTTF);
		break;
	}

	case CODE_H05R0_CELLAGE:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batAge);
		break;
	}

	case CODE_H05R0_CELLCYCLES:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batCycles);
		break;
	}

	case CODE_H05R0_CELLCALINTERRES:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batIntResistance);
		break;
	}

	case CODE_H05R0_SETCHARGVOLTAGE:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],setChargVolt);
		break;
	}

		case CODE_H05R0_SETCHARGCURRENT:
	{
		SampletoPort(cMessage[port-1][shift],cMessage[port-1][1+shift],setChargCurrent);
		break;
	}

		default:
		result =H05R0_ERR_UnknownMessage;
		break;
	}
	
	return result;
}
/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART5)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART6)
		return P3;
	else if(huart->Instance == USART3)
		return P4;
	else if(huart->Instance == USART1)
		return P5;
	else if(huart->Instance == USART4)
		return P6;
	
	return 0;
}

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void) {

	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellSOCCommandDefinitionstream);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellCapacityCommandDefinitionstream);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellTempCommandDefinitionstream);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellVoltageCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellVoltageCommandDefinitionstream);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellCurrentCommandDefinitionstream);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellPowerCommandDefinitionstream);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellCurrentCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellPowerCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadTemperatureCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellCapacityCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellStateOfChargeCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellEstimatedTTECommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellEstimatedTTFCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellAgeCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellCyclesCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadCellCalInterResCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadSetChargVoltageCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadSetChargCurrentCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_ReadAllAnalogMeasurementsCommandDefinition);

}

/*-----------------------------------------------------------*/


/* Module special task function (if needed) */
void LipoChargerTask(void *argument){

	/* Infinite loop */
 // Test variable.
	for(;;){
		/*  */
		switch (tofMode) {

		case STREAM_TO_PORT:
			Exportstreamtoport(module1, port1, mode1, Numofsamples1, timeout1);
			break;
		case SAMPLE_TO_PORT:

			Exporttoport(module2, port2, mode2);
			break;
		case STREAM_TO_Terminal:
			Exportstreamtoterminal(Numofsamples3, timeout3, port3, mode3);

			break;

		default:
			osDelay(10);
			break;
		}

		taskYIELD();
	}

}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |								  Local Function                          |
/* -----------------------------------------------------------------------
/*
 * @brief: write 16-bit data to a Battery charger/gauge register
 * @param1: register's address to write data at
 * @param2: data to be written
 * @retval: status
 */
Module_Status WriteReg(uint16_t regAddress, uint16_t Data)
{
	Module_Status Status = H05R0_ERROR;
	uint8_t tempBuffer[4] = {0};
	uint8_t i2cSize = 0;
	uint8_t i2cSlaveAddress = 0;

	if (FST_I2C_LMT_ADD >= regAddress)
	{
		tempBuffer[0] = (uint8_t) regAddress;
		tempBuffer[1] = (uint8_t) Data;
		tempBuffer[2] = (uint8_t) (Data>>8);
		i2cSize = 3;
		i2cSlaveAddress = I2C_6Ch_W_ADD;
	}
	else
	{
		tempBuffer[0] = (uint8_t) regAddress;
		tempBuffer[1] = (uint8_t) Data;
		tempBuffer[2] = (uint8_t) (Data>>8);
		i2cSize = 3;
		i2cSlaveAddress = I2C_16h_W_ADD;
	}

	if (H05R0_OK == WriteI2C(I2C_PORT, i2cSlaveAddress, tempBuffer, i2cSize))
		Status = H05R0_OK;

	else
		Status = H05R0_ERROR;


	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read 16-bit data from a Battery charger/gauge register
 * @param1: register's address to read data from
 * @param2: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadReg(uint16_t regAddress, uint16_t *Buffer, uint8_t NoBytes)
{
	Module_Status Status = H05R0_ERROR;
	uint8_t tempBuffer[2] = {0};
	uint8_t i2cSize = 0;
	uint8_t i2cSlaveWriteAdd = 0;
	uint8_t i2cSlaveReadAdd = 0;

	if (Buffer == NULL)
		return Status;

	if (FST_I2C_LMT_ADD >= regAddress)
	{
		tempBuffer[0] = (uint8_t) regAddress;
		i2cSize = 1;
		i2cSlaveWriteAdd = I2C_6Ch_W_ADD;
		i2cSlaveReadAdd = I2C_6Ch_R_ADD;
	}
	else
	{
		tempBuffer[0] = (uint8_t) regAddress;
		i2cSize = 1;
		i2cSlaveWriteAdd = I2C_16h_W_ADD;
		i2cSlaveReadAdd = I2C_16h_R_ADD;
	}

		if (H05R0_OK == WriteI2C(I2C_PORT, i2cSlaveWriteAdd, tempBuffer, i2cSize))
			Status = H05R0_OK;

		else
			Status = H05R0_ERROR;

		if (H05R0_OK == ReadI2C(I2C_PORT, i2cSlaveReadAdd, (uint8_t*) Buffer, NoBytes))
			Status = H05R0_OK;

		else
			Status = H05R0_ERROR;

	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read 16-bit data from a Battery charger/gauge register
 * @param1: register's address to read data from
 * @param2: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadIdReg(uint16_t regAddress, uint16_t *Buffer, uint8_t NoBytes)
{
	Module_Status Status = H05R0_OK;
	uint8_t tempBuffer[2] = {0};
	uint8_t i2cSize = 0;
	uint8_t i2cSlaveWriteAdd = 0;
	uint8_t i2cSlaveReadAdd = 0;

	if (Buffer == NULL)
		return Status;

	tempBuffer[0] = (uint8_t) regAddress;
	i2cSize = 1;
	i2cSlaveWriteAdd = I2C_16h_W_ADD;
	i2cSlaveReadAdd = I2C_16h_R_ADD;


	if (H05R0_OK != WriteSMBUS(SMBUS_PORT, i2cSlaveWriteAdd, tempBuffer, i2cSize))
		Status = H05R0_ERROR;


	if (H05R0_OK != ReadSMBUS(SMBUS_PORT, i2cSlaveReadAdd, (uint8_t*) Buffer, NoBytes))
		Status = H05R0_ERROR;


	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: initialize Battery charger/gauge
 * @retval: status
 */
Module_Status Init_MAX17330(void)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;
	uint16_t tempBuffer[3] ={0};

	/* setting temperature sensing source from external thermistors and their type */
	tempVar = (EXT_THERM_10K << 11) | (EXT_THERM_EN << 12);
	if (H05R0_OK == WriteReg(PACK_CONFIG_REG_ADD, tempVar))
		Status = H05R0_OK;

	/* write sense resistor value to nRSense register */
	if (H05R0_OK == WriteReg(SENSE_RES_REG_ADD, SENSE_RES_REG_VAL))
		Status = H05R0_OK;

	/* write sense resistor B_Const to nThermCfg register */
	if (H05R0_OK == WriteReg(THERM_CONFIG_REG_ADD, THRMRES_CONFIG_REG_VAL))
		Status = H05R0_OK;

	/* enable battery ALRT, write configs to shadow memory */
	/* set the alert voltage thresholds */
	tempVar = MIN_VOLT_ALRT_THRE | (MAX_VOLT_ALRT_THRE << 8);
	if (H05R0_OK == WriteReg(VOLT_ALRT_THRE_REG_ADD, tempVar))
		Status = H05R0_OK;

	tempVar = MIN_CUR_ALRT_THRE | (MAX_CUR_ALRT_THRE << 8);
	if (H05R0_OK == WriteReg(CUR_ALRT_THRE_REG_ADD, tempVar))
		Status = H05R0_OK;

	tempVar = MIN_TEMP_ALRT_THRE | (MAX_TEMP_ALRT_THRE << 8);
	if (H05R0_OK == WriteReg(TEMP_ALRT_THRE_REG_ADD, tempVar))
		Status = H05R0_OK;

	tempVar = MIN_SOC_ALRT_THRE | (MAX_SOC_ALRT_THRE << 8);
	if (H05R0_OK == WriteReg(SOC_ALRT_THRE_REG_ADD, tempVar))
		Status = H05R0_OK;

	/* enable alerts by setting Aen bit */
	tempVar = ALRT_EN;
	if (H05R0_OK == WriteReg(CONFIG_REG_ADD, tempVar))
		Status = H05R0_OK;

	if (H05R0_OK == ReadReg(PACK_CONFIG_REG_ADD, &tempBuffer[0], sizeof(tempBuffer[0])))
			Status = H05R0_OK;

	if (H05R0_OK == ReadReg(SENSE_RES_REG_ADD, &tempBuffer[1], sizeof(tempBuffer[0])))
		Status = H05R0_OK;

	if (H05R0_OK == ReadReg(THERM_CONFIG_REG_ADD, &tempBuffer[2], sizeof(tempBuffer[0])))
		Status = H05R0_OK;

	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read Battery charger/gauge IDs
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadID(IdType *BatId)
{
	Module_Status Status = H05R0_OK;

	if (BatId == NULL)
		return H05R0_INV;

	/* de-initialize I2C and initialize SMBUS protocol. All chip's name and Id registers are reached
	 * through SMBUS  */
	HAL_I2C_DeInit(I2C_PORT);
	MX_I2C2_SMBUS_Init();

	if (H05R0_OK != ReadIdReg(MANFCTR_NAME_REG_ADD, BatId->ManId,MANFCTR_NAME_SIZE))
			Status = H05R0_ERROR;

	if (H05R0_OK != ReadIdReg(DEVICE_NAME_REG_ADD, BatId->DevId,DEVICE_NAME_SIZE))
		Status = H05R0_ERROR;


	/* de-initialize SMBUS and initialize I2C protocol to let the I2C port run in standard condition
	 * whenever it is called */
	HAL_SMBUS_DeInit(SMBUS_PORT);
	MX_I2C2_Init();

	return Status;
}

/* -----------------------------------------------------------------------
 |								  User Function
/* -----------------------------------------------------------------------
 */
/*
 * @brief: read battery cell voltage
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadCellVoltage(float *batVolt)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batVolt == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(CELL_VOLT_REG_ADD, &tempVar, sizeof(tempVar)))
			Status = H05R0_OK;

	*batVolt = (float) (VOLT_RESOL_VAL * tempVar);
	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read battery cell current
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadCellCurrent(float *batCurrent)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;
	int16_t tempSingedVar = 0;

	if (batCurrent == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(CURRENT_REG_ADD, &tempVar, sizeof(tempVar)))
			Status = H05R0_OK;

	/* convert the received value from two's complementary to signed decimal value */
	if (H05R0_OK == ConvertTwosComplToDec(tempVar, &tempSingedVar))
	tempSingedVar = tempVar;

	*batCurrent = (float) (CUR_RESOL_VAL * tempSingedVar);

	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read battery cell power
 * @param1: pointer to a buffer to store received data
 * @retval: status
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
/*-----------------------------------------------------------*/

/*
 * @brief: read battery cell temperature
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadTemperature(float *Temp)
{
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
/*-----------------------------------------------------------*/

/*
 * @brief: read battery cell capacity
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadCellCapacity(float *batCapacity)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batCapacity == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(FULL_CAP_REG_ADD, &tempVar, sizeof(tempVar)))
			Status = H05R0_OK;

	*batCapacity = (float) (CAP_RESOL_VAL * tempVar);
	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read battery cell state of charge
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadCellStateOfCharge(uint8_t *batSOC)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batSOC == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(REP_SOC_REG_ADD, &tempVar, sizeof(tempVar)))
			Status = H05R0_OK;

	*batSOC = (uint8_t) (tempVar / PERCENT_RESOL_VAL);
	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read battery time to empty
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadCellEstimatedTTE(uint32_t *batTTE)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batTTE == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(TTE_REG_ADD, &tempVar, sizeof(tempVar)))
			Status = H05R0_OK;

	*batTTE = (uint32_t) (TIME_RESOL_VAL * tempVar);
	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read battery cell time to full
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadCellEstimatedTTF(uint32_t *batTTF)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batTTF == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(TTF_REG_ADD, &tempVar, sizeof(tempVar)))
			Status = H05R0_OK;

	*batTTF = (uint32_t)  (TIME_RESOL_VAL * tempVar);
	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read battery cell age
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadCellAge(uint8_t *batAge)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batAge == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(AGE_REG_ADD, &tempVar, sizeof(tempVar)))
			Status = H05R0_OK;

	*batAge = (uint8_t) (tempVar / PERCENT_RESOL_VAL);
	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read battery cell charge discharge cycles
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadCellCycles(uint16_t *batCycles)
{
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
/*-----------------------------------------------------------*/

/*
 * @brief: read battery cell calculated internal resistance
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadCellCalInterRes(float *batIntResistance)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (batIntResistance == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(INTERNAL_RES_REG_ADD, &tempVar, sizeof(tempVar)))
			Status = H05R0_OK;

	*batIntResistance = (float) tempVar / RES_RESOL_VAL;
	return Status;
}
/*-----------------------------------------------------------*/

/*
 *
 */
Module_Status ReadAllAnalogMeasurements(AnalogMeasType *batMeasurements)
{
	Module_Status Status = H05R0_OK;
	uint8_t cntStatus = 0u;

	Status = ReadCellVoltage(&batMeasurements->batVolt);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellCurrent(&batMeasurements->batCurrent);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellPower(&batMeasurements->batPower);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadTemperature(&batMeasurements->Temp);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellCapacity(&batMeasurements->batCapacity);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellStateOfCharge(&batMeasurements->batSOC);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellEstimatedTTE(&batMeasurements->batTTE);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellEstimatedTTF(&batMeasurements->batTTF);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellAge(&batMeasurements->batAge);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellCycles(&batMeasurements->batCycles);
	if (H05R0_OK != Status)
		cntStatus++;

	Status = ReadCellCalInterRes(&batMeasurements->batIntResistance);
	if (H05R0_OK != Status)
		cntStatus++;
	Status = ReadSetChargVoltage(&batMeasurements->setChargVolt);
	if (H05R0_OK != Status)
		cntStatus++;
	Status = ReadSetChargCurrent(&batMeasurements->setChargCurrent);
	if (H05R0_OK != Status)
		cntStatus++;

	if (FALSE != cntStatus)
		Status = H05R0_ERROR;

	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read battery cell calculated internal resistance
 * @param1: the raw value to be processed
 * @param2: pointer to a buffer to store converted signed value
 * @retval: status
 */
Module_Status ConvertTwosComplToDec(uint16_t twosComplVal, int16_t *sgnDecimalVal)
{

	if (sgnDecimalVal == NULL && ((UNSNGD_HALF_WORD_MAX_VAL < twosComplVal) || (UNSNGD_HALF_WORD_MIN_VAL > twosComplVal)))
	return H05R0_INV;

	if (TWO_COMPL_VAL_MASK < twosComplVal)
		*sgnDecimalVal = -((~twosComplVal) + 1);

	else
		*sgnDecimalVal = twosComplVal;

	return H05R0_OK;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read battery previously set charging voltage
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadSetChargVoltage(float *setChargVolt)
{
	Module_Status Status = H05R0_ERROR;
	uint16_t tempVar = 0u;

	if (setChargVolt == NULL)
		return H05R0_INV;

	if (H05R0_OK == ReadReg(CHARGE_VOLTAGE_REG_ADD, &tempVar, sizeof(tempVar)))
			Status = H05R0_OK;

	*setChargVolt = (float) (VOLT_RESOL_VAL * tempVar);
	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read battery previously set charging current
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadSetChargCurrent(float *setChargCurrent)
{
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
/*-----------------------------------------------------------*/

/*
 * @brief: write configurations to non-volatile memory registers
 * @param1: pointer to a buffer storing the configurations to be written
 * @retval: status
 */
Module_Status WriteConfigsToNV(uint16_t *pConfigBuffer)
{
	Module_Status Status = H05R0_OK;
	uint16_t tempVar = 0u;

	/* write the desired configs to their addresses at the shadow RAM */

	/* clear CommStat.NVError bit */
	tempVar = CMD_STAT_ERR_CLEAR;
	if (H05R0_OK != ReadReg(CMD_STAT_REG_ADD, &tempVar, sizeof(tempVar)))
		return H05R0_COM_ERR;

	/* write 0xE904 to the Command register 0x060 to initiate a block copy */
	if (H05R0_OK != WriteReg(CMD_REG_ADD, 0xE904))
		return H05R0_COM_ERR;

	/* wait t_BLOCK for the copy to complete */
	Delay_ms(BLOCK_TIME);

	/* check the CommStat.NVError bit, if set repeat the process */

	/* write 0x000F to the Command register 0x060 to POR the IC */
	if (H05R0_OK != WriteReg(CMD_REG_ADD, 0x000F))
		return H05R0_COM_ERR;

	/* wait 10msec */
	Delay_ms(10);

	/* write 0x8000 to Config2 register 0x0AB to reset firmware */
	if (H05R0_OK != WriteReg(0x0AB, 0x8000))
		return H05R0_COM_ERR;

	/* Wait for POR_CMD bit (bit 15) of the Config2 register to be cleared to indicate
	 *  POR sequence is complete. */

	return Status;
}
/*-----------------------------------------------------------*/

/*
 * @brief: read number of remaining non-volatile memory writes
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadNumOfRemainingWrites(uint8_t *remWrites)
{
	Module_Status Status = H05R0_OK;
	uint16_t tempVar = 0u;
	uint8_t retValueLSB = 0u;
	uint8_t retValueMSB = 0u;
	uint8_t numOnes = 0u;
	bool bitState = FALSE;

	/* the following steps are implemented as stated in the data sheet */
	/* write 0xE29B to the Command register 0x060 to initiate a block copy */
	if (H05R0_OK != WriteReg(CMD_REG_ADD, 0xE29B))
		return H05R0_COM_ERR;

	/* wait t_BLOCK for the copy to complete */
	Delay_ms(RECALL_TIME);

	/* read memory address 1FDh */
	if (H05R0_OK != ReadReg(REM_UPDT_REG_ADD, &tempVar, 1))
		return H05R0_COM_ERR;

	/* decode the received data */
	retValueLSB = (uint8_t) tempVar;
	retValueMSB = (uint8_t) (tempVar >> 8);
	tempVar = (retValueLSB | retValueMSB);

	/* calculate the number of ones of tempVar value */
	for (volatile uint8_t bitIndex = 0u; bitIndex<8; bitIndex++)
	{
		bitState = (bool) ((tempVar >> bitIndex) & TRUE);
		if (TRUE == bitState)
			numOnes++;
	}

	/* determine the number of left updates */
	*remWrites = 8 - numOnes;
//	if (BAT_NUM_1S_1 == tempVar)
//		*remWrites = 7;
//
//	else if (BAT_NUM_1S_2 == tempVar)
//		*remWrites = 6;
//
//	else if (BAT_NUM_1S_3 == tempVar)
//		*remWrites = 5;
//
//	else if (BAT_NUM_1S_4 == tempVar)
//		*remWrites = 4;
//
//	else if (BAT_NUM_1S_5 == tempVar)
//		*remWrites = 3;
//
//	else if (BAT_NUM_1S_6 == tempVar)
//		*remWrites = 2;
//
//	else if (BAT_NUM_1S_7 == tempVar)
//		*remWrites = 1;
//
//	else if (BAT_NUM_1S_8 == tempVar)
//		*remWrites = 0;
//
//	else
//		Status = H05R0_ERROR;

	return Status;
}
/*-----------------------------------------------------------*/

Module_Status StreamToTerminal(uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout)
{
	Module_Status status = H05R0_OK;
	tofMode=STREAM_TO_Terminal;
	port3 = port ;
	Numofsamples3=Numofsamples;
	timeout3=timeout;
	mode3= function;
	return status;
}
Module_Status Exportstreamtoterminal(uint32_t Numofsamples, uint32_t timeout,uint8_t Port,All_Data function)
 {
	Module_Status status = H05R0_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	float floatData;
	uint8_t uint8Data = 0;
	uint16_t uint16Data = 0;
	uint32_t uint32Data = 0;
	static uint8_t temp[4] = { 0 };
	char cstring[100];
	long numTimes = timeout / period;
	if (period < MIN_MEMS_PERIOD_MS)
		return H05R0_ERR_WrongParams;

	// TODO: Check if CLI is enable or not
	switch (function) {
	case batVolt:
		if (period > timeout)
			timeout = period;

		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadCellVoltage(&floatData);

			snprintf(cstring, 50, "CellVoltage | Voltage: %.2f\r\n", floatData);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
		break;

	case batCurrent:
		if (period > timeout)
			timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadCellCurrent(&floatData);

			snprintf(cstring, 50, "CellCurrent | Current: %.2f\r\n", floatData);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
		break;

		break;

	case batPower:

		if (period > timeout)
			timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadCellPower(&floatData);

			snprintf(cstring, 50, "CellPower | Power: %.2f\r\n", floatData);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
		break;
		break;

	case Temp:

		if (period > timeout)
			timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadTemperature(&floatData);

			snprintf(cstring, 50, "Temperature | Temperature: %.2f\r\n",
					floatData);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
		break;

	case batCapacity:

		if (period > timeout)
			timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadCellCapacity(&floatData);

			snprintf(cstring, 50, "CellCapacity | Capacity: %.2f\r\n",
					floatData);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
		break;

	case batSOC:

		timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadCellStateOfCharge(&uint8Data);

			snprintf(cstring, 50, "CellStateOfCharge | Charge: %.2f\r\n",
					uint8Data);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}

		break;

	case batTTE:

		timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadCellEstimatedTTE(&uint32Data);

			snprintf(cstring, 50, "CellEstimatedTTE : %.2f\r\n", uint32Data);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
	case batTTF:

		timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadCellEstimatedTTF(&uint32Data);

			snprintf(cstring, 50, "CellEstimatedTTF : %.2f\r\n", uint32Data);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
	case batAge:

		timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadCellAge(&uint8Data);

			snprintf(cstring, 50, "CellAge : %.2f\r\n", uint8Data);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
	case batCycles:

		timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadCellCycles(&uint16Data);

			snprintf(cstring, 50, "CellCycles | Cycles: %.2f\r\n", uint16Data);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
	case batIntResistance:

		break;

	case setChargVolt:

		timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadSetChargVoltage(&floatData);

			snprintf(cstring, 50, "SetChargVoltage | Voltage: %.2f\r\n",
					floatData);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
	case setChargCurrent:

		timeout = period;
		stopStream = false;

		while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			pcOutputString = FreeRTOS_CLIGetOutputBuffer();
			ReadSetChargCurrent(&floatData);

			snprintf(cstring, 50, "SetChargCurrent | Current: %.2f\r\n",
					floatData);

			writePxMutex(Port, (char*) cstring, strlen((char*) cstring),
					cmd500ms, HAL_MAX_DELAY);
			if (PollingSleepCLISafe(period, Numofsamples) != H05R0_OK)
				break;
		}
	default:
		status = H05R0_ERR_WrongParams;
		break;
	}

	tofMode = DEFAULT;
	return status;
}


static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples)
{
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay =  period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream
		for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[PcPort - 1][chr] == '\r' && Numofsamples > 0) {
				UARTRxBuf[PcPort - 1][chr] = 0;
				flag=1;
				return H05R0_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H05R0_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H05R0_OK;
}

/*
 * @brief: read number of remaining non-volatile memory writes
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status LockNonVolatileMemory(void)
{
	bool csErrorBit = 0;
	uint16_t tempVar = 0u;

	/* read command status register */
	if (H05R0_OK != ReadReg(CMD_STAT_REG_ADD, &tempVar, 1))
		return H05R0_COM_ERR;

	//csErrorBit = tempVar & 32;

	/* repeat the process till CommStat.NVError bit is zero */
	while (TRUE == csErrorBit)
	{
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

/*-----------------------------------------------------------*/
/*
 *  Sending a sample of the required module on the required port
 *  If the topology file is not activated, therefore The module number is 0
 *  Transmission using LSP FIRST.
 *  function It is one of the elements of All_Data
 */
Module_Status Exporttoport(uint8_t module,uint8_t port,All_Data function)
 {
	float floatData = 0;
	uint8_t uint8Data = 0;
	uint16_t uint16Data = 0;
	uint32_t uint32Data = 0;
	static uint8_t temp[4] = { 0 };
	Module_Status status = H05R0_OK;

	if (port == 0 && module == myID) {
		return H05R0_ERR_WrongParams;
	}
	switch (function) {
	case batVolt:
		status = ReadCellVoltage(&floatData);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			temp[2] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			temp[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
				messageParams[1] = BOS_OK;
			else
				messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_FLOAT;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(float) + 3);
		}
		break;

	case batCurrent:
		status = ReadCellCurrent(&floatData);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			temp[2] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			temp[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
				messageParams[1] = BOS_OK;
			else
				messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_FLOAT;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(float) + 3);
		}
		break;

	case batPower:
		status = ReadCellPower(&floatData);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			temp[2] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			temp[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
				messageParams[1] = BOS_OK;
			else
				messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_FLOAT;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(float) + 3);
		}
		break;

	case Temp:
		status = ReadTemperature(&floatData);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			temp[2] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			temp[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
				messageParams[1] = BOS_OK;
			else
				messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_FLOAT;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(float) + 3);
		}
		break;

	case batCapacity:
		status = ReadCellCapacity(&floatData);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			temp[2] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			temp[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
				messageParams[1] = BOS_OK;
			else
				messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_FLOAT;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(float) + 3);
		}
		break;

	case batSOC:
		status = ReadCellStateOfCharge(&uint8Data);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) uint8Data;
			writePxITMutex(port, (char*) &temp[0], 1 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
						messageParams[1] = BOS_OK;
					else
						messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_UINT8;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) uint8Data;
			SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(uint8_t) + 3);
		}
		break;

	case batTTE:
		status = ReadCellEstimatedTTE(&uint32Data);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 8);
			temp[2] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 16);
			temp[3] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 24);
			writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
						messageParams[1] = BOS_OK;
					else
						messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_UINT32;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE,
					sizeof(uint32Data) + 3);
		}
		break;

	case batTTF:
		status = ReadCellEstimatedTTF(&uint32Data);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 8);
			temp[2] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 16);
			temp[3] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 24);
			writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
						messageParams[1] = BOS_OK;
					else
						messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_UINT32;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &uint32Data) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE,
					sizeof(uint32Data) + 3);
		}
		break;

	case batAge:
		status = ReadCellAge(&uint8Data);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) uint8Data;
			writePxITMutex(port, (char*) &temp[0], sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
						messageParams[1] = BOS_OK;
					else
						messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_UINT8;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) uint8Data;
			SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(uint8_t) + 3);
		}
		break;

	case batCycles:
		status = ReadCellCycles(&uint16Data);
		if (module == myID) {
			temp[0] = (uint8_t) ((*(uint16_t*) &uint16Data) >> 0);
			temp[1] = (uint8_t) ((*(uint16_t*) &uint16Data) >> 8);
			writePxITMutex(port, (char*) &temp[0], 2 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
						messageParams[1] = BOS_OK;
					else
						messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_UINT16;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint16_t*) &uint16Data) >> 0);
			messageParams[4] = (uint8_t) ((*(uint16_t*) &uint16Data) >> 8);
			SendMessageToModule(module, CODE_READ_RESPONSE,
					sizeof(uint16_t) + 3);
		}
		break;

	case batIntResistance:
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			temp[2] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			temp[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
				messageParams[1] = BOS_OK;
			else
				messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_FLOAT;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(float) + 3);
		}
		break;

	case setChargVolt:
		status = ReadSetChargVoltage(&floatData);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			temp[2] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			temp[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
				messageParams[1] = BOS_OK;
			else
				messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_FLOAT;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(float) + 3);
		}
		break;

	case setChargCurrent:
		status = ReadSetChargCurrent(&floatData);
		if (module == myID || module == 0) {
			temp[0] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			temp[2] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			temp[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			writePxITMutex(port, (char*) &temp[0], 4 * sizeof(uint8_t), 10);
		} else {
			if (H05R0_OK == status)
				messageParams[1] = BOS_OK;
			else
				messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_FLOAT;
			messageParams[2] = 1;
			messageParams[3] = (uint8_t) ((*(uint32_t*) &floatData) >> 0);
			messageParams[4] = (uint8_t) ((*(uint32_t*) &floatData) >> 8);
			messageParams[5] = (uint8_t) ((*(uint32_t*) &floatData) >> 16);
			messageParams[6] = (uint8_t) ((*(uint32_t*) &floatData) >> 24);
			SendMessageToModule(module, CODE_READ_RESPONSE, sizeof(float) + 3);
		}
		break;

	default:
		status = H05R0_ERR_WrongParams;
		break;
	}
	tofMode = DEFAULT;
	memset(&temp[0], 0, sizeof(temp));
	return status;
}

Module_Status SampletoPort(uint8_t module,uint8_t port,All_Data function)
 {
	Module_Status status = H05R0_OK;
	tofMode = SAMPLE_TO_PORT;
	port2 = port;
	module2 = module;
	mode2 = function;
	return status;
}


/*
 * Sending a Stream of the required module on the required port
 * 	period=timeout/Numofsamples -->minimum=100
 * 	The minimum time between one sample and another is its value 100ms
 *
 */
Module_Status StreamtoPort(uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout)
{
	Module_Status status = H05R0_OK;
	tofMode=STREAM_TO_PORT;
	port1 = port ;
	module1 =module;
	Numofsamples1=Numofsamples;
	timeout1=timeout;
	mode1= function;
	return status;

}
Module_Status Exportstreamtoport (uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout)
 {
	Module_Status status = H05R0_OK;
	uint32_t samples = 0;
	uint32_t period = 0;
	period = timeout / Numofsamples;

	if (timeout < MIN_PERIOD_MS || period < MIN_PERIOD_MS)
		return H05R0_ERR_WrongParams;

	while (samples < Numofsamples) {
		status = Exporttoport(module, port, function);
		vTaskDelay(pdMS_TO_TICKS(period));
		samples++;
	}
	tofMode = DEFAULT;
	samples = 0;
	return status;
}

static Module_Status StreamMemsToCLI(uint32_t Numofsamples, uint32_t timeout, SampleMemsToString function)
{
	Module_Status status = H05R0_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H05R0_ERR_WrongParams;

	// TODO: Check if CLI is enable or not
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[PcPort - 1][chr] == '\r' ) {
				UARTRxBuf[PcPort - 1][chr] = 0;
			}
		}
	if (1 == flag) {
		flag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(PcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}
	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char *)pcOutputString, 100);


		writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period,Numofsamples) != H05R0_OK)
			break;
	}

	memset((char *) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcOutputString, "\r\n");
	return status;
}

void SampleVoltageToString(char *cstring, size_t maxLen)
{
	float Voltage = 0;
	ReadCellVoltage(&Voltage);
	snprintf(cstring, maxLen, "Voltage: %.2f\r\n", Voltage);
}

void SamplecurrentToString(char *cstring, size_t maxLen)
{
	float current = 0;
	ReadCellCurrent(&current);
	snprintf(cstring, maxLen, "current: %.2f\r\n", current);
}
void SamplepowrToString(char *cstring, size_t maxLen)
{
	float powr = 0;
	ReadCellPower(&powr);
	snprintf(cstring, maxLen, "powr: %.2f\r\n", powr);
}
void SampleTempToString(char *cstring, size_t maxLen)
{
	float Temp = 0;
	ReadTemperature(&Temp);
	snprintf(cstring, maxLen, "Temp: %.2f\r\n", Temp);
}
void SamplebatCapacityToString(char *cstring, size_t maxLen)
{
	float batCapacity = 0;
	ReadCellCapacity(&batCapacity);
	snprintf(cstring, maxLen, "batCapacity: %.2f\r\n", batCapacity);
}
void SamplebatSOCToString(char *cstring, size_t maxLen)
{
	uint8_t batSOC = 0;
	ReadCellStateOfCharge(&batSOC);
	snprintf(cstring, maxLen, "batSOC: %.2f\r\n", batSOC);
}

/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */
portBASE_TYPE CLI_ReadCellVoltageCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	float batVolt=0;
	static const int8_t *pcOKMessage=(int8_t* )"CellVoltage is:%0.3fV\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadCellVoltage(&batVolt);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batVolt);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellVoltageCommandstream(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H05R0_OK;

	uint32_t Numofsamples, pTimeout;
	static int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;

	(void) xWriteBufferLen;
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);

	Numofsamples = atoi(pcParameterString1);
	pTimeout = atoi(pcParameterString2);

	StreamMemsToCLI(Numofsamples, pTimeout, SampleVoltageToString);
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellPowrCommandstream(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H05R0_OK;

	uint32_t Numofsamples, pTimeout;
	static int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;

	(void) xWriteBufferLen;
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);

	Numofsamples = atoi(pcParameterString1);
	pTimeout = atoi(pcParameterString2);

	StreamMemsToCLI(Numofsamples, pTimeout, SamplepowrToString);
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellsocCommandstream(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H05R0_OK;

	uint32_t Numofsamples, pTimeout;
	static int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;

	(void) xWriteBufferLen;
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);

	Numofsamples = atoi(pcParameterString1);
	pTimeout = atoi(pcParameterString2);

	StreamMemsToCLI(Numofsamples, pTimeout, SamplebatSOCToString);
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellcapacityCommandstream(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H05R0_OK;

	uint32_t Numofsamples, pTimeout;
	static int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;

	(void) xWriteBufferLen;
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);

	Numofsamples = atoi(pcParameterString1);
	pTimeout = atoi(pcParameterString2);

	StreamMemsToCLI(Numofsamples, pTimeout, SamplebatCapacityToString);
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCelltempCommandstream(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H05R0_OK;

	uint32_t Numofsamples, pTimeout;
	static int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;

	(void) xWriteBufferLen;
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);

	Numofsamples = atoi(pcParameterString1);
	pTimeout = atoi(pcParameterString2);

	StreamMemsToCLI(Numofsamples, pTimeout, SampleTempToString);
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellcurrentCommandstream(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H05R0_OK;

	uint32_t Numofsamples, pTimeout;
	static int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;

	(void) xWriteBufferLen;
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);

	Numofsamples = atoi(pcParameterString1);
	pTimeout = atoi(pcParameterString2);

	StreamMemsToCLI(Numofsamples, pTimeout, SamplecurrentToString);
}
/*-----------------------------------------------------------*/

portBASE_TYPE CLI_ReadCellCurrentCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	float batCurrent=0;
	static const int8_t *pcOKMessage=(int8_t* )"CellCurrent is:%0.3fA\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadCellCurrent(&batCurrent);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batCurrent);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellPowerCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	float batPower=0;
	static const int8_t *pcOKMessage=(int8_t* )"CellPower is:%0.3fW \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadCellPower(&batPower);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batPower);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadTemperatureCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	float Temp=0;
	static const int8_t *pcOKMessage=(int8_t* )"Temperature is:%0.3fC\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadTemperature(&Temp);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,Temp);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellCapacityCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	float batCapacity=0;
	static const int8_t *pcOKMessage=(int8_t* )"CellCapacity is:%0.3fA\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadCellCapacity(&batCapacity);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batCapacity);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellStateOfChargeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	uint8_t batSOC=0;
	static const int8_t *pcOKMessage=(int8_t* )"CellStateOfCharge is:%d%% \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadCellStateOfCharge(&batSOC);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batSOC);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellEstimatedTTECommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	uint32_t batTTE=0;
	static const int8_t *pcOKMessage=(int8_t* )"CellEstimatedTTE is:%dms \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadCellEstimatedTTE(&batTTE);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batTTE);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellEstimatedTTFCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	uint32_t batTTF=0;
	static const int8_t *pcOKMessage=(int8_t* )"CellEstimatedTTF is:%dms \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadCellEstimatedTTF(&batTTF);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batTTF);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellAgeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	uint8_t batAge=0;
	static const int8_t *pcOKMessage=(int8_t* )"CellAge is:%d%% \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadCellAge(&batAge);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batAge);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellCyclesCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	uint16_t batCycles=0;
	static const uint16_t *pcOKMessage=(uint16_t* )"CellCycles is:%d \n\r";
	static const uint16_t *pcErrorsMessage =(uint16_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadCellCycles(&batCycles);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batCycles);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadCellCalInterResCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	float batIntResistance=0;
	static const int8_t *pcOKMessage=(int8_t* )"CellCalInterRes is:%0.3fohm \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadCellCalInterRes(&batIntResistance);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batIntResistance);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadSetChargVoltageCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	float setChargVolt=0;
	static const int8_t *pcOKMessage=(int8_t* )"SetChargVoltage is:%0.3fV\n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadSetChargVoltage(&setChargVolt);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,setChargVolt);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadSetChargCurrentCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	float setChargCurrent=0;
	static const int8_t *pcOKMessage=(int8_t* )"SetChargCurrent is:%0.3fA \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadSetChargCurrent(&setChargCurrent);

	 if(status == H05R0_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,setChargCurrent);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_ReadAllAnalogMeasurementsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H05R0_OK;
	AnalogMeasType batMeasurements;

	static const int8_t *pcOKMessage =(int8_t* )"AllAnalogMeasurements: \r\n"
		"CellVoltage is:%0.3fV\n\r"
		"CellCurrent is:%0.3fA\n\r"
		"CellPower is:%0.3fW \n\r"
		"Temperature is:%0.3fC\n\r"
		"CellCapacity is:%0.3fA\n\r"
		"CellStateOfCharge is:%d%% \n\r"
		"CellEstimatedTTE is:%dms \n\r"
		"CellEstimatedTTF is:%dms \n\r"
		"CellAge is:%d%% \n\r"
		"CellCycles is:%d \n\r"
		"CellCalInterRes is:%0.3fohm \n\r"
		"SetChargVoltage is:%0.3fV\n\r"
		"SetChargCurrent is:%0.3fA \n\r";

	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=ReadAllAnalogMeasurements(&batMeasurements);

	 if(status == H05R0_OK)
	 {
	 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,batMeasurements.batVolt,batMeasurements.batCurrent,batMeasurements.batPower
	 ,batMeasurements.Temp,batMeasurements.batCapacity,batMeasurements.batSOC,batMeasurements.batTTE,batMeasurements.batTTF
	 ,batMeasurements.batAge,batMeasurements.batCycles,batMeasurements.batIntResistance,batMeasurements.setChargVolt
	 ,batMeasurements.setChargCurrent);

	 }

	 else if(status == H05R0_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
