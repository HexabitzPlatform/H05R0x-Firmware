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
#include "H05R0_adc.h"


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

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] =
{{.ParamPtr = NULL, .ParamFormat =FMT_FLOAT, .ParamName =""}};

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
AnalogMeasType AnalogMeasurement;
float ChargingCurrent,ChargingVolt=0;
uint8_t StateOfCharger=0,FullCharge=0;
uint8_t ChargStatus = 1u;


TimerHandle_t xTimerStream = NULL;

/* Stream to port variables */
volatile uint32_t PortNumOfSamples = 0u;    /* Number of samples for port streaming */
volatile uint32_t PortSamples = 0u;         /* Current sample count for port (if needed separately) */
uint8_t PortModule = 0u;           /* Module ID for port streaming */
uint8_t PortNumber = 0u;           /* Port number for streaming */
All_Data PortFunction;                    /* Function pointer or struct for port streaming */

/* Stream to terminal variables */
volatile uint32_t TerminalNumOfSamples = 0u; /* Number of samples for terminal streaming */
volatile uint8_t TerminalPort = 0u;          /* Port number for terminal streaming */
All_Data TerminalFunction;                   /* Function pointer or struct for terminal streaming */
uint32_t TerminalTimeout = 0u;               /* Timeout value for terminal streaming */
uint8_t StreamMode = 0u;                     /* Streaming mode selector (port or terminal) */
uint8_t StopeCliStreamFlag = 0u;             /* Flag to stop CLI streaming */
/* General streaming variable */
uint32_t SampleCount = 0u;                   /* Total sample counter */


/* Private function prototypes -----------------------------------------------*/
void StreamTimeCallback(TimerHandle_t xTimerStream);
Module_Status SampleToTerminal(uint8_t dstPort, All_Data dataFunction) ;
Module_Status Exporttoport(uint8_t module,uint8_t port,All_Data function);
Module_Status Exportstreamtoport (uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout);
Module_Status Exportstreamtoterminal(uint32_t Numofsamples, uint32_t timeout,uint8_t Port,All_Data function);
void LipoChargerTask(void *argument);
void ExecuteMonitor(void);
void FLASH_Page_Eras(uint32_t Addr );
Module_Status ConvertTwosComplToDec(uint16_t twosComplVal, int16_t *sgnDecimalVal);
Module_Status BAT_ReadIdReg(uint16_t regAddress, uint16_t *Buffer, uint8_t NoBytes);
static Module_Status StreamMemsToCLI(uint32_t Numofsamples, uint32_t timeout, SampleMemsToString function);
void MX_TIM1_Init(void);
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
	 * The SBF status flag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the flag */
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

/*-----------------------------------------------------------*/
/* --- H05R0 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H05R0_OK;


	switch(code){
	case CODE_H05R0_CELLVOLTAGE:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batVolt);
		break;
	}

	case CODE_H05R0_CELLCURRENT:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batCurrent);
		break;
	}

	case CODE_H05R0_CELLPOWER:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batPower);
		break;
	}

	case CODE_H05R0_CELLTEMPERATURE:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],Temp);
		break;
	}

	case CODE_H05R0_CELLCAPACITY:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batCapacity);
		break;
	}

	case CODE_H05R0_CELLSTATEOFCHARGE:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batSOC);
		break;
	}

	case CODE_H05R0_CELLESTIMATEDTTE:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batTTE);
		break;
	}

	case CODE_H05R0_CELLESTIMATEDTTF:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batTTF);
		break;
	}

	case CODE_H05R0_CELLAGE:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batAge);
		break;
	}

	case CODE_H05R0_CELLCYCLES:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batCycles);
		break;
	}

	case CODE_H05R0_CELLCALINTERRES:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],batIntResistance);
		break;
	}

	case CODE_H05R0_SETCHARGVOLTAGE:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],setChargVolt);
		break;
	}

		case CODE_H05R0_SETCHARGCURRENT:
	{
		SampleToPort(cMessage[port-1][shift],cMessage[port-1][1+shift],setChargCurrent);
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

	if(huart->Instance == USART4)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P3;
	else if(huart->Instance == USART1)
		return P4;
	else if(huart->Instance == USART6)
		return P5;
	else if(huart->Instance == USART5)
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

/***************************************************************************/
/* Samples a module parameter value based on parameter index.
 * paramIndex: Index of the parameter (1-based index).
 * value: Pointer to store the sampled float value.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
	Module_Status status = BOS_OK;

	switch (paramIndex) {

	/* Invalid parameter index */
	default:
		status = BOS_ERR_WrongParam;
		break;
	}

	return status;
}


/* Module special task function (if needed) */
void LipoChargerTask(void *argument){

	 int static timer=0,flag=0;

	/* Infinite loop */
	for(;;){

		/* Read Charging Current */
		CheckChargingStatus();

		/* in case the battery is charging */
		if (AnalogMeasurement.ChargingStatus ==0)
		{
			ReadCellCurrent(&ChargingCurrent);
			ReadCellVoltage(&ChargingVolt);
			ReadCellStateOfCharge(&StateOfCharger);

			 if (StateOfCharger==100)
			{
			FullCharge=BATTARY_FULL;
			ChargStatus=CHARGING;
			TIM1->CCR1 = MAX_CCR_VALUE;
			}
			 if (StateOfCharger<99)
			{
				 ChargStatus=CHARGING;
				 FullCharge=BATTARY_EMPTY;
			}
			 if ((ChargingVolt*ChargingCurrent)>=0.1&&FullCharge==BATTARY_EMPTY)
			{
				ChargStatus=CHARGING;
				if (timer < MAX_CCR_VALUE && flag == 0)
				{
					timer += 200;
					if (timer >= MAX_CCR_VALUE)
						flag = 1;
				} else if (flag == 1)
				{
					timer -= 200;
					if (timer <= 0)
						flag = 0;
				}
				/* increase CCR */
				TIM1->CCR1 = timer;
			}
			 if ((ChargingVolt*ChargingCurrent)<0.1&&FullCharge==BATTARY_EMPTY)
			{
			ChargStatus=DISCHARGING;//Stop the loads and wait for the current to increase
			TIM1->CCR1 = 0;
			}

		}
		else if (AnalogMeasurement.ChargingStatus == 1&&ChargingCurrent<0)
			{
			ChargStatus=DISCHARGING;
			FullCharge=BATTARY_EMPTY;
			TIM1->CCR1 = 0;
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
void StreamTimeCallback(TimerHandle_t xTimerStream){
	/* Increment sample counter */
	++SampleCount;

	/* Stream mode to port: Send samples to port */
	if(STREAM_MODE_TO_PORT == StreamMode){
		if((SampleCount <= PortNumOfSamples) || (0 == PortNumOfSamples)){
			SampleToPort(PortModule,PortNumber,PortFunction);

		}
		else{
			SampleCount =0;
			xTimerStop(xTimerStream,0);
		}
	}
	/* Stream mode to terminal: Export to terminal */
	else if(STREAM_MODE_TO_TERMINAL == StreamMode){
		if((SampleCount <= TerminalNumOfSamples) || (0 == TerminalNumOfSamples)){
			SampleToTerminal(TerminalPort,TerminalFunction);
		}
		else{
			SampleCount =0;
			xTimerStop(xTimerStream,0);
		}
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
		tempBuffer[1] = (uint8_t) (regAddress>>8);
		i2cSize = 2;
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
	MX_I2C_SMBUS_Init();

	HAL_SMBUS_IsDeviceReady(&hsmbus2, 0x16, 5, 1000);

	if (H05R0_OK != ReadIdReg(MANFCTR_NAME_REG_ADD, BatId->ManId,MANFCTR_NAME_SIZE))
			Status = H05R0_ERROR;

//	if (H05R0_OK != ReadIdReg(DEVICE_NAME_REG_ADD, BatId->DevId,DEVICE_NAME_SIZE))
//		Status = H05R0_ERROR;


	/* de-initialize SMBUS and initialize I2C protocol to let the I2C port run in standard condition
	 * whenever it is called */
	HAL_SMBUS_DeInit(SMBUS_PORT);
	MX_I2C_Init();

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
 * @brief: read  charger Current measurement
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadChargerCurrent(float *ChargerCurrent)
{
	Module_Status Status = H05R0_ERROR;
	uint32_t tempVar = 0u;
	if (H05R0_OK ==ReadADCValue(&hadc1,ADC_CHANNEL8,&tempVar,100))
		Status = H05R0_OK;

	   *ChargerCurrent=(((tempVar*2.5)/(4096))/(GAIN_CHARGER_VAL)/SENSE_CHARGER_VAL);

	   return Status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: read  VBUS Voltage
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadVBUSVoltage(float *VBUSVolt)
{
	Module_Status Status = H05R0_ERROR;
	uint32_t tempVar = 0u;
	if (H05R0_OK ==ReadADCValue(&hadc1,ADC_CHANNEL9,&tempVar,100))
		Status = H05R0_OK;

	   *VBUSVolt=(((tempVar*2.5)/(4096))*3.35);

	   return Status;

}

/*-----------------------------------------------------------*/
/*
 * @brief: MCU LDO Enable To secure feeding for the processor after the charger is separated
 */
Module_Status MCULDOEnable(Out_State PinState)
{
	Module_Status Status = H05R0_OK;
	if (PinState == ENABLE_OUT) {
		HAL_GPIO_WritePin(MCU_LDO_EN_GPIO_Port, MCU_LDO_EN_Pin, GPIO_PIN_SET);
	}
	else
		HAL_GPIO_WritePin(MCU_LDO_EN_GPIO_Port, MCU_LDO_EN_Pin, GPIO_PIN_RESET);
	return Status;
}

/*-----------------------------------------------------------*/
/*
 * @brief: MCU Out Volt Enable To secure 3.3V for other Modules
 */
Module_Status MCUOutVoltEnable( Out_State PinState)
{
	Module_Status Status = H05R0_OK;
	if (PinState == ENABLE_OUT) {
		HAL_GPIO_WritePin(OUT_EN_3V3_GPIO_Port, OUT_EN_3V3_Pin, GPIO_PIN_SET);
	}
	else
		HAL_GPIO_WritePin(OUT_EN_3V3_GPIO_Port, OUT_EN_3V3_Pin, GPIO_PIN_RESET);

	return Status;
}
/*-----------------------------------------------------------*/
/*
 * @brief: VBUS Output Switch Enable To Load Control
 */
Module_Status VBUSOutSwitchEnable( Out_State PinState)
{
	Module_Status Status = H05R0_OK;
	if (PinState == ENABLE_OUT) {
		HAL_GPIO_WritePin(VBUS_OUT_EN_GPIO_Port, VBUS_OUT_EN_Pin,GPIO_PIN_RESET);
	}
	else
		HAL_GPIO_WritePin(VBUS_OUT_EN_GPIO_Port, VBUS_OUT_EN_Pin, GPIO_PIN_SET);

	return Status;
}
/*-----------------------------------------------------------*/

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

/*-----------------------------------------------------------*/

/*
 * @brief: read number of remaining non-volatile memory writes
 * @param1: pointer to a buffer to store received data
 * @retval: status
 */
Module_Status ReadNumOfRemainingWrites(uint8_t *updatesRemaining)
{
	Module_Status Status = H05R0_OK;
	uint16_t tempVar = 0u;
	uint8_t updatesUsed = 0 , index = 0 , combinedData = 0;
	uint8_t totalUpdates = 7 ;

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

	    // Calculate updates remaining
	     *updatesRemaining = totalUpdates - updatesUsed;

	return Status;
}

/***************************************************************************/
/**
 * @brief  Streams a single sensor data sample to the terminal.
 * @param  dstPort: Port number to stream data to.
 * @param  dataFunction: Function to sample data (e.g., batVolt, batCurrent, batPower, Temp, etc.).
 * @param  numOfSamples: Number of samples (kept for compatibility, not used for repetition).
 * @param  streamTimeout: Timeout period for the operation (in milliseconds).
 * @retval Module_Status indicating success or failure of the operation.
 */
Module_Status SampleToTerminal(uint8_t dstPort, All_Data dataFunction) {
	Module_Status status = H05R0_OK; /* Initialize operation status as success */
	int8_t *pcOutputString = NULL; /* Pointer to CLI output buffer */
	uint32_t period = 0u; /* Calculated period for the operation */
	char cstring[100] = { 0 }; /* Buffer for formatted output string */

	/* Process data based on the requested sensor function */
	switch (dataFunction) {
	case batVolt: {
		float voltage = 0.0f;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample battery voltage data */
		ReadCellVoltage(&voltage);

		/* Format battery voltage data into a string */
		snprintf(cstring, sizeof(cstring), "CellVoltage | Voltage: %.2f\r\n",
				voltage);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case batCurrent: {
		float current = 0.0f;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample battery current data */
		ReadCellCurrent(&current) ;

		/* Format battery current data into a string */
		snprintf(cstring, sizeof(cstring), "CellCurrent | Current: %.2f\r\n",
				current);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case batPower: {
		float power = 0.0f;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample battery power data */
		ReadCellPower(&power);

		/* Format battery power data into a string */
		snprintf(cstring, sizeof(cstring), "CellPower | Power: %.2f\r\n",
				power);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case Temp: {
		float temperature = 0.0f;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample temperature data */
		ReadTemperature(&temperature) ;

		/* Format temperature data into a string */
		snprintf(cstring, sizeof(cstring),
				"Temperature | Temperature: %.2f\r\n", temperature);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case batCapacity: {
		float capacity = 0.0f;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample battery capacity data */
		ReadCellCapacity(&capacity);

		/* Format battery capacity data into a string */
		snprintf(cstring, sizeof(cstring), "CellCapacity | Capacity: %.2f\r\n",
				capacity);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case batSOC: {
		uint8_t stateOfCharge = 0;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample battery state of charge data */
		ReadCellStateOfCharge(&stateOfCharge) ;
		/* Format battery state of charge data into a string */
		snprintf(cstring, sizeof(cstring), "CellStateOfCharge | Charge: %d\r\n",
				stateOfCharge);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case batTTE: {
		uint32_t timeToEmpty = 0;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample battery estimated time to empty data */
		ReadCellEstimatedTTE(&timeToEmpty) ;

		/* Format battery time to empty data into a string */
		snprintf(cstring, sizeof(cstring), "CellEstimatedTTE: %u\r\n",
				timeToEmpty);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case batTTF: {
		uint32_t timeToFull = 0;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample battery estimated time to full data */
		ReadCellEstimatedTTF(&timeToFull) ;

		/* Format battery time to full data into a string */
		snprintf(cstring, sizeof(cstring), "CellEstimatedTTF: %u\r\n",
				timeToFull);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case batAge: {
		uint8_t age = 0;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample battery age data */
		ReadCellAge(&age);

		/* Format battery age data into a string */
		snprintf(cstring, sizeof(cstring), "CellAge: %d\r\n", age);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case batCycles: {
		uint16_t cycles = 0;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample battery cycles data */
		ReadCellCycles(&cycles);

		/* Format battery cycles data into a string */
		snprintf(cstring, sizeof(cstring), "CellCycles | Cycles: %d\r\n",
				cycles);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case setChargVolt: {
		float chargeVoltage = 0.0f;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample set charge voltage data */
		ReadSetChargVoltage(&chargeVoltage) ;

		/* Format set charge voltage data into a string */
		snprintf(cstring, sizeof(cstring),
				"SetChargVoltage | Voltage: %.2f\r\n", chargeVoltage);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case setChargCurrent: {
		float chargeCurrent = 0.0f;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample set charge current data */
		ReadSetChargCurrent(&chargeCurrent) ;

		/* Format set charge current data into a string */
		snprintf(cstring, sizeof(cstring),
				"SetChargCurrent | Current: %.2f\r\n", chargeCurrent);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case batIntResistance: {
		/* Note: This case is empty in the original code, leaving it as a placeholder */
		return H05R0_ERR_WrongParams; /* Return error as no implementation exists */
	}

	default:
		return H05R0_ERR_WrongParams; /* Return error for invalid sensor function */
	}

	/* Return final status indicating success or prior error */
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
			if (UARTRxBuf[pcPort - 1][chr] == '\r' && Numofsamples > 0) {
				UARTRxBuf[pcPort - 1][chr] = 0;
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
Module_Status CheckChargingStatus(void)
{
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
/**
 * @brief  Samples data from a sensor and exports it to a specified port or module.
 * @param  dstModule: The module number to export data to.
 * @param  dstPort: The port number to export data to.
 * @param  dataFunction: Function to sample data (e.g., batVolt, batCurrent, batPower, Temp, etc.).
 * @retval Module_Status indicating success or failure of the operation.
 */
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction)
{
    static uint8_t temp[6] = {0};       /* Buffer for data transmission */
    Module_Status status = H05R0_OK;    /* Initialize operation status as success */

    /* Check if the port and module ID are valid */
    if (dstPort == 0 && dstModule == myID)
    {
        return H05R0_ERR_WrongParams;   /* Return error for invalid parameters */
    }

    /* Process data based on the requested sensor function */
    switch (dataFunction)
    {
        case batVolt:
        {
            float voltage = 0.0f;
            status = ReadCellVoltage(&voltage);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                /* Pack data into temp buffer */
                memcpy(temp, &voltage, sizeof(float));
                writePxITMutex(dstPort, (char*)temp, sizeof(float), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_FLOAT;
                MessageParams[2] = 1;
                memcpy(&MessageParams[3], &voltage, sizeof(float));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(float) + 3);
            }
            break;
        }

        case batCurrent:
        {
            float current = 0.0f;
            status = ReadCellCurrent(&current);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                /* Pack data into temp buffer */
                memcpy(temp, &current, sizeof(float));
                writePxITMutex(dstPort, (char*)temp, sizeof(float), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_FLOAT;
                MessageParams[2] = 1;
                memcpy(&MessageParams[3], &current, sizeof(float));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(float) + 3);
            }
            break;
        }

        case batPower:
        {
            float power = 0.0f;
            status = ReadCellPower(&power);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                /* Pack data into temp buffer */
                memcpy(temp, &power, sizeof(float));
                writePxITMutex(dstPort, (char*)temp, sizeof(float), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_FLOAT;
                MessageParams[2] = 1;
                memcpy(&MessageParams[3], &power, sizeof(float));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(float) + 3);
            }
            break;
        }

        case Temp:
        {
            float temperature = 0.0f;
            status = ReadTemperature(&temperature);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                /* Pack data into temp buffer */
                memcpy(temp, &temperature, sizeof(float));
                writePxITMutex(dstPort, (char*)temp, sizeof(float), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_FLOAT;
                MessageParams[2] = 1;
                memcpy(&MessageParams[3], &temperature, sizeof(float));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(float) + 3);
            }
            break;
        }

        case batCapacity:
        {
            float capacity = 0.0f;
            status = ReadCellCapacity(&capacity);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                /* Pack data into temp buffer */
                memcpy(temp, &capacity, sizeof(float));
                writePxITMutex(dstPort, (char*)temp, sizeof(float), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_FLOAT;
                MessageParams[2] = 1;
                memcpy(&MessageParams[3], &capacity, sizeof(float));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(float) + 3);
            }
            break;
        }

        case batSOC:
        {
            uint8_t stateOfCharge = 0;
            status = ReadCellStateOfCharge(&stateOfCharge);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                temp[0] = stateOfCharge;
                writePxITMutex(dstPort, (char*)temp, sizeof(uint8_t), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_UINT8;
                MessageParams[2] = 1;
                MessageParams[3] = stateOfCharge;

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint8_t) + 3);
            }
            break;
        }

        case batTTE:
        {
            uint32_t timeToEmpty = 0;
            status = ReadCellEstimatedTTE(&timeToEmpty);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                /* Pack data into temp buffer */
                memcpy(temp, &timeToEmpty, sizeof(uint32_t));
                writePxITMutex(dstPort, (char*)temp, sizeof(uint32_t), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_UINT32;
                MessageParams[2] = 1;
                memcpy(&MessageParams[3], &timeToEmpty, sizeof(uint32_t));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint32_t) + 3);
            }
            break;
        }

        case batTTF:
        {
            uint32_t timeToFull = 0;
            status = ReadCellEstimatedTTF(&timeToFull);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                /* Pack data into temp buffer */
                memcpy(temp, &timeToFull, sizeof(uint32_t));
                writePxITMutex(dstPort, (char*)temp, sizeof(uint32_t), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_UINT32;
                MessageParams[2] = 1;
                memcpy(&MessageParams[3], &timeToFull, sizeof(uint32_t));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint32_t) + 3);
            }
            break;
        }

        case batAge:
        {
            uint8_t age = 0;
            status = ReadCellAge(&age);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                temp[0] = age;
                writePxITMutex(dstPort, (char*)temp, sizeof(uint8_t), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_UINT8;
                MessageParams[2] = 1;
                MessageParams[3] = age;

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint8_t) + 3);
            }
            break;
        }

        case batCycles:
        {
            uint16_t cycles = 0;
            status = ReadCellCycles(&cycles);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                temp[0] = (uint8_t)(cycles);
                temp[1] = (uint8_t)(cycles >> 8);
                writePxITMutex(dstPort, (char*)temp, sizeof(uint16_t), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_UINT16;
                MessageParams[2] = 1;
                MessageParams[3] = (uint8_t)(cycles);
                MessageParams[4] = (uint8_t)(cycles >> 8);

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint16_t) + 3);
            }
            break;
        }

        case batIntResistance:
        {
            float resistance = 0.0f;
            /* Note: Original code lacks sampling function, assuming placeholder */
            status = H05R0_ERR_WrongParams;  /* Return error as no implementation exists */

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                memcpy(temp, &resistance, sizeof(float));
                writePxITMutex(dstPort, (char*)temp, sizeof(float), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = BOS_ERROR;
                MessageParams[0] = FMT_FLOAT;
                MessageParams[2] = 1;
                memcpy(&MessageParams[3], &resistance, sizeof(float));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(float) + 3);
            }
            break;
        }

        case setChargVolt:
        {
            float chargeVoltage = 0.0f;
            status = ReadSetChargVoltage(&chargeVoltage);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                /* Pack data into temp buffer */
                memcpy(temp, &chargeVoltage, sizeof(float));
                writePxITMutex(dstPort, (char*)temp, sizeof(float), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_FLOAT;
                MessageParams[2] = 1;
                memcpy(&MessageParams[3], &chargeVoltage, sizeof(float));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(float) + 3);
            }
            break;
        }

        case setChargCurrent:
        {
            float chargeCurrent = 0.0f;
            status = ReadSetChargCurrent(&chargeCurrent);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                /* Pack data into temp buffer */
                memcpy(temp, &chargeCurrent, sizeof(float));
                writePxITMutex(dstPort, (char*)temp, sizeof(float), 10);
            }
            else
            {
                /* Send data to another module */
                MessageParams[1] = (status == H05R0_OK) ? BOS_OK : BOS_ERROR;
                MessageParams[0] = FMT_FLOAT;
                MessageParams[2] = 1;
                memcpy(&MessageParams[3], &chargeCurrent, sizeof(float));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(float) + 3);
            }
            break;
        }

        default:
            return H05R0_ERR_WrongParams;  /* Return error for invalid sensor function */
    }

    /* Clear the temp buffer */
    memset(temp, 0, sizeof(temp));

    /* Return final status indicating success or prior error */
    return status;
}

//Module_Status SampleToPort(uint8_t module,uint8_t port,All_Data function)
// {
//	Module_Status status = H05R0_OK;
//	tofMode = SAMPLE_TO_PORT;
//	port2 = port;
//	module2 = module;
//	mode2 = function;
//	return status;
//}


/*
 * Sending a Stream of the required module on the required port
 * 	period=timeout/Numofsamples -->minimum=100
 * 	The minimum time between one sample and another is its value 100ms
 *
 */
//Module_Status StreamtoPort(uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout)
//{
//	Module_Status status = H05R0_OK;
//	tofMode=STREAM_TO_PORT;
//	port1 = port ;
//	module1 =module;
//	Numofsamples1=Numofsamples;
//	timeout1=timeout;
//	mode1= function;
//	return status;
//
//}


//Module_Status Exportstreamtoport (uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout)
// {
//	Module_Status status = H05R0_OK;
//	uint32_t samples = 0;
//	uint32_t period = 0;
//	period = timeout / Numofsamples;
//
//	if (timeout < MIN_PERIOD_MS || period < MIN_PERIOD_MS)
//		return H05R0_ERR_WrongParams;
//
//	while (samples < Numofsamples) {
//		status = Exporttoport(module, port, function);
//		vTaskDelay(pdMS_TO_TICKS(period));
//		samples++;
//	}
//	tofMode = DEFAULT;
//	samples = 0;
//	return status;
//}

static Module_Status StreamMemsToCLI(uint32_t Numofsamples, uint32_t timeout, SampleMemsToString function)
{
	Module_Status status = H05R0_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H05R0_ERR_WrongParams;

	// TODO: Check if CLI is enable or not
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[pcPort - 1][chr] == '\r' ) {
				UARTRxBuf[pcPort - 1][chr] = 0;
			}
		}
	if (1 == flag) {
		flag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(pcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}
	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char *)pcOutputString, 100);


		writePxMutex(pcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
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

/***************************************************************************/
/*
 * brief: Streams data to the specified port and module with a given number of samples.
 * param targetModule: The target module to which data will be streamed.
 * param portNumber: The port number on the module.
 * param portFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToPort(uint8_t dstModule,uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout){
	Module_Status Status =H05R0_OK;
	uint32_t SamplePeriod =0u;

	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)){
		return H05R0_ERROR; /* Assuming H05R0_ERROR is defined in Module_Status */
	}

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_PORT;
	PortModule =dstModule;
	PortNumber =dstPort;
	PortFunction =dataFunction;
	PortNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100)){
			return H05R0_ERROR;
		}
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100)){
		return H05R0_ERROR;
	}

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100)){
		return H05R0_ERROR;
	}

	return Status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified terminal port with a given number of samples.
 * param targetPort: The port number on the terminal.
 * param dataFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToTerminal(uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout){
	Module_Status Status =H05R0_OK;
	uint32_t SamplePeriod =0u;
	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)){
		return H05R0_ERROR; /* Assuming H05R0_ERROR is defined in Module_Status */
	}

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_TERMINAL;
	TerminalPort =dstPort;
	TerminalFunction =dataFunction;
	TerminalTimeout =streamTimeout;
	TerminalNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100)){
			return H05R0_ERROR;
		}
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100)){
		return H05R0_ERROR;
	}

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100)){
		return H05R0_ERROR;
	}

	return Status;
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
