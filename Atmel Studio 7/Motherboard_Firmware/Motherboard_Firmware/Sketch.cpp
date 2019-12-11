/*===========================================================
* Project: Motherboard_Firmware
* Developed by: Anthony Kaul(3D Excellence LLC) in collaboration with Filabot (Triex LLC)
* This firmware converts SPC (Statistical Process Control) into signals that can be recognized by a PC
* This firmware also allows for communication to various devices via serial and coordinates all items
* All rights reserved


* Version information:
* v1.2 - beta
* ===========================================================*/

// ***** INCLUDES ***** //
#include <Arduino.h>
#include <FreeRTOS_ARM.h>
#include "SerialProcessing.h"
#include "hardwareTypes.h"
#include "SpcProcessing.h"
#include "Screen.h"
#include "board.h"
#include "Error.h"
#include "DataConversions.h"
#include "SerialNative.h"
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "NVM_Operations.h"

// ***** INCLUDES ***** //

// ***** FreeRTOS  ***** //
#define INCLUDE_vTaskDelay   1
#define configUSE_PREEMPTION 1
#define configKERNEL_INTERRUPT_PRIORITY     0xFF
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    0x10
// Redefine AVR Flash string macro as nop for ARM
#undef F
#define F(str) str
// ***** FreeRTOS  ***** //


// ***** TASKS **** //
#define TASKCHECKSPC
#define TASKCHECKSERIALCOMMANDS
#define TASKGETPULLERDATA
#define TASKGETTRAVERSEDATA
#define TASKCHECKENCODER
#define TASKGETFULLUPDATE
//#define TASKFILAMENTCAPTURE
#define TASKCALCULATE
#define TASKHANDSHAKE
// ***** TASKS **** //



// **** PROTOTYPES **** //
void CheckSerialCommands();
void RunSPCDataLoop();
int CheckInteralCommands(char* code);
int CheckSpoolerCommands(char* code);
int PrintRandomDiameterData();
void PrintRandomRPMData();
bool startsWith(const char* pre, const char* str);
void TaskCheckSPC( void *pvParameters );
void TaskCheckSerialExpander( void *pvParameters );
void TaskCheckSerialCommands( void *pvParameters );
void TaskRunSimulation(void *pvParameters );
void TaskGetPullerData (void *pvParameters);
void TaskGetTraverseData (void *pvParameters);
void TaskCheckEncoder (void *pvParameters);
void TaskGetFullUpdate (void *pvParameters);
void TaskFilamentCapture (void *pvParameters);
void TaskCalculate (void *pvParameters);
void TaskHandshake (void *pvParameters);
void checkSPC();
// **** PROTOTYPES **** //



// These are used to get information about static SRAM and flash memory sizes
extern "C" char __data_start[];    // start of SRAM data
extern "C" char _end[];     // end of SRAM data (used to check amount of SRAM this program's variables use)
extern "C" char __data_load_end[];  // end of FLASH (used to check amount of Flash this program's code and data uses)

// declare since extern in board.h
bool SIMULATIONACTIVE = false; //sets default value for simulation
_SerialNative SerialNative;
uint32_t SPOOLWEIGHT = 0;
float FILAMENTLENGTH = 0.0;
float FILAMENTDIAMETER = 0.0;
volatile bool HANDSHAKE = false;



// ***** CLASS DECLARATIONs **** //
SerialCommand sCommand;
SpcProcessing spcProcessing;
SerialProcessing serialProcessing;
SemaphoreHandle_t xSemaphore = NULL;
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
NVM_Operations nvm_operations;

#ifdef TASKCHECKENCODER
Encoder encoder(ENCODER_PINA, ENCODER_PINB);
#endif
// ***** CLASS DECLARATIONs **** //


float pullerRPM = 0;
int32_t previousEncoderValue = 0;
unsigned long previousMillis = 834000;
unsigned int fullUpdateCounter = 0;
unsigned int traverseDataCounter = 0;
unsigned int pullerDataCounter = 0;
bool previousCaptureState = false;


float previousLength = 0;
float spoolWeight = 0.0;
int64_t feedRate = 0.0;
int64_t previousFeedrateSampleTime = 0;


void setup()
{


	uint32_t RST_status = (RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos; // Get status from the last Reset
	uint32_t CORE_status = (SUPC->SUPC_SR); // Get status from the last Core Reset

	if (CORE_status == SUPC_SR_BODRSTS)
	{
		int i = 0;
		i = i + 1;
	}
	
	
	if (RST_status == RSTC_SR_RSTTYP_GeneralReset)
	{
		int i = 0;
		i = i + 1;
	}
	if (RST_status == RSTC_SR_RSTTYP_BackupReset)
	{
		int i = 0;
		i = i + 1;
	}
	if (RST_status == RSTC_SR_RSTTYP_WatchdogReset)
	{
		int i = 0;
		i = i + 1;
	}
	if (RST_status == RSTC_SR_RSTTYP_SoftwareReset)
	{
		int i = 0;
		i = i + 1;
	}
	if (RST_status == RSTC_SR_RSTTYP_UserReset)
	{
		int i = 0;
		i = i + 1;
	}



	SerialNative.begin(SERIAL_BAUD); //using native serial rather than programming port on DUE
	SerialNative.setTimeout(1);

	ads.begin();
	ads.setGain(GAIN_ONE);

	pinMode(ENCODER_PB, INPUT_PULLUP);
	pinMode(START_PB, INPUT_PULLUP);
	pinMode(STOP_PB, INPUT_PULLUP);

	


	// **** INITS ***** //
	nvm_operations.init();
	spcProcessing.init();
	serialProcessing.init();
	startErrorHandler();
	// **** INITS ***** //

	// ***** Instances ***** //
	xSemaphore = xSemaphoreCreateMutex();

	#ifdef TASKCHECKSPC
	xTaskCreate(
	TaskCheckSPC
	,  (const portCHAR *)"CheckSPC"   // A name just for humans
	,  1000  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,   1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );
	#endif
	
	#ifdef TASKCHECKSERIALCOMMANDS
	xTaskCreate(
	TaskCheckSerialCommands
	,  (const portCHAR *)"CheckSerialCommands"   // A name just for humans
	,  2000  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );
	#endif

	#ifdef TASKGETPULLERDATA
	xTaskCreate(
	TaskGetPullerData
	,  (const portCHAR *)"GetPullerDATA"   // A name just for humans
	,  250  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );
	#endif

	#ifdef TASKGETTRAVERSEDATA
	xTaskCreate(
	TaskGetTraverseData
	,  (const portCHAR *)"GetTraverseData"   // A name just for humans
	,  250  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );
	#endif

	#ifdef TASKCHECKENCODER
	xTaskCreate(
	TaskCheckEncoder
	,  (const portCHAR *)"CheckEncoder"   // A name just for humans
	,  500  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );
	#endif
	
	#ifdef TASKGETFULLUPDATE
	xTaskCreate(
	TaskGetFullUpdate
	,  (const portCHAR *)"GetFullUpdate"   // A name just for humans
	,  250  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );
	#endif

	#ifdef TASKFILAMENTCAPTURE
	xTaskCreate(
	TaskFilamentCapture
	,  (const portCHAR *)"FilamentCapture"   // A name just for humans
	,  250  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );
	#endif

	#ifdef TASKCALCULATE
	xTaskCreate(
	TaskCalculate
	,  (const portCHAR *)"TaskCalculate"   // A name just for humans
	,  250  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );
	#endif

	#ifdef TASKHANDSHAKE
	xTaskCreate(
	TaskHandshake
	,  (const portCHAR *)"TaskHandshake"   // A name just for humans
	,  250  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );
	#endif

	// ***** Instances ***** //

	vTaskStartScheduler(); //start FreeRTOS scheduler
	SerialNative.println("Insufficient RAM"); //code execution should never get here, but could if there is not enough RAM
	while(1); //hang processor on error
}

void loop()
{
	//nothing to do here, all routines done in tasks

	
}

void TaskCheckSPC(void *pvParameters)  // This is a task.
{

	while(1) // A Task shall never return or exit.
	{
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();
			int32_t waitTime = 25;

			if (HANDSHAKE)
			{

				if (spcProcessing.ISR_READY() && !spcProcessing.HasNewData)
				{
					spcProcessing.StartQuery();//enable interrupts and start the bit gathering from spc
				}
				
				if (spcProcessing.HasNewData)
				{
					spcProcessing.HasNewData = false;
					//SerialUSB.println(spcProcessing.GetDiameter()->charDiameterWithDecimal); //Serial print is broken using long values, use char instead
					SerialCommand _serialCommand;
					_serialCommand.hardwareType = hardwareType.indicator;
					_serialCommand.command = "Diameter";
					_serialCommand.value = spcProcessing.GetDiameter()->charDiameterWithDecimal;
					char output[MAX_CMD_LENGTH] = {0};
					
					FILAMENTDIAMETER = spcProcessing.GetDiameter()->floatDiameterWithDecimal;
					BuildSerialOutput(&_serialCommand, output);
					SerialNative.println(output);
					
				}
			}
			
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil( &xLastWakeTime, waitTime);
		}
		
	}

}

void TaskCheckSerialCommands(void *pvParameters)  // This is a task.
{
	while(1) // A Task shall never return or exit.
	{
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();

			serialProcessing.Poll();
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil( &xLastWakeTime, 10);
		}
	}
}

void TaskGetPullerData(void *pvParameters)  // This is a task.
{
	while(1) // A Task shall never return or exit.
	{
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();

			if (HANDSHAKE){
				SerialCommand command = {0};
				
				switch(pullerDataCounter)
				{
					case 0:
					command.command = "velocity";
					command.hardwareType = hardwareType.puller;
					command.value = NULL;
					pullerDataCounter++;
					break;
					case 1:
					command.command = "PullerRPM";
					command.hardwareType = hardwareType.puller;
					command.value = NULL;
					pullerDataCounter++;
					break;
					case 2:
					command.command = "FilamentLength";
					command.hardwareType = hardwareType.puller;
					command.value = NULL;
					pullerDataCounter++;
					break;
					case 3:
					command.command = "Feedrate";
					command.hardwareType = hardwareType.puller;
					command.value = NULL;
					pullerDataCounter++;
					break;
					default:
					pullerDataCounter = 0;
					break;
				}
				
				if (!serialProcessing.FullUpdateRequested && command.hardwareType != NULL)
				{
					serialProcessing.SendDataToDevice(&command);
					delay(15); //puller likes to take its sweet time responding, need at least 10ms to sync back...
				}
			}

			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil( &xLastWakeTime, 200);
		}

		
	}

}

void TaskGetTraverseData(void *pvParameters)  // This is a task.
{
	while(1) // A Task shall never return or exit.
	{
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();
			if (HANDSHAKE){
				SerialCommand command = {0};
				
				switch(traverseDataCounter)
				{
					case 0:
					command.command = "SpoolRPM";
					command.hardwareType = hardwareType.traverse;
					command.value = NULL;
					traverseDataCounter++;
					break;

					case 1:
					command.command = "SpoolTicks";
					command.hardwareType = hardwareType.traverse;
					command.value = NULL;
					traverseDataCounter++;
					break;

					default:
					traverseDataCounter = 0;
					break;
				}
				if (!serialProcessing.FullUpdateRequested && command.hardwareType != NULL)
				{
					serialProcessing.SendDataToDevice(&command);
				}
				
				
			}
			//vTaskDelay(10);
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil( &xLastWakeTime, 200);
		}

		
	}

}

void TaskCheckEncoder(void *pvParameters)  // This is a task.
{
	while(1) // A Task shall never return or exit.
	{
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();

			#ifdef TASKCHECKENCODER
			
			int32_t encoderValue = encoder.read();
			if (previousEncoderValue != encoderValue)
			{
				
				SerialCommand sCommand = {0};
				sCommand.hardwareType = hardwareType.puller;

				//char decimalNumber[20] = {0};
				//CONVERT_FLOAT_TO_STRING(encoderValue, decimalNumber);
				
				
				if (encoderValue < previousEncoderValue)
				{
					sCommand.command = "increase_rpm";
				}
				if (encoderValue > previousEncoderValue)
				{
					sCommand.command = "decrease_rpm";
				}
				
				char value[MAX_CMD_LENGTH] = {0};
				CONVERT_NUMBER_TO_STRING(INT_FORMAT, abs(encoderValue - previousEncoderValue), value);
				
				sCommand.value = value;
				
				if (!serialProcessing.FullUpdateRequested && sCommand.hardwareType != NULL)
				{
					serialProcessing.SendDataToDevice(&sCommand);
					delay(10);
				}
				previousEncoderValue = encoderValue;
			}
			
			#endif

			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil( &xLastWakeTime, 20);
		}

		
	}

}

void TaskGetFullUpdate(void *pvParameters)  // This is a task.
{
	while(1) // A Task shall never return or exit.
	{
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();

			if (HANDSHAKE){
				if (serialProcessing.FullUpdateRequested)
				{
					SerialCommand command = {0};
					switch(fullUpdateCounter)
					{
						
						case 0:
						command.command = "velocity";
						command.hardwareType = hardwareType.puller;
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 1:
						command.command = "InnerOffset";
						command.hardwareType = hardwareType.traverse;
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 2:
						command.command = "SpoolWidth";
						command.hardwareType = hardwareType.traverse;
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 3:
						command.command = "RunMode";
						command.hardwareType = hardwareType.traverse;
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 4:
						command.command = "StartPosition";
						command.hardwareType = hardwareType.traverse;
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 5:
						command.command = "SpecificGravity";
						command.hardwareType = hardwareType.internal;
						command.value = nvm_operations.GetSpecificGravity();
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						default:
						fullUpdateCounter = 0;
						serialProcessing.FullUpdateRequested = false;
						break;
					}
					
				}
			}
			
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil( &xLastWakeTime, 60);
		}

		
	}

}

void TaskFilamentCapture(void *pvParameters)  // This is a task.
{
	while(1) // A Task shall never return or exit.
	{
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();
			if (HANDSHAKE)
			{
				if (previousCaptureState != serialProcessing.FilamentCapture )
				{
					

					char value[MAX_CMD_LENGTH] = {0};
					CONVERT_NUMBER_TO_STRING(STRING_FORMAT, serialProcessing.FilamentCapture == true ? "1" : "0", value);
					SerialCommand command = {0};
					command.command = "FilamentCapture";
					command.hardwareType = hardwareType.traverse;
					command.value = value;

					if (!serialProcessing.FullUpdateRequested && command.hardwareType != NULL)
					{
						serialProcessing.SendDataToDevice(&command);
						command.hardwareType = hardwareType.puller;
						delay(10);
						serialProcessing.SendDataToDevice(&command);
					}
					previousCaptureState = serialProcessing.FilamentCapture;
				}
			}
			
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil( &xLastWakeTime, 50);
		}

		
	}

}

void TaskCalculate(void *pvParameters)  // This is a task.
{
	while(1) // A Task shall never return or exit.
	{
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();

			if (serialProcessing.FilamentCapture )
			{
				float specificGracity = atof(nvm_operations.GetSpecificGravity());
				int64_t timeDifference = 0;
				uint32_t filamentLengthDelta = 0;
				int64_t currentTime = 0;
				//SPOOLWEIGHT
				if (FILAMENTLENGTH != previousLength)
				{
					if (spoolWeight < 0) {spoolWeight = 0.0;}
					spoolWeight = spoolWeight + (float)((float)(HALF_PI / 2) * pow(FILAMENTDIAMETER, 2) * (FILAMENTLENGTH - previousLength)) * (float)specificGracity;
					////feedRate = FILAMENTLENGTH /
					//currentTime = millis();
					//timeDifference = currentTime - previousFeedrateSampleTime;
					//filamentLengthDelta = ((float)(FILAMENTLENGTH - previousLength) * 1000);
					//feedRate = (filamentLengthDelta * timeDifference) / 1000;
					previousLength = FILAMENTLENGTH;
					//previousFeedrateSampleTime = currentTime;
					//
					//
//
					SPOOLWEIGHT = uint32_t(spoolWeight);
					char value[MAX_CMD_LENGTH] = {0};
					CONVERT_NUMBER_TO_STRING(INT_FORMAT, SPOOLWEIGHT, value);
					
					SerialCommand command = {0};
					command.command = "SpoolWeight";
					command.hardwareType = hardwareType.internal;
					command.value = value;

					if (!serialProcessing.FullUpdateRequested && command.hardwareType != NULL)
					{
						serialProcessing.SendDataToDevice(&command);
					}
//
					//value[MAX_CMD_LENGTH] = {0};
					//CONVERT_NUMBER_TO_STRING(INT_FORMAT, feedRate, value);
					//command.command = "Feedrate";
					//command.hardwareType = hardwareType.internal;
					//command.value = value;
//
					//
					previousCaptureState = serialProcessing.FilamentCapture;
//
					//if (!serialProcessing.FullUpdateRequested && command.hardwareType != NULL)
					//{
						//serialProcessing.SendDataToDevice(&command);
					//}
				}
				
			}
			else
			{
				spoolWeight = 0;
				feedRate = 0;
				previousLength = 0;
				FILAMENTLENGTH = 0;
			}
			
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil( &xLastWakeTime, 200);
		}

		
	}

}

void TaskHandshake(void *pvParameters)  // This is a task.
{
	while(1) // A Task shall never return or exit.
	{
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();

			if (_SerialNative().dtr())
			{
				if (!HANDSHAKE){
					SerialCommand command;
					command.hardwareType = hardwareType.internal;
					command.command = "Handshake";
					command.value = "";

					serialProcessing.SendDataToDevice(&command);
				}
				//HANDSHAKE = true;
			}
			else
			{
				HANDSHAKE = false;
			}
			
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil( &xLastWakeTime, 500);
		}

		
	}

}





