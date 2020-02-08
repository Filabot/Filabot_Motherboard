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
#include "SAM3Timer.h"

// ***** INCLUDES ***** //

// ***** FreeRTOS  ***** //
#define INCLUDE_vTaskDelay   1
#define configUSE_PREEMPTION 1

#ifdef __NVIC_PRIO_BITS
/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
#define configPRIO_BITS       				__NVIC_PRIO_BITS
#else
#define configPRIO_BITS       				4        /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY	0x0f

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	10

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
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
uint32_t SPOOLWEIGHTLIMIT = 0;
float FILAMENTLENGTH = 0.0;
float FILAMENTDIAMETER = 0.0;
volatile bool HANDSHAKE = false;
QueueHandle_t xQueue;


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
//unsigned long previousMillis = 834000;
unsigned int fullUpdateCounter = 0;
unsigned int traverseDataCounter = 0;
unsigned int pullerDataCounter = 0;
bool previousCaptureState = false;
char *restartReason = {0};

float previousLength = 0;
float spoolWeight = 0.0;
int64_t feedRate = 0.0;
int64_t previousFeedrateSampleTime = 0;

int64_t motorPulsesSent = 0;
int64_t motorPulsesReceived = 0;


void TC3_Handler()
{

	TC_GetStatus(TC1, 0);
	
	static bool state = false;

	if (state)
	{
		REG_PIOD_SODR = 0x1 << 5;
	}
	else
	{
		REG_PIOD_CODR = 0x1 << 5;
	}

	//if ((motorPulsesSent - motorPulsesReceived) < 10000)
	//{
		//motorPulsesSent++;
		state = !state;
	//}
	
	

}

void MotorPulse_ISR()
{

	//static int i = 1;
//
	//if (i == 0 )
	//{
		//motorPulsesReceived++;
		//i = 1;
	//}
	//else
	//{
		//i--;
	//}
	
}

void setup()
{
	xQueue = xQueueCreate(10, MAX_CMD_LENGTH);

	uint32_t RST_status = (RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos; // Get status from the last Reset
	uint32_t CORE_status = (SUPC->SUPC_SR); // Get status from the last Core Reset

	if (CORE_status == SUPC_SR_BODRSTS)
	restartReason = "Brownout Detector Reset Status";

	if (CORE_status == SUPC_SR_SMRSTS)
	restartReason = "Supply Monitor Reset Status";
	
	if (RST_status == RSTC_SR_RSTTYP_GeneralReset)
	restartReason = "First power-up Reset";
	
	if (RST_status == RSTC_SR_RSTTYP_BackupReset)
	restartReason = "Return from Backup Mode";

	if (RST_status == RSTC_SR_RSTTYP_WatchdogReset)
	restartReason = "Watchdog fault occurred";

	if (RST_status == RSTC_SR_RSTTYP_SoftwareReset)
	restartReason = "Processor reset required by the software";

	if (RST_status == RSTC_SR_RSTTYP_UserReset)
	restartReason = "NRST pin detected low";
	
	SerialNative.begin(SERIAL_BAUD); //using native serial rather than programming port on DUE
	SerialNative.setTimeout(1);

	pinMode(15, OUTPUT);
	pinMode(14, INPUT);
	SAM3Timer::startTimer(TC1, 0, TC3_IRQn, 0);
	attachInterrupt(digitalPinToInterrupt(14), MotorPulse_ISR, RISING);

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
	//xSemaphore = xSemaphoreCreateMutex();
	vSemaphoreCreateBinary(xSemaphore);

	#ifdef TASKCHECKSPC
	xTaskCreate(
	TaskCheckSPC
	,  (const portCHAR *)"CheckSPC"   // A name just for humans
	,  2000  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,   4  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  NULL );
	#endif
	
	#ifdef TASKCHECKSERIALCOMMANDS
	xTaskCreate(
	TaskCheckSerialCommands
	,  (const portCHAR *)"CheckSerialCommands"   // A name just for humans
	,  2000  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
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
	,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
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
	,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
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
	,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
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
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 1 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();
			int32_t waitTime = 30;
			
			
			if (HANDSHAKE)
			{
				spcProcessing.QueryFailed(350);

				if (spcProcessing.ISR_READY() && !spcProcessing.HasNewData)
				{
					spcProcessing.StartQuery();//enable interrupts and start the bit gathering from spc
				}
				else
				{
					spcProcessing.RunSPCDataLoop();
					spcProcessing.HasNewData = false;
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
		if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 5 ) == pdTRUE )
		{
			TickType_t xLastWakeTime;
			xLastWakeTime = xTaskGetTickCount();

			serialProcessing.Poll();
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil( &xLastWakeTime, 20);
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
					//delay(20); //puller likes to take its sweet time responding, need at least 10ms to sync back...
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

					case 2:
					{
						command.command = "FilamentDiameter";
						command.hardwareType = hardwareType.traverse;
						char value[MAX_CMD_LENGTH] = {0};
						itoa((spcProcessing.GetDiameter()->floatDiameterWithDecimal * 1000), value, 10);
						command.value = value;
						//command.value = (spcProcessing.GetDiameter()->floatDiameterWithDecimal * 1000);
						traverseDataCounter++;
						break;
					}
					

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
			vTaskDelayUntil( &xLastWakeTime, 210);
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
					//delay(10);
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
						//delay(20);
						fullUpdateCounter++;
						break;

						case 1:
						command.command = "PullerRestartReason";
						command.hardwareType = hardwareType.puller;
						serialProcessing.SendDataToDevice(&command);
						//delay(20);
						fullUpdateCounter++;
						break;

						case 2:
						command.command = "InnerOffset";
						command.hardwareType = hardwareType.traverse;
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 3:
						command.command = "SpoolWidth";
						command.hardwareType = hardwareType.traverse;
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 4:
						command.command = "RunMode";
						command.hardwareType = hardwareType.traverse;
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 5:
						command.command = "StartPosition";
						command.hardwareType = hardwareType.traverse;
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;
						
						case 6:
						command.command = "NominalDiameter";
						command.hardwareType = hardwareType.internal;
						command.value = nvm_operations.GetNominalDiameter();
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 7:
						command.command = "UpperLimit";
						command.hardwareType = hardwareType.internal;
						command.value = nvm_operations.GetUpperLimit();
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;
						
						case 8:
						command.command = "LowerLimit";
						command.hardwareType = hardwareType.internal;
						command.value = nvm_operations.GetLowerLimit();
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;
						
						case 9:
						command.command = "SpecificGravity";
						command.hardwareType = hardwareType.internal;
						command.value = nvm_operations.GetSpecificGravity();
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 10:
						command.command = "SpoolWeightLimit";
						command.hardwareType = hardwareType.internal;
						command.value = nvm_operations.GetSpoolWeightLimit();
						serialProcessing.SendDataToDevice(&command);
						fullUpdateCounter++;
						break;

						case 11:
						command.command = "MotherboardRestartReason";
						command.hardwareType = hardwareType.internal;
						command.value = restartReason;
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
					CONVERT_NUMBER_TO_STRING(INT_FORMAT, serialProcessing.FilamentCapture == true, value);
					SerialCommand command = {0};
					command.command = "FilamentCapture";
					command.hardwareType = hardwareType.traverse;
					command.value = value;

					if (!serialProcessing.FullUpdateRequested && command.hardwareType != NULL)
					{
						serialProcessing.SendDataToDevice(&command);
						
						serialProcessing.SendDataToDevice(&command); // do it twice
						
						command.hardwareType = hardwareType.puller;
						
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
				float specificGravity = atof(nvm_operations.GetSpecificGravity());
				static float currentFilamentWeight = 0;
				static float previousWeight = 0;
				static TickType_t previousTime = 0;
				static int i = 0;
				//SPOOLWEIGHT
				if (FILAMENTLENGTH != previousLength && FILAMENTLENGTH > previousLength || !previousCaptureState )
				{
					if (spoolWeight < 0) {spoolWeight = 0.0;}
					spoolWeight = spoolWeight + (HALF_PI / 2.0) * pow(FILAMENTDIAMETER, 2) * (FILAMENTLENGTH - previousLength) * specificGravity;
					if (i == 0)
					{
						currentFilamentWeight = spoolWeight;
					}
					
					previousLength = FILAMENTLENGTH;
					
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
					
					

					++i;
					if (i >= 5 || !previousCaptureState)
					{
						int64_t timeDelta = xLastWakeTime - previousTime;//filamentTimes[i - 1]- filamentTimes[0];
						float weightDelta = currentFilamentWeight - previousWeight;
						float rate = (60.0 * (60.0 * ((weightDelta) / (float)(timeDelta / 1000.0)))) / 1000.0;
						i = 0;
						
						if (rate < 0.0) {rate = 0.0;}

						value[MAX_CMD_LENGTH] = {0};
						CONVERT_NUMBER_TO_STRING("%0.3f", rate, value);
						command = {0};
						command.command = "OutputRate";
						command.hardwareType = hardwareType.internal;
						command.value = value;
						if (!serialProcessing.FullUpdateRequested && command.hardwareType != NULL)
						{
							serialProcessing.SendDataToDevice(&command);
						}
						previousTime = xLastWakeTime;
						previousWeight = currentFilamentWeight;
					}

					previousCaptureState = serialProcessing.FilamentCapture;


				}
				
			}
			else
			{
				spoolWeight = 0;
				feedRate = 0;
				previousLength = 0;
				FILAMENTLENGTH = 0;
			}
			previousCaptureState = serialProcessing.FilamentCapture;
			
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
				else
				{
					SerialCommand command;
					command.hardwareType = hardwareType.internal;
					command.command = "KeepAlive";
					command.value = "Connected";

					serialProcessing.SendDataToDevice(&command);
				}
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





