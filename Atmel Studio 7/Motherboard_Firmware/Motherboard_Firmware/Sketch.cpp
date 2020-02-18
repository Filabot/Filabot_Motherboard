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
#include "Pulling.h"
//#include "FreeRTOS_ARM.h"

// ***** INCLUDES ***** //

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
void TaskCheckSPC(void);
void TaskCheckSerialExpander(void);
void TaskCheckSerialCommands(void);
void TaskRunSimulation(void);
void TaskGetTraverseData (void);
void TaskCheckEncoder (void);
void TaskGetFullUpdate (void);
void TaskFilamentCapture (void);
void TaskCalculate (void);
void TaskHandshake (void);
//void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress );
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
int32_t KEEPALIVETIMER = 0;


// ***** CLASS DECLARATIONs **** //
SerialCommand sCommand;
SpcProcessing spcProcessing;
SerialProcessing serialProcessing;
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
NVM_Operations nvm_operations;

Encoder encoder(ENCODER_PINA, ENCODER_PINB);
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

uint32_t SpcTimer = 0;
uint32_t SerialTimer = 0;
uint32_t TraverseTimer = 0;
uint32_t HandshakeTimer = 0;
uint32_t FullUpdateTimer = 0;
uint32_t CalculateTimer = 0;
uint32_t FilamentCaptureTimer = 0;
uint32_t EncoderTimer = 0;


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

	
	if (serialProcessing.FilamentCapture)
	motorPulsesSent++;
	else
	motorPulsesSent = 0;
	
	
	state = !state;

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
	WDT_Disable(WDT);
	REG_SUPC_MR |= SUPC_MR_KEY(0xA5) | SUPC_MR_BODDIS_DISABLE;
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
	//attachInterrupt(digitalPinToInterrupt(14), MotorPulse_ISR, RISING);

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
}

void loop()
{
	uint32_t loopTime = millis();

	

	if (millis() - SerialTimer >= 20)
	{
		TaskCheckSerialCommands();
		SerialTimer = millis();
	}

	if (millis() - SpcTimer >= 30)
	{
		TaskCheckSPC();
		SpcTimer = millis();
	}

	if (millis() - TraverseTimer >= 200)
	{
		TaskGetTraverseData();
		TraverseTimer = millis();
	}

	if (millis() - HandshakeTimer >= 500)
	{
		TaskHandshake();
		HandshakeTimer = millis();
	}

	if (millis() - CalculateTimer >= 500)
	{
		TaskCalculate();
		CalculateTimer = millis();
	}

	if (millis() - FullUpdateTimer >= 100)
	{
		TaskGetFullUpdate();
		FullUpdateTimer = millis();
	}

	if (millis() - FilamentCaptureTimer >= 100)
	{
		TaskFilamentCapture();
		FilamentCaptureTimer = millis();
	}

	if (millis() - EncoderTimer >= 100)
	{
		TaskCheckEncoder();
		EncoderTimer = millis();
	}

	loopTime = millis() - loopTime;

}

void TaskCheckSPC()
{

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
}

void TaskCheckSerialCommands()  // This is a task.
{
	serialProcessing.Poll();
}


void TaskGetTraverseData()  // This is a task.
{
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

}

void TaskCheckEncoder()  // This is a task.
{

	int32_t encoderValue = encoder.read();
	if (previousEncoderValue != encoderValue)
	{
		float rpm = (previousEncoderValue - encoderValue) / 2.0;

		Pulling::IncDecWheelRPM(rpm);
		
		previousEncoderValue = encoderValue;

		SerialCommand command;
		command.hardwareType = hardwareType.internal;
		command.command = "velocity";

		char value[MAX_CMD_LENGTH] = {0};
		CONVERT_NUMBER_TO_STRING(FLOAT_FORMAT, Pulling::GetWheelRPM(), value);
		command.value = value;
		if (HANDSHAKE)
			serialProcessing.SendDataToDevice(&command);
	}
}

void TaskGetFullUpdate()  // This is a task.
{
	if (HANDSHAKE){
		if (serialProcessing.FullUpdateRequested)
		{
			SerialCommand command = {0};
			switch(fullUpdateCounter)
			{
				
				case 0:
				command.command = "InnerOffset";
				command.hardwareType = hardwareType.traverse;
				serialProcessing.SendDataToDevice(&command);
				fullUpdateCounter++;
				break;

				case 1:
				command.command = "SpoolWidth";
				command.hardwareType = hardwareType.traverse;
				serialProcessing.SendDataToDevice(&command);
				fullUpdateCounter++;
				break;

				case 2:
				command.command = "RunMode";
				command.hardwareType = hardwareType.traverse;
				serialProcessing.SendDataToDevice(&command);
				fullUpdateCounter++;
				break;

				case 3:
				command.command = "StartPosition";
				command.hardwareType = hardwareType.traverse;
				serialProcessing.SendDataToDevice(&command);
				fullUpdateCounter++;
				break;
				
				case 4:
				command.command = "NominalDiameter";
				command.hardwareType = hardwareType.internal;
				command.value = nvm_operations.GetNominalDiameter();
				serialProcessing.SendDataToDevice(&command);
				fullUpdateCounter++;
				break;

				case 5:
				command.command = "UpperLimit";
				command.hardwareType = hardwareType.internal;
				command.value = nvm_operations.GetUpperLimit();
				serialProcessing.SendDataToDevice(&command);
				fullUpdateCounter++;
				break;
				
				case 6:
				command.command = "LowerLimit";
				command.hardwareType = hardwareType.internal;
				command.value = nvm_operations.GetLowerLimit();
				serialProcessing.SendDataToDevice(&command);
				fullUpdateCounter++;
				break;
				
				case 7:
				command.command = "SpecificGravity";
				command.hardwareType = hardwareType.internal;
				command.value = nvm_operations.GetSpecificGravity();
				serialProcessing.SendDataToDevice(&command);
				fullUpdateCounter++;
				break;

				case 8:
				command.command = "SpoolWeightLimit";
				command.hardwareType = hardwareType.internal;
				command.value = nvm_operations.GetSpoolWeightLimit();
				serialProcessing.SendDataToDevice(&command);
				fullUpdateCounter++;
				break;

				case 9:
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
}

void TaskFilamentCapture()  // This is a task.
{
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
			}
			previousCaptureState = serialProcessing.FilamentCapture;
		}
	}
}

void TaskCalculate()  // This is a task.
{
	if (serialProcessing.FilamentCapture )
	{
		float specificGravity = atof(nvm_operations.GetSpecificGravity());
		static float currentFilamentWeight = 0;
		static float previousWeight = 0;
		static int32_t previousTime = 0;
		static int i = 0;
		
		FILAMENTLENGTH = (( motorPulsesSent / 1.857 ) * .03020) / 1000; //0.03180 is wheel circumference / steps

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

			value[MAX_CMD_LENGTH] = {0};
			CONVERT_NUMBER_TO_STRING("%0.3f", FILAMENTLENGTH, value);
			command = {0};
			command.command = "FilamentLength";
			command.hardwareType = hardwareType.internal;
			command.value = value;
			if (!serialProcessing.FullUpdateRequested && command.hardwareType != NULL)
			{
				serialProcessing.SendDataToDevice(&command);
			}

			++i;
			if (i >= 5 || !previousCaptureState)
			{
				int64_t timeDelta = millis() - previousTime;//filamentTimes[i - 1]- filamentTimes[0];
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

				previousTime = millis();
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
	


}

void TaskHandshake()  // This is a task.
{
	if (SerialNative.dtr())
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
	
}



//static void HardFault_Handler(void)
//{
    //asm volatile
    //(
        //"tst lr, #4 \n\t"
        //"ite eq \n\t"
        //"mrseq r0, msp \n\t"
        //"mrsne r0, psp \n\t"
        //"ldr r1, [r0, #24] \n\t"
        //"ldr r2, handler2_address_const \n\t"
        //"bx r2 \n\t"
        //"handler2_address_const: .word prvGetRegistersFromStack \n\t"
    //);
//}



//void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
//{
///* These are volatile to try and prevent the compiler/linker optimising them
//away as the variables never actually get used.  If the debugger won't show the
//values of the variables, make them global my moving their declaration outside
//of this function. */
//volatile uint32_t r0;
//volatile uint32_t r1;
//volatile uint32_t r2;
//volatile uint32_t r3;
//volatile uint32_t r12;
//volatile uint32_t lr; /* Link register. */
//volatile uint32_t pc; /* Program counter. */
//volatile uint32_t psr;/* Program status register. */
//
    //r0 = pulFaultStackAddress[ 0 ];
    //r1 = pulFaultStackAddress[ 1 ];
    //r2 = pulFaultStackAddress[ 2 ];
    //r3 = pulFaultStackAddress[ 3 ];
//
    //r12 = pulFaultStackAddress[ 4 ];
    //lr = pulFaultStackAddress[ 5 ];
    //pc = pulFaultStackAddress[ 6 ];
    //psr = pulFaultStackAddress[ 7 ];
//
    ///* When the following line is hit, the variables contain the register values. */
    //for( ;; );
//}