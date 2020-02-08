/*
* spcProcessing.cpp
*
* Created: 12/20/2018 4:57:59 PM
* Author: Anthony
*/


#include "SpcProcessing.h"
#include <Arduino.h>
#include "hardwareTypes.h"
#include "DataConversions.h"
#include "SerialProcessing.h"
#include "Error.h"
#include "Structs.h"
#include "FreeRTOS_ARM.h"
#include "SAM3Timer.h"


SpcProcessing *SpcProcessing::firstinstance;
SpcProcessing *thisInstance;

SerialProcessing serialProcessing1;


void ISR_SPC();
bool dataInvalid = false;

// default constructor main entry point
SpcProcessing::SpcProcessing()
{
	if(!firstinstance)
	{
		firstinstance = this;
		thisInstance = this;
	}

	
} //spcProcessing

volatile int numberErrors;
long debugTime = 0;
volatile uint32_t isrTime = 0;

volatile bool SPC_ISR_LOCK = false;
volatile char rawSPC_ISR[53] = {0};
volatile int ISR_LOOP_COUNTER = 0;
volatile int MAIN_LOOP_COUNTER = 0;
volatile char rawSPC[53] = {0};
TickType_t previousQuery = 0;
volatile bool tc6TimerRunning = false;


void SpcProcessing::init(void)
{
	pinMode(INDICATOR_REQ, OUTPUT);
	pinMode(INDICATOR_CLK, INPUT_PULLUP);
	pinMode(INDICATOR_DAT, INPUT_PULLUP);

	SAM3Timer::startTimer(TC2, 0, TC6_IRQn, 6000); //TC1 channel 0, the IRQ for that channel and the desired frequency

	//attachInterrupt(digitalPinToInterrupt(INDICATOR_CLK), ISR_SPC, FALLING);
	
	StartQuery();
}

void TC6_Handler()
{
	TC_GetStatus(TC2, 0);
	bool clockPulse = GPIO_READ(INDICATOR_CLK);


	if (SPC_ISR_LOCK && tc6TimerRunning)
	{
		static bool previousClockPulse = 1;

		if (previousClockPulse && !clockPulse) //catch falling edge
		{
			bool dat = GPIO_READ(INDICATOR_DAT);
			if (dat)
			{

				rawSPC_ISR[ISR_LOOP_COUNTER++] = 49;
			}
			else
			{
				rawSPC_ISR[ISR_LOOP_COUNTER++] = 48;
			}
			

			if (ISR_LOOP_COUNTER >= 52) //there can only be 52 bits to the spc data
			{
				BaseType_t xHigherPriorityTaskWoken;
				xHigherPriorityTaskWoken = pdFALSE;
				thisInstance->StopQuery();
				xQueueSendFromISR( xQueue, ( void *)&rawSPC_ISR, &xHigherPriorityTaskWoken );
				SPC_ISR_LOCK = false; //unlock ISR to synchronize spc data loop
				//thisInstance->RunSPCDataLoop();
				thisInstance->HasNewData = true;
				
				if( xHigherPriorityTaskWoken )
				{
					/* Actual macro used here is port specific. */
					portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
				}
			}
			
		}
		previousClockPulse = clockPulse;
	}
}

void ISR_SPC()
{
	if (SPC_ISR_LOCK)
	{
		if (ISR_LOOP_COUNTER == 0) {isrTime=xTaskGetTickCount();}

		
		int32_t booleanTrueSum = 0;
		
		for (int i = 0; i < 11; ++i)
		{
			bool pinValue = GPIO_READ(INDICATOR_DAT);
			if (pinValue)
			{
				booleanTrueSum++;
			}
		}

		if (booleanTrueSum >= 5)
		{
			rawSPC_ISR[ISR_LOOP_COUNTER++] = 49;
		}
		else
		{
			rawSPC_ISR[ISR_LOOP_COUNTER++] = 48;
		}
		
		if (ISR_LOOP_COUNTER >= 52) //there can only be 52 bits to the spc data
		{
			
			if (thisInstance)
			{
				BaseType_t xHigherPriorityTaskWoken;
				xHigherPriorityTaskWoken = pdFALSE;
				thisInstance->StopQuery();
				xQueueSendFromISR( xQueue, ( void *)&rawSPC_ISR, &xHigherPriorityTaskWoken );
				SPC_ISR_LOCK = false; //unlock ISR to synchronize spc data loop
				//thisInstance->RunSPCDataLoop();
				thisInstance->HasNewData = true;
				
				if( xHigherPriorityTaskWoken )
				{
					/* Actual macro used here is port specific. */
					portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
				}
			}
			isrTime = xTaskGetTickCount() - isrTime;
		}
	}
}

void SpcProcessing::RunSPCDataLoop(void)
{
	BaseType_t xTaskWokenByReceive = pdFALSE;
	if (!SPC_ISR_LOCK ) //check for locked ISR
	{
		//Serial.println(rawSPC);
		

		while (xQueueReceiveFromISR(xQueue, (void *) &rawSPC, &xTaskWokenByReceive));
		{

			for (int i = 0; i < 52; ++i)
			{
				if (rawSPC_ISR[i] == 0) //no positions can be 0's
				{
					return;
				}
			}
			if (dataInvalid)
			{
				dataInvalid = false;
				return;
			}
			

			if (rawSPC[0] == 0){ //if position 0 in array equals null then skip
				//Serial.println("RESETTING FROM NO DATA");
				//digitalWrite(INDICATOR_REQ, HIGH);
				//PORTC |= digitalPinToBitMask(INDICATOR_REQ); //set req high to restart ISR
				return;
			}

			

			bool dataStreamValid = false; //set dataStreamValid false since this is the start of the verification process
			for (unsigned int i = 0; i < 12; i++) //first 12 indices should be 1's (49), if not then the data isn't valid
			{
				if (rawSPC[i] == 48) // || rawSPC_ISR[16] == 49) //48 is 0 (zero) in ascii 0-12 cannot be 0, and 13 cannot be 1
				{
					dataStreamValid = false;
					
					SerialCommand sError;
					sError.hardwareType = hardwareType.indicator;
					sError.command = "DiameterError";
					sError.value = "SPC Datastream Validation Error";
					
					//eError.hardwareType = hardwareType.indicator;
					//eError.errorLevel = errorLevel.datastream_validation_failed;
					//eError.errorCode = errorCode.spc_data_error;
					//AddError(&eError);

					char sErrorOutput [MAX_CMD_LENGTH] = {0};
					BuildSerialOutput(&sError, sErrorOutput);
					SerialNative.println(sErrorOutput);
					
					ISR_LOOP_COUNTER = 0;
					return;

				}
				else
				{
					dataStreamValid = true;
				}
			}

			if (dataStreamValid)
			{
				byte bytes[13] = {0};
				for (int i = 0; i < 13; i++)
				{
					int idx = (i*4) + 4;
					int bitPointer = 0;
					for (int j = i * 4; j < idx ; j++)
					{
						bitWrite(bytes[i], bitPointer, rawSPC[j] == 49); //49 ascii for 1 //grab nibbles from rawSPC
						bitPointer++;
					}
				}
				if (bytes[11] > 4)
				{

					dataStreamValid = false; //invalid data
					return;
				}
				
				float preDecimalNumber = 0.0;
				char buf[7] = {0};
				
				for(int i=0;i<6;i++){ //grab array positions 5-10 for diameter numbers
					
					buf[i]=bytes[i+5]+'0';
					
					buf[6]=0;
					
					preDecimalNumber=atol(buf); //assembled measurement, no decimal place added
				}
				
				int decimalPointLocation = bytes[11];
				
				SPCDiameter = preDecimalNumber / (pow(10, decimalPointLocation)); //add decimal to number
				
				spcDiameter.decimalPointLocation = decimalPointLocation;
				spcDiameter.intDiameterNoDecimal = preDecimalNumber;
				spcDiameter.floatDiameterWithDecimal = SPCDiameter;
				itoa(preDecimalNumber, spcDiameter.charDiameterNoDecimal, 10);
				
				
				char decimalNumber[20] = {0};
				CONVERT_FLOAT_TO_STRING(SPCDiameter, decimalNumber);
				CONVERT_FLOAT_TO_STRING(SPCDiameter, spcDiameter.charDiameterWithDecimal);

				static int numberOfZeros = 0;
				if (spcDiameter.charDiameterWithDecimal[2] == 48 && spcDiameter.charDiameterWithDecimal[0] == 48 )
				{
					numberOfZeros++;
				}
				else
				{
					numberOfZeros = 0;
				}
				if (numberOfZeros >= 2)
				{
					numberOfZeros = 0;
				}
				if (numberOfZeros == 1)
				{
					previousQuery = xTaskGetTickCount();
					return;
				}

				SerialCommand _serialCommand;
				_serialCommand.hardwareType = hardwareType.internal;
				_serialCommand.command = "Diameter";
				_serialCommand.value = spcDiameter.charDiameterWithDecimal;
				char output[MAX_CMD_LENGTH] = {0};
				
				FILAMENTDIAMETER = spcDiameter.floatDiameterWithDecimal;
				serialProcessing1.SendDataToDevice(&_serialCommand);

				_serialCommand.hardwareType = hardwareType.internal;
				_serialCommand.command = "testTime";
				itoa(isrTime, _serialCommand.value, 10);
				if (isrTime > 100000)
				serialProcessing1.SendDataToDevice(&_serialCommand);

				for (int i = 0; i < 52; i++) //clean up array for next go around, cannot use memset since rawSPC is volatile
				{
					rawSPC[i] = 0;
				}
				
				MAIN_LOOP_COUNTER = 0;
				ISR_LOOP_COUNTER = 0;

			}
			
			//HasNewData = true;
			if( xTaskWokenByReceive != pdFALSE )
			{
				/* We should switch context so the ISR returns to a different task.
				NOTE:  How this is done depends on the port you are using.  Check
				the documentation and examples for your port. */
				taskYIELD ();
			}
		}
		previousQuery = xTaskGetTickCount();
	}
}

SpcDiameter *SpcProcessing::GetDiameter(void){
	return &spcDiameter;
}

bool SpcProcessing::QueryFailed(int32_t waitTime)
{
	
	static bool firstRun = true;

	if (firstRun)
	{
		previousQuery = xTaskGetTickCount();
		firstRun = false;
	}
	if (xTaskGetTickCount() >= previousQuery + (waitTime)) //100 milliseconds //if previous query didn't finish in time it is dead
	{
		SerialCommand command;
		command.hardwareType = hardwareType.indicator;
		command.command = "DiameterError";
		command.value = "SPC Failed to Acquire Data";

		char buffer[MAX_CMD_LENGTH] = {0};
		BuildSerialOutput(&command, buffer);

		SerialNative.println(buffer);

		previousQuery = xTaskGetTickCount();
		
		return true;
	}
	
	return false;
}

bool SpcProcessing::HasError(void)
{
	return HasErrors();
}

Error *SpcProcessing::GetError(void)
{
	return &eError;
}

volatile bool SpcProcessing::ISR_READY(void)
{
	return !SPC_ISR_LOCK;
}

void SpcProcessing::StartQuery(void)
{
	SPC_ISR_LOCK = true; //lock ISR so main program loop doesn't interrupt
	//attachInterrupt(digitalPinToInterrupt(INDICATOR_CLK), ISR_SPC, FALLING);
	tc6TimerRunning = true;

	ISR_LOOP_COUNTER = 0;
	MAIN_LOOP_COUNTER = 0;
	
	HasNewData = false;
	digitalWrite(INDICATOR_REQ, LOW); //sets output high, inverted low on the SPC
	
}

void SpcProcessing::StopQuery(void)
{
	//detachInterrupt(digitalPinToInterrupt(INDICATOR_CLK)); //kill interrupt
	tc6TimerRunning = false;
	
	digitalWrite(INDICATOR_REQ, HIGH);
	
}


// default destructor
SpcProcessing::~SpcProcessing()
{
} //~spcProcessing


