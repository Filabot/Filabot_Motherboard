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


SpcProcessing *SpcProcessing::firstinstance;
SpcProcessing *thisInstance;

void ISR_SPC();

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


volatile bool SPC_ISR_LOCK = false;
volatile char rawSPC_ISR[53] = {0};
volatile int ISR_LOOP_COUNTER = 0;
volatile int MAIN_LOOP_COUNTER = 0;
volatile char rawSPC[53] = {0};
int previousQuery = 0;


void SpcProcessing::init(void)
{
	pinMode(INDICATOR_REQ, OUTPUT);
	pinMode(INDICATOR_CLK, INPUT_PULLUP);
	pinMode(INDICATOR_DAT, INPUT_PULLUP);

	
	//attachInterrupt(digitalPinToInterrupt(INDICATOR_CLK), ISR_SPC, FALLING);
	
	StartQuery();
}

void ISR_SPC()
{
	//delayMicroseconds(50);
	rawSPC_ISR[ISR_LOOP_COUNTER++] = digitalRead(INDICATOR_DAT) == 0 ? 48 : 49;
	
	if (ISR_LOOP_COUNTER >= 52) //there can only be 52 bits to the spc data
	{
		detachInterrupt(digitalPinToInterrupt(INDICATOR_CLK)); //dump the interrupt to stop anymore triggering
		
		for (int i = 0; i < 52; i++)
		{
			rawSPC[i] = rawSPC_ISR[i]; //copy volatile memory to non-volatile and clear the volatile to synchronize the main program loop
			rawSPC_ISR[i] = 0;
		}
		
		SPC_ISR_LOCK = false; //unlock ISR to synchronize spc data loop

		if (thisInstance)
		{
			thisInstance->RunSPCDataLoop();
			thisInstance->StopQuery();

		}
		
	}
}



void SpcProcessing::RunSPCDataLoop(void)
{
	
	
	if (IsInSimulationMode)
	{
		PrintRandomDiameterData();
		return;
	}

	if (!SPC_ISR_LOCK) //check for locked ISR
	{
		//Serial.println(rawSPC);

		if (rawSPC[0] == 0){ //if position 0 in array equals null then skip
			//Serial.println("RESETTING FROM NO DATA");
			//digitalWrite(INDICATOR_REQ, HIGH);
			//PORTC |= digitalPinToBitMask(INDICATOR_REQ); //set req high to restart ISR
			return;
		}

		

		bool dataStreamValid = false; //set dataStreamValid false since this is the start of the verification process
		for (unsigned int i = 0; i < 12; i++) //first 12 indices should be 1's (49), if not then the data isn't valid
		{
			if (rawSPC[i] == 48 || rawSPC[16] == 49) //48 is 0 (zero) in ascii 0-12 cannot be 0, and 13 cannot be 1
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
				SPCDiameter = 0.00;
				ISR_LOOP_COUNTER = 0;
				break;
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

			HasNewData = true;

			for (int i = 0; i < 52; i++) //clean up array for next go around, cannot use memset since rawSPC is volatile
			{
				rawSPC[i] = 0;
			}
			
			MAIN_LOOP_COUNTER = 0;
			ISR_LOOP_COUNTER = 0;
			previousQuery = millis();
		}

	}
	
}

char *SpcProcessing::GetSerialOutputBuffer(void)
{
	HasNewData = false;
	return serialOutputBuffer;
}

SpcDiameter *SpcProcessing::GetDiameter(void){

	return &spcDiameter;
	
}

int SpcProcessing::GetLoopCounts(void)
{
	MAIN_LOOP_COUNTER++;
	return MAIN_LOOP_COUNTER;
}

bool SpcProcessing::QueryFailed(int32_t waitTime)
{
	if (!HANDSHAKE)
		previousQuery = millis();

	if (millis() >= previousQuery + (waitTime)) //100 milliseconds //if previous query didn't finish in time it is dead
	{
		//StopQuery();
		//SerialNative.println("Query Error");
		//eError.hardwareType = hardwareType.indicator;
		//eError.errorLevel = errorLevel.device_disconnected;
		//eError.errorCode = errorCode.diameter_device_disconnected;
		//AddError(&eError);

		SerialCommand command;
		command.hardwareType = hardwareType.indicator;
		command.command = "DiameterError";
		command.value = "SPC Failed to Acquire Data";

		char buffer[MAX_CMD_LENGTH] = {0};
		BuildSerialOutput(&command, buffer);

		SerialNative.println(buffer);

		previousQuery = millis();
		
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

int SpcProcessing::PrintRandomDiameterData(void)
{
	char diameter[5];
	ltoa(random(17000, 18000), diameter, 10);
	
	SerialNative.print("0;111111111111111111110000");
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			//SerialNative.print(((byte)diameter[i] >> j) & 1);
		}
	}
	SerialNative.println("00100000");
	return 0;
}
bool SpcProcessing::ISR_READY(void)
{
	return !SPC_ISR_LOCK;
}

void SpcProcessing::StartQuery(void)
{
	SPC_ISR_LOCK = true; //lock ISR so main program loop doesn't interrupt
	attachInterrupt(digitalPinToInterrupt(INDICATOR_CLK), ISR_SPC, FALLING);

	delayMicroseconds(150); //this delay lengthens the inverted low pulse to the SPC stream. 2ms is the minimum time for the SPC to initiate the clock signal properly, maybe a timer is better?
	
	ISR_LOOP_COUNTER = 0;
	MAIN_LOOP_COUNTER = 0;
	
	HasNewData = false;
	digitalWrite(INDICATOR_REQ, LOW); //sets output high, inverted low on the SPC
	
}

void SpcProcessing::StopQuery(void)
{
	detachInterrupt(digitalPinToInterrupt(INDICATOR_CLK)); //kill interrupt
	
	digitalWrite(INDICATOR_REQ, HIGH);
	
}


// default destructor
SpcProcessing::~SpcProcessing()
{
} //~spcProcessing


