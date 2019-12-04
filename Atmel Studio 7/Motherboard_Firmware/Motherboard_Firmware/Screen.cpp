/*
* Screen.cpp
*
* Created: 4/3/2019 11:10:52 AM
* Author: Anthony
*/


#include "Screen.h"
#include "Structs.h"
#include "SerialProcessing.h"
#include "DataConversions.h"
#include "Error.h"

Screen *Screen::firstInstance;

bool previousHadError = false;

// default constructor
Screen::Screen()
{
	if (!firstInstance)
	{
		firstInstance = this;
	}
} //Screen

void Screen::init()
{
	
	
}

void Screen::UpdateDiameter(SpcDiameter *spcDiameter)
{
	SerialProcessing serialProcessing;
	SerialCommand serialCommand;

	serialCommand.command = "SPCDiameter";
	serialCommand.hardwareType = hardwareType.screen;
	serialCommand.value = spcDiameter->charDiameterNoDecimal;

	serialProcessing.SendScreenData(&serialCommand);

	serialCommand.command = "SPCDiameterDecimalLocation";
	serialCommand.hardwareType = hardwareType.screen;
	char buf[5] = {0};
	CONVERT_NUMBER_TO_STRING(INT_FORMAT, spcDiameter->decimalPointLocation, buf);
	serialCommand.value = buf;

	serialProcessing.SendScreenData(&serialCommand);
}

void Screen::UpdateSpool(Spool *spool)
{
	SerialProcessing serialProcessing;
	SerialCommand serialCommand;

	serialCommand.command = "SpoolRPM";
	serialCommand.hardwareType = hardwareType.screen;
	serialCommand.value = spool->RPM;

	serialProcessing.SendScreenData(&serialCommand);

}

void Screen::UpdatePuller(Puller *puller)
{
	SerialProcessing serialProcessing;
	SerialCommand serialCommand;

	serialCommand.command = "PullerRPM";
	serialCommand.hardwareType = hardwareType.screen;
	serialCommand.value = puller->RPM;

	serialProcessing.SendScreenData(&serialCommand);

}

void Screen::SendError(Error *error)
{
	SerialProcessing serialProcessing;
	SerialCommand serialCommand;

	

	serialCommand.command = "SPCDiameterError"; //this sends it to the "diameter slot" on the screen
	serialCommand.hardwareType = hardwareType.screen;
	//serialCommand.value = 
	//itoa((int)error->errorCode, serialCommand.value, 10);
	
	char value[MAX_CMD_LENGTH] = {0};
	itoa((int)error->errorCode, value, 10);
	serialCommand.value = value;


	serialProcessing.SendScreenData(&serialCommand);
}




void ConcantenateChars(char *buffer_A, char *buffer_B, char *outputBuffer)
{
	sprintf(outputBuffer, SCREENFORMAT, buffer_A, buffer_B);
}



void Screen::DisplayError(void)
{
	//for (int i = 0; i < 10; i++)
	//{
	//errors[i] = eError[i];
	//}

	//errors = *eError;
}

void Screen::ClearError(void)
{
	//error.errorLevel = 0;

}






// default destructor
Screen::~Screen()
{
} //~Screen
