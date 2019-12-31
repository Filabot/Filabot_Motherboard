#include "SerialProcessing.h"
#include <Arduino.h>
#include "board.h"
#include "hardwareTypes.h"
#include <stdio.h>
#include "DataConversions.h"
#include "Structs.h"
#include "SerialPortExpander.h"
#include "SerialNative.h"
#include "NVM_Operations.h"



template<size_t SIZE, class T> inline size_t array_size(T (&arr)[SIZE]);

SerialProcessing *SerialProcessing::firstInstance;
SerialPortExpander _serialPortExpander;


//bool SIMULATIONMODE; //sets default value for simulation


SerialProcessing::SerialProcessing(){
	if(!firstInstance){
		firstInstance = this;
	}
}

void SerialProcessing::init(){
	_serialPortExpander.init();
	SerialNative.setTimeout(50);

}

void SerialProcessing::Poll(void)
{

	//if (!commandActive)
	//{
	CheckSerial(&SerialNative, 0); //check for new data coming from PC
	//}
	//if (!commandActive)
	//{
	CheckSerial(&Serial1, 1); //check for new data coming from expander
	//}
	

}

unsigned int SerialProcessing::CheckSerial(_SerialNative *port, int portNumber)
{
	//commandActive = true;
	char computerdata[MAX_CMD_LENGTH] = {0};
	byte computer_bytes_received = 0;

	SerialCommand sCommand;
	sCommand.command = NULL;
	sCommand.hardwareType = NULL;
	sCommand.value = NULL;
	port->setTimeout(10);

	int pos = 0;
	long start = millis();

	if (port->available() > 0)
	{
		port->readBlock(computerdata, MAX_CMD_LENGTH);
		
		for (int i = 0; i < MAX_CMD_LENGTH; i++){
			if (computerdata[i] > 0) {
				if (computerdata[i] == 10 || computerdata[i] == 13){
					computerdata[i] = 0;
					break;
				}
				
				computer_bytes_received++;
			}
		}
	}

	if (computer_bytes_received != 0) {             //If computer_bytes_received does not equal zero
		
		CommandParse(&sCommand, computerdata);
		
		computer_bytes_received = 0;                  //Reset the var computer_bytes_received to equal 0
		
		

		if (portNumber == 0) //if data comes from USB/PC
		{
			ProcessDataFromPC(&sCommand);
		}
		else //if data comes from expander
		{
			SendToPC(&sCommand);
			SendScreenData(&sCommand);
		}
		
	}
	//commandActive = false;

	return 1;
}

unsigned int SerialProcessing::CheckSerial(HardwareSerial *port, int portNumber) //check expander ports
{
	char computerdata[MAX_CMD_LENGTH] = {0};
	byte computer_bytes_received = 0;

	SerialCommand sCommand = {0};
	int i = 0;

	if (port->available() > 0)
	{
		computerdata[MAX_CMD_LENGTH] = {0};
		
		while(port->available() > 0)
		{
			computerdata[i++] = port->read();
			if (i > MAX_CMD_LENGTH){break;}
		}
		
	}

	if (i > 1) 
	{
		CommandParse(&sCommand, computerdata);
		
		if (sCommand.tokenCount == 4)
		{
			computer_bytes_received = 0;                  //Reset the var computer_bytes_received to equal 0
		
			if (portNumber == 0) //if data comes from USB/PC
			{
				ProcessDataFromPC(&sCommand);
			}
			else //if data comes from expander
			{
				if (sCommand.hardwareType == hardwareType.puller)
				{
					int i = 0;
					i = 1;
				}
				SendToPC(&sCommand);
			}
		}
	}
	

	return 1;
}



unsigned int SerialProcessing::CommandParse(SerialCommand *sCommand, char str[MAX_CMD_LENGTH])
{
	str_replace(str, "\r", "");
	str_replace(str, "\n", "");

	char str2[MAX_CMD_LENGTH] = {0};
	strcpy(str2, str);

	int16_t tokenCount = 0;
	char *ptr = str;

	while((ptr = strchr(ptr, DELIMITER[0])) != NULL)
	{
		tokenCount++;
		ptr++;
	}


	char *hardwareID = strtoke(str, DELIMITER); //hardware ID
	char *cmd = strtoke(NULL, DELIMITER);
	char *arguments = strtoke(NULL, DELIMITER);
	char *checksum = strtoke(NULL, DELIMITER);

	if (!checksumPassed(checksum, str2))
	{
		SerialCommand sCommand;
		sCommand.hardwareType = hardwareType.internal;
		sCommand.command = "Checksum Failure";
		sCommand.value = NULL;
		SendDataToDevice(&sCommand);
		return 0;
	}

	for (int i=0; hardwareID[i]!= '\0'; i++)
	{
		//Serial.println(hardwareType[i]);
		if (!isdigit(hardwareID[i]) != 0)
		{
			SerialNative.println("100;Invalid Hardware ID, number is not a digit");
			return 0;
		}
	}

	
	sCommand->hardwareType = atoi(hardwareID);
	sCommand->command = cmd;
	sCommand->value = arguments;
	sCommand->tokenCount = tokenCount;
	

	//char output[MAX_CMD_LENGTH] = {0};
	//BuildSerialOutput(sCommand, output);
	//
	//SerialNative.println(output);

	
	return 1;
}

unsigned int SerialProcessing::ProcessDataFromPC(SerialCommand *sCommand)
{
	//int cmp = strcmp(sCommand->command, "GetFullUpdate");

	
	if(sCommand->hardwareType == hardwareType.internal)
	{
		this->CheckInteralCommands(sCommand);
	}

	this->SendDataToDevice(sCommand);

	return 1;
}

unsigned int SerialProcessing::SendDataToDevice(SerialCommand *sCommand)
{

	if((sCommand->hardwareType > hardwareType.indicator) && (sCommand->hardwareType < hardwareType.screen))
	{
		//serialPortExpander.channel
		_serialPortExpander.ProcessSerialExpander(sCommand);
		delay(20);
	}
	if (sCommand->hardwareType == hardwareType.internal)
	{
		SendToPC(sCommand);
	}
	return 1;
}

unsigned int SerialProcessing::SendToPC(SerialCommand *sCommand)
{
	if (strcmp(sCommand->command, "FilamentLength") == 0)
	{
		if (sCommand->value != "" || sCommand->value != NULL)
		{
			FILAMENTLENGTH = atof(sCommand->value);
		}
		
	}

	char serialOutputBuffer[MAX_CMD_LENGTH] = {0};
	BuildSerialOutput(sCommand, serialOutputBuffer);
	SerialNative.println(serialOutputBuffer);

	return 1;

}

unsigned int SerialProcessing::SendScreenData(SerialCommand *sCommand)
{

	if (sCommand->hardwareType == hardwareType.traverse || sCommand->hardwareType == hardwareType.screen)
	{
		SerialCommand _sCommand = {0};
		_sCommand.hardwareType = hardwareType.screen;
		_sCommand.value = sCommand->value;
		_sCommand.command = sCommand->command;
		char serialOutputBuffer[MAX_CMD_LENGTH] = {0};
		BuildSerialOutput(&_sCommand, serialOutputBuffer);
		Serial3.println(serialOutputBuffer);
		//SerialUSB.println(serialOutputBuffer); //Serial print is broken using long values, use char instead

	}
	return 1;
}

void SerialProcessing::CheckInteralCommands(SerialCommand *sCommand)
{
	if ( strcmp(sCommand->command, "NominalDiameter") == 0)
	{
		nvm_operations.SetNominalDiameter(atof(sCommand->value), true);
	}
	if ( strcmp(sCommand->command, "UpperLimit") == 0)
	{
		nvm_operations.SetUpperLimit(atof(sCommand->value), true);
	}
	if ( strcmp(sCommand->command, "LowerLimit") == 0)
	{
		nvm_operations.SetLowerLimit(atof(sCommand->value), true);
	}
	
	if ( strcmp(sCommand->command, "SpecificGravity") == 0)
	{
		nvm_operations.SetSpecificGravity(atof(sCommand->value), true);
	}
	if ( strcmp(sCommand->command, "SpoolWeightLimit") == 0)
	{
		nvm_operations.SetSpoolWeightLimit(atoi(sCommand->value), true);
	}
	if ( strcmp(sCommand->command, "GetFullUpdate") == 0)
	{
		FullUpdateRequested = true;
	}
	if ( strcmp(sCommand->command, "FilamentCapture") == 0)
	{
		ProcessFilamentCaptureState(sCommand);
	}
	if (strcmp(sCommand->command, "Handshake") == 0)
	{
		HANDSHAKE = true;
	}
	if (strcmp(sCommand->command, "IsInSimulationMode") == 0)
	{
		SIMULATIONACTIVE = strcmp(sCommand->value, "true") == 0;
	}
	

}

void SerialProcessing::ProcessFilamentCaptureState(SerialCommand *sCommand)
{
	if (sCommand->value != NULL)
	{
		static bool previousCaptureState = false;

		FilamentCapture = strcmp(sCommand->value, "1") == 0 ? true : false;

		if (HANDSHAKE)
		{
			if (previousCaptureState != FilamentCapture )
			{
				
				char value[MAX_CMD_LENGTH] = {0};
				CONVERT_NUMBER_TO_STRING(STRING_FORMAT, FilamentCapture == true ? "1" : "0", value);
				SerialCommand command = {0};
				command.command = "FilamentCapture";
				command.hardwareType = hardwareType.traverse;
				command.value = value;

				if (!FullUpdateRequested && command.hardwareType != NULL)
				{
					SendDataToDevice(&command);
					command.hardwareType = hardwareType.puller;
					SendDataToDevice(&command);
				}
				previousCaptureState = FilamentCapture;
			}
		}
	}
	else
	{
		SerialCommand command = {0};
		command.command = sCommand->command;
		command.hardwareType = sCommand->hardwareType;
		char value[MAX_CMD_LENGTH] = {0};
		CONVERT_NUMBER_TO_STRING(INT_FORMAT, FilamentCapture == true ? 1 : 0, value);
		command.value = value;
		SendToPC(&command);
	}

	
}



void SerialProcessing::str_replace(char src[MAX_CMD_LENGTH], char *oldchars, char *newchars) { // utility string function

	char *p = strstr(src, oldchars);
	char buf[MAX_CMD_LENGTH] = {"\0"};
	do {
		if (p) {
			memset(buf, '\0', strlen(buf));
			if (src == p) {
				strcpy(buf, newchars);
				strcat(buf, p + strlen(oldchars));
				} else {
				strncpy(buf, src, strlen(src) - strlen(p));
				strcat(buf, newchars);
				strcat(buf, p + strlen(oldchars));
			}
			memset(src, '\0', strlen(src));
			strcpy(src, buf);
		}
	} while (p && (p = strstr(src, oldchars)));
}

void BuildSerialOutput(SerialCommand *sCommand, char *outputBuffer)
{
	char buf[MAX_CMD_LENGTH] = {0};
	sCommand->checksum = 0;

	if (sCommand->value == NULL)
		sCommand->value = "";

	sprintf(buf, OUTPUT_STRING_DSS, sCommand->hardwareType, sCommand->command, sCommand->value);

	for (int i = 0; i < MAX_CMD_LENGTH; i++)
	{
		sCommand->checksum = sCommand->checksum ^ buf[i]; //checksum, then add to command
	}
	
	sprintf(outputBuffer, OUTPUT_STRING_DSSS, sCommand->hardwareType, sCommand->command, sCommand->value, sCommand->checksum);


}

char* strtoke(char *str, const char *delim)
{
	static char *start = NULL; /* stores string str for consecutive calls */
	char *token = NULL; /* found token */
	/* assign new start in case */
	if (str) start = str;
	/* check whether text to parse left */
	if (!start) return NULL;
	/* remember current start as found token */
	token = start;
	/* find next occurrence of delim */
	start = strpbrk(start, delim);
	/* replace delim with terminator and move start to follower */
	if (start) *start++ = '\0';
	/* done */
	return token;
}

bool checksumPassed (char *serialChecksum, char *serialStringToCheck)
{

	char str2[MAX_CMD_LENGTH] = {0};
	char str3[MAX_CMD_LENGTH] = {0};
	strcpy(str2, serialStringToCheck);


	if (serialChecksum == NULL || serialChecksum == "" || serialChecksum[0] == '\0')
	return false;

	int32_t tCount = 0;
	int32_t tokenPosition = 0;
	byte checksumValue = 0;
	byte strPtr = 0;
	for (int i = 0; i < MAX_CMD_LENGTH; ++i)
	{
		if (tCount != 3)
		{
			str3[strPtr++] = str2[i];
		}
		else
		{
			if (str2[i] == ';')
			{
				tCount++;
			}
		}
		if (str2[i] == ';' && tCount < 3)
		{
			tokenPosition = i;
			tCount++;
		}
	}
	char *test = str3;
	for (int i = 0; i < MAX_CMD_LENGTH; ++i)
	{
		checksumValue = checksumValue ^ str3[i];
	}
	if (checksumValue != atoi(serialChecksum))
	return false;

	return true;

}

void PrintRandomRPMData()
{
	long rpm = random(10, 15);

	//SerialNative.print(hardwareType.puller);
	//SerialNative.print(";getrpm = ");
	//SerialNative.print(rpm);
	//SerialNative.println(";");
}

// default destructor
SerialProcessing::~SerialProcessing()
{
} //~SerialProcessing

