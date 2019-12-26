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
		
		
		
		//computer_bytes_received = port->findUntil()
		//port->readBytes(computerdata, MAX_CMD_LENGTH);
		
		for (int i = 0; i < MAX_CMD_LENGTH; i++){
			if (computerdata[i] > 0) {
				if (computerdata[i] == 10 || computerdata[i] == 13){
					computerdata[i] = 0;
					break;
				}
				
				computer_bytes_received++;
			}
		}
		//computer_bytes_received = port->readBytesUntil(10, computerdata, MAX_CMD_LENGTH); //We read the data sent from the serial monitor(pc/mac/other) until we see a <CR>. We also count how many characters have been received
		//computerdata[computer_bytes_received] = 0; //We add a 0 to the spot in the array just after the last character we received.. This will stop us from transmitting incorrect data that may have been left in the buffer
		
		//while(port->available()){
		//port->read();
		//} //flush buffer
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

	if (i != 0) {
		CommandParse(&sCommand, computerdata);
		
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
	

	return 1;
}



unsigned int SerialProcessing::CommandParse(SerialCommand *sCommand, char str[MAX_CMD_LENGTH])
{
	str_replace(str, "\r", "");
	str_replace(str, "\n", "");

	char *hardwareID = strtok(str, DELIMITER); //hardware ID
	char *cmd = strtok(NULL, DELIMITER);
	char *arguments = strtok(NULL, DELIMITER);

	//if (cmd != '\0' || arguments != '\0'){
	//SerialNative.print(hardwareID);
	//SerialNative.print(";");
	//SerialNative.print(cmd);
	//SerialNative.print(";");
	//SerialNative.println(arguments);
	//}


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
	if ( strcmp(sCommand->command, "SpecificGravity") == 0)
	{
		nvm_operations.SetSpecificGravity(atof(sCommand->value));
	}
	if ( strcmp(sCommand->command, "SpoolWeightLimit") == 0)
	{
		nvm_operations.SetSpoolWeightLimit(atoi(sCommand->value));
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
	if (sCommand->value == NULL)
	{
		sprintf(outputBuffer, OUTPUT_STRING_DS, sCommand->hardwareType, sCommand->command, sCommand->value);
		return;
	}

	sprintf(outputBuffer, OUTPUT_STRING_DSS, sCommand->hardwareType, sCommand->command, sCommand->value);
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

