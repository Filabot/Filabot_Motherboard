/*
* SerialPortExpander.cpp
*
* Created: 12/27/2018 10:18:01 AM
* Author: Anthony
*/


#include "SerialPortExpander.h"
#include "board.h"
#include <Arduino.h>
#include "DataConversions.h"
#include "Structs.h"

SerialPortExpander *SerialPortExpander::firstInstance;

DmaSerial dma_serial1 = DmaSerial((Uart*)USART0, ID_USART0);

// default constructor
SerialPortExpander::SerialPortExpander()
{
	if(!firstInstance)
	{
		firstInstance = this;
	}

} //SerialPortExpander


void SerialPortExpander::init(void)
{
	pinMode(EXPANDER_CHAN_A, OUTPUT);              //Set the digital pin as output
	pinMode(EXPANDER_CHAN_B, OUTPUT);              //Set the digital pin as output
	pinMode(EXPANDER_CHAN_C, OUTPUT);              //Set the digital pin as output
	
	digitalWrite(EXPANDER_CHAN_A, bitRead(0, 0));             //Here we have two commands combined into one.
	digitalWrite(EXPANDER_CHAN_B, bitRead(0, 1));             //The digitalWrite command sets a pin to 1/0 (high or low)
	digitalWrite(EXPANDER_CHAN_C, bitRead(0, 2));
	
	Serial1.begin(115200);
	//Serial1.setTimeout(50);

	//dma_serial1.begin(SERIAL_BAUD);
	
}

void SerialPortExpander::ProcessSerialExpander(SerialCommand *sCommand)
{
	

	char charBuilder[MAX_CMD_LENGTH] = {0};
	
	if (sCommand->hardwareType == hardwareType.puller)
	{
		if (sCommand->value == NULL)
		{
			sprintf(charBuilder, "%s", sCommand->command);
		}
		else
		{
			sprintf(charBuilder, "%s %s", sCommand->command, sCommand->value);
		}
		sprintf(charBuilder, "%s%s", charBuilder, "\r");
	}
	else
	{
		BuildSerialOutput(sCommand, charBuilder);
		sprintf(charBuilder, "%s%s", charBuilder, "\n");
	}
	


	Open_channel(sCommand);
	
	Serial1.write(charBuilder);
	//dma_serial1.put(charBuilder);
	//Serial1.flush();

}

void SerialPortExpander::Open_channel(SerialCommand *sCommand)
{                             //This function controls what UART port is opened.

	//port = atoi(sCommand->hardwareType);                           //Convert the ASCII char value of the port to be opened into an int
	port = sCommand->hardwareType;
	if (port < 1 || port > 8)port = 1;              //If the value of the port is within range (1-8) then open that port. If it’s not in range set it port 1
	port -= 1;                                      //So, this device knows its ports as 0-1 but we have them labeled 1-8 by subtracting one from the port to be opened we correct for this.

	digitalWrite(EXPANDER_CHAN_A, bitRead(port, 0));             //Here we have two commands combined into one.
	digitalWrite(EXPANDER_CHAN_B, bitRead(port, 1));             //The digitalWrite command sets a pin to 1/0 (high or low)
	digitalWrite(EXPANDER_CHAN_C, bitRead(port, 2));             //The bitRead command tells us what the bit value is for a specific bit location of a number
	delay(2);                                       //this is needed to make sure the channel switching event has completed
	return;                                         //go back
}



// default destructor
SerialPortExpander::~SerialPortExpander()
{
} //~SerialPortExpander

