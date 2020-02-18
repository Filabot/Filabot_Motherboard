/*
* SerialNative.cpp
*
* Created: 6/19/2019 3:10:25 PM
* Author: Anthony
*/


#include "SerialNative.h"
#include "board.h"
#include "DataConversions.h"

// default constructor
_SerialNative::_SerialNative()
{
} //SerialNative

void _SerialNative::setTimeout(unsigned long timeout)
{
	SerialUSB.setTimeout(timeout);
}

void _SerialNative::print(const char *c, ...)
{
	int ret=0;
	char vastr[MAX_CMD_LENGTH]={0};
	
	va_list ap;

	memset(vastr,0,MAX_CMD_LENGTH);
	va_start(ap,c);
	ret=vsprintf(vastr,(const char *)c,ap);
	va_end(ap);

	SerialUSB.write(vastr, MAX_CMD_LENGTH);
	
	//if (SerialUSB){	SerialUSB.print(vastr); }
}

void _SerialNative::print(char *c, ...)
{
	int ret=0;
	char vastr[MAX_CMD_LENGTH]={0};
	
	va_list ap;

	memset(vastr,0,MAX_CMD_LENGTH);
	va_start(ap,c);
	ret=vsprintf(vastr,(char *)c,ap);
	va_end(ap);

	SerialUSB.write(vastr, MAX_CMD_LENGTH);

	//if (SerialUSB){	SerialUSB.print(vastr); }
}

void _SerialNative::println(const char *c, ...)
{
	int ret=0;
	char vastr[MAX_CMD_LENGTH]={0};
	
	va_list ap;

	memset(vastr,0,MAX_CMD_LENGTH);
	va_start(ap,c);
	ret=vsprintf(vastr,(const char *)c,ap);
	va_end(ap);

	

	//if (SerialUSB)
	//{
	//SerialUSB.println(vastr);
	//
	//}
	

	SerialUSB.write(vastr, MAX_CMD_LENGTH);
	SerialUSB.write("\r\n");
}

void _SerialNative::println(char *c, ...)
{
	int ret=0;
	char vastr[MAX_CMD_LENGTH]={0};
	
	va_list ap;

	memset(vastr,0,MAX_CMD_LENGTH);
	va_start(ap,c);
	ret=vsprintf(vastr,(char *)c,ap);
	va_end(ap);

	SerialUSB.write(vastr, MAX_CMD_LENGTH);
	SerialUSB.write("\r\n");

	//if (SerialUSB){	SerialUSB.println(vastr); }
}

void _SerialNative::print(int num)
{
	if (SerialUSB){SerialUSB.print(num);}
}

void _SerialNative::println(int num)
{
	if (SerialUSB){SerialUSB.println(num);}
}

size_t _SerialNative::readBytesUntil(char terminator, uint8_t *buffer, size_t length)
{
	if (SerialUSB){ return readBytesUntil(terminator, (char *)buffer, length);}
}

size_t _SerialNative::readBytesUntil( char terminator, char *buffer, size_t length)
{
	if (SerialUSB){ return SerialUSB.readBytesUntil(terminator, buffer, length);}
}
int _SerialNative::read()
{
	if(SerialUSB) {return SerialUSB.read();}
}

void _SerialNative::ConcatenateChars(char *buffer_A, char *buffer_B, char *outputBuffer)
{
	
	sprintf(outputBuffer, SCREENFORMAT, buffer_A, buffer_B);
}

// default destructor
_SerialNative::~_SerialNative()
{
} //~SerialNative
