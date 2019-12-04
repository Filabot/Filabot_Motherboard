/*
* SerialNative.h
*
* Created: 6/19/2019 3:10:25 PM
* Author: Anthony
*/


#ifndef __SERIALNATIVE_H__
#define __SERIALNATIVE_H__
#include "Arduino.h"


class _SerialNative : public Serial_
{
	//variables
	public:
	
	
	protected:
	private:

	//functions
	public:
	_SerialNative();
	~_SerialNative();

	//bool begin(uint32_t baud_count);
	//bool available(void);
	int read();
	size_t readBytesUntil (char terminator, uint8_t *buffer, size_t length);
	size_t readBytesUntil( char terminator, char *buffer, size_t length);
	void print(const char *c, ...);
	void print(char *c, ...);
	void print(int);
	void println(const char *c, ...);
	void println(char *c, ...);
	void println(int);
	void setTimeout(unsigned long timeout);
	protected:
	private:
	void ConcatenateChars(char *buffer_A, char *buffer_B, char *outputBuffer);
	_SerialNative( const _SerialNative &c );
	_SerialNative& operator=( const _SerialNative &c );

}; //_SerialNative

#endif //__SERIALNATIVE_H__
