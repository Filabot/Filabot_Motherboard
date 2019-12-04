#ifndef __SERIALPROCESSING_H__
#define __SERIALPROCESSING_H__

#include <Arduino.h>
#include "board.h"
#include "hardwareTypes.h"
#include "Structs.h"
#include "SerialNative.h"


const byte numChars = 32;
static char receivedChars[numChars]; // an array to store the received data



class SerialProcessing {

	public:
	SerialProcessing();
	~SerialProcessing();
	
	void init();
	void Poll(void);
	unsigned int CheckSerial(HardwareSerial *port, int portNumber);
	unsigned int CheckSerial(_SerialNative *port, int portNumber);
	//unsigned int CheckSerial(_SerialNative *port, int portNumber);
	unsigned int CommandParse(SerialCommand *sCommand, char str[MAX_CMD_LENGTH]);
	unsigned int SendScreenData(SerialCommand *sCommand);
	unsigned int SendDataToDevice(SerialCommand *sCommand);
	bool newData = false;
	bool commandActive;
	bool FullUpdateRequested;
	bool FilamentCapture;




	private:
	static SerialProcessing *firstInstance;
	SerialProcessing( const SerialProcessing &c );
	SerialProcessing& operator=( const SerialProcessing &c );
	void CheckInteralCommands(SerialCommand *sCommand);
	char serialOutputBuffer[MAX_CMD_LENGTH] = {0};
	unsigned int ProcessDataFromPC(SerialCommand *sCommand);
	unsigned int SendToPC(SerialCommand *sCommand);
	//bool commandActive = false;  //primitive lock for serial processing to expander
	void str_replace(char src[MAX_CMD_LENGTH], char *oldchars, char *newchars);
	void ProcessFilamentCaptureState(SerialCommand *sCommand);

};


char * CleanseData(char *data);


SerialCommand GetSerialArgs(char * serialData);
bool ExistsInIntArray(uint16_t *arrayToCheck, size_t arraySize, uint16_t numberToCheck);
void BuildSerialOutput(SerialCommand *sCommand, char *outputBuffer);
bool startsWith(const char *pre, const char *str);
void PrintRandomRPMData();






#endif//__SERIALPROCESSING_H__

