/*
 * Error.h
 *
 * Created: 4/18/2019 9:20:54 AM
 *  Author: Anthony
 */ 


#ifndef ERROR_H_
#define ERROR_H_

//#include <Vector.h>
#include "Arduino.h"



typedef struct
{
	uint16_t hardwareType; //see hardwareTypes.h
	//char *command;
	char *errorDescription;
	byte errorLevel;
	byte errorCode;
} Error;

//char *ErrorCodes[];

const int DATASTREAM_VALIDATION_FAILED = 1;
const int DEVICE_DISCONNECTED = 2;

//errorLevel:
// 1 = datastream validation failed
// 2 = device disconnected
// 3 = undefined

const int DIAMETER_DEVICE_DISCONNECTED  = 1;
const int SPC_DATA_ERROR = 2;
//errorCode:
// 1 = DIAMETER DEVICE DISCONNECTED
// 2 = SPC DATA ERROR

typedef struct{
	byte datastream_validation_failed; //indicator
	byte device_disconnected; //spooler
	
} struct_errorLevels;

const static struct_errorLevels errorLevel =
{
	.datastream_validation_failed = DATASTREAM_VALIDATION_FAILED,
	.device_disconnected = DEVICE_DISCONNECTED,
};

typedef struct{
	byte diameter_device_disconnected; //indicator
	byte spc_data_error; //spooler
	
} struct_errorCodes;

const static struct_errorCodes errorCode =
{
	.diameter_device_disconnected = DIAMETER_DEVICE_DISCONNECTED,
	.spc_data_error = SPC_DATA_ERROR,
};





void startErrorHandler(void);
void AddError(Error *eError);
void ClearError(byte ErrorCode);
bool HasErrors(void);
bool HasErrorCode(byte code);
Error *GetErrorByCode(byte code);











#endif /* ERROR_H_ */