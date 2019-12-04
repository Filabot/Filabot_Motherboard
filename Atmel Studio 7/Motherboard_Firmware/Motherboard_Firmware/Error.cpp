/*
* Error.cpp
*
* Created: 4/23/2019 9:44:09 AM
*  Author: Anthony
*/

#include "Error.h"
#include <Vector.h>

Error errorArray[15];
Vector<Error> errorVector;

char *ErrorCodes[] =
{
	"RESERVED",
	"DIAMETER   DISCONNECT",
	"SPC  DATA  ERROR"

};



void startErrorHandler(void)
{
	errorVector.setStorage(errorArray);
}

void AddError(Error *eError)
{
	Error *err;


	if (!errorVector.full())
	{
		bool itemExists;
		for (int i = 0; i < errorVector.max_size(); i++)
		{
			err = &errorVector.at(i);
			itemExists = err->errorCode == eError->errorCode;
			if (itemExists){break;}
		}

		if (!itemExists){
			eError->errorDescription =  ErrorCodes[eError->errorCode];
			errorVector.push_back(*eError);
		}
	}

	//Error ev = errorVector.at(0);
	//Serial.print("");

}

void ClearError(byte ErrorCode)
{
	Error *err;

	

	for (int i = 0; i < errorVector.max_size(); i++)
	{
		err = &errorVector.at(i);
		if (err->errorCode == ErrorCode){
			errorVector.remove(i);
			err = &errorVector.at(i);
			err->errorCode = 0;
			err->errorLevel = 0;
			err->hardwareType = 0;
			//SerialUSB.print("");
		}
		
		
	}

}

bool HasErrors(void)
{
	
	for (int i = 0; i < errorVector.max_size(); i++)
	{
		if (errorVector.at(i).errorCode > 0)
		{
			return true;
		}
	}
	
	
	return false;

}

bool HasErrorCode(byte code)
{

	for (int i = 0; i < errorVector.max_size(); i++)
	{
		if (errorVector.at(i).errorCode == code)
		{
			return true;
		}
	}


	return false;
}

Error *GetErrorByCode(byte code)
{
	Error *err;
	//err.errorCode = 0;
	//err.errorLevel = 0;
	//err.hardwareType = 0;

	for (int i = 0; i < errorVector.max_size(); i++)
	{
		if (errorVector.at(i).errorCode == code)
		{
			err = &errorVector.at(i);
		}
	}

	return err;


}
