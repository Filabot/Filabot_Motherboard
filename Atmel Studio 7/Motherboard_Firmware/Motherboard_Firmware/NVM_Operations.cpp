/*
* NVM_Operations.cpp
*
* Created: 11/27/2019 3:22:29 PM
*  Author: Anthony
*/

#include <Arduino.h>
#include "board.h"
#include "hardwareTypes.h"
#include "Structs.h"
#include "DueFlashStorage.h"
#include "NVM_Operations.h"
#include "DataConversions.h"

NVM_Operations *NVM_Operations::firstInstance;

DueFlashStorage dueFlashStorage;

NVM_Operations::NVM_Operations()
{
	if(!firstInstance)
	{
		firstInstance = this;
	}
}

void NVM_Operations::init(){
	
	if (dueFlashStorage.read(0) == 255) // flash bytes will be 255 at first run
	{
		NVM_Storage.LowerLimit = 0.0;
		NVM_Storage.NominalDiameter = 0.0;
		NVM_Storage.SpecificGravity = 0.0;
		NVM_Storage.SpoolWeight = 0;
		NVM_Storage.SpoolWidth = 0;
		NVM_Storage.TravereStart = 0;
		NVM_Storage.TraverseInnerOffset = 0.0;
		NVM_Storage.UpperLimit = 0.0;

		byte byteStorage[sizeof(_NVM_Storage)]; // create byte array to store the struct
		memcpy(byteStorage, &NVM_Storage, sizeof(_NVM_Storage)); // copy the struct to the byte array
		dueFlashStorage.write(4, byteStorage, sizeof(_NVM_Storage)); // write byte array to flash
		dueFlashStorage.write(0, 0);
	}

	ReadStorage();
}

bool NVM_Operations::SetSpecificGravity(float value)
{
	if (value != NVM_Storage.SpecificGravity)
	{
		NVM_Storage.SpecificGravity = value;
		SaveStorage();
		ReadStorage();
	}

	return true;
}

char *NVM_Operations::GetSpecificGravity(void)
{
	static char output[MAX_CMD_LENGTH] = {0};
	CONVERT_NUMBER_TO_STRING("%0.2f", NVM_Storage.SpecificGravity, output);
	

	return output;
}

bool NVM_Operations::SaveStorage(void)
{
	byte byteStorage[sizeof(_NVM_Storage)]; // create byte array to store the struct
	memcpy(byteStorage, &NVM_Storage, sizeof(_NVM_Storage)); // copy the struct to the byte array
	dueFlashStorage.write(4, byteStorage, sizeof(_NVM_Storage)); // write byte array to flash
	return true;
}

bool NVM_Operations::ReadStorage(void)
{
	byte *byteStoragePtr = dueFlashStorage.readAddress(4); // byte array which is read from flash at address 4
	memcpy(&NVM_Storage, byteStoragePtr, sizeof(_NVM_Storage)); // copy byte array to temporary struct

	return true;
}

// default destructor
NVM_Operations::~NVM_Operations()
{
} //~NVM_Operations

