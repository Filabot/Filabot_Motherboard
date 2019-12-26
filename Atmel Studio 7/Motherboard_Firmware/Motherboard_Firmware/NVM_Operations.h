/*
* NVM_Operations.h
*
* Created: 11/27/2019 3:22:14 PM
*  Author: Anthony
*/


#ifndef NVM_OPERATIONS_H_
#define NVM_OPERATIONS_H_

#include <Arduino.h>
#include "board.h"
#include "hardwareTypes.h"
#include "Structs.h"



class NVM_Operations {

	public:
	NVM_Operations();
	~NVM_Operations();

	void init();
	bool SetSpecificGravity(float value);
	char *GetSpecificGravity(void);
	bool SetSpoolWeightLimit(uint32_t value);
	char *GetSpoolWeightLimit(void);


	private:
	static NVM_Operations *firstInstance;
	NVM_Operations( const NVM_Operations &c );
	NVM_Operations& operator=( const NVM_Operations &c );
	bool SaveStorage(void);
	bool ReadStorage(void);
	_NVM_Storage NVM_Storage;

};


#endif /* NVM_OPERATIONS_H_ */