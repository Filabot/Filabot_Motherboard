/*
* SpcProcessing.h
*
* Created: 12/20/2018 4:57:59 PM
* Author: Anthony
*/


#ifndef __SPCPROCESSING_H__
#define __SPCPROCESSING_H__

#include <Arduino.h>
#include "board.h"
#include "Error.h"
#include "Structs.h"


class SpcProcessing
{
	//variables
	public:
	bool IsInSimulationMode = false;
	volatile bool HasNewData = false;
	bool HasError(void);
	volatile bool ISR_READY(void);
	
	

	protected:
	
	private:
	char serialOutputBuffer[MAX_CMD_LENGTH] = {0};
	Error eError;
	double SPCDiameter;
	SpcDiameter spcDiameter;

		

	//functions
	public:
	SpcProcessing();
	~SpcProcessing();
	void RunSPCDataLoop(void);
	void init(void);
	SpcDiameter *GetDiameter(void);
	Error *GetError(void);
	void StartQuery(void);
	void StopQuery(void);
	bool QueryFailed(uint32_t waitTime);
	
	protected:
	
	private:
	static SpcProcessing *firstinstance;
	SpcProcessing( const SpcProcessing &c );
	SpcProcessing& operator=( const SpcProcessing &c );
	
	
}; //spcProcessing

#endif //__SPCPROCESSING_H__
