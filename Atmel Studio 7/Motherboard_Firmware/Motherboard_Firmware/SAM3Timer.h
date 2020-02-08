/* 
* SAM3Timer.h
*
* Created: 2/2/2020 3:59:15 PM
* Author: Anthony
*/


#ifndef __SAM3TIMER_H__
#define __SAM3TIMER_H__

#include "Arduino.h"

class SAM3Timer
{
//variables
public:
protected:
private:

//functions
public:
	SAM3Timer();
	~SAM3Timer();

	static void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
	static void changeTimerFrequency(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);


protected:
private:
	SAM3Timer( const SAM3Timer &c );
	SAM3Timer& operator=( const SAM3Timer &c );

}; //SAM3Timer

#endif //__SAM3TIMER_H__
