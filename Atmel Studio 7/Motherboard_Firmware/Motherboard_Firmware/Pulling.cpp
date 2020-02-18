/*
* Pulling.cpp
*
* Created: 2/14/2020 4:16:06 PM
* Author: Anthony
*/


#include "Pulling.h"
#include "Arduino.h"
#include "SAM3Timer.h"

static float _rpm = 0.0;

// default constructor
Pulling::Pulling()
{

} //Pulling

void Pulling::SetWheelRPM(float rpm)
{
	
	if (rpm > 300.0)
	{
		rpm = 300.0;
	}
	if (rpm < 0.0)
	{
		rpm = 0.0;
	}
	_rpm = rpm;
	int32_t freq = (((rpm * (16.0 * 200.0) ) / 60.0) * 1.857) * 2.0;
	SAM3Timer::changeTimerFrequency(TC1, 0, TC3_IRQn, freq);
}

void Pulling::IncDecWheelRPM(float rpm)
{
	_rpm += rpm;

	SetWheelRPM(_rpm);
}

float Pulling::GetWheelRPM()
{
	return _rpm;
}

// default destructor
Pulling::~Pulling()
{
} //~Pulling
