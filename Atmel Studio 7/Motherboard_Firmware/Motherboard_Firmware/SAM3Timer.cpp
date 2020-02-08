/* 
* SAM3Timer.cpp
*
* Created: 2/2/2020 3:59:14 PM
* Author: Anthony
*/


#include "SAM3Timer.h"
#include "Arduino.h"

// default constructor
SAM3Timer::SAM3Timer()
{
} //SAM3Timer

void SAM3Timer::startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) 
{
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t)irq);
	//TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
	TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR | TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK4);
	uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
	TC_SetRA(tc, channel, rc/2); //50% high, 50% low
	TC_SetRC(tc, channel, rc);
	TC_Start(tc, channel);
	tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
	tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
	NVIC_EnableIRQ(irq);
}

void SAM3Timer::changeTimerFrequency(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t)irq);
	//TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
	TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR | TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK1);
	uint32_t rc = VARIANT_MCK/2/frequency; //2 because we selected TIMER_CLOCK1 above
	TC_SetRA(tc, channel, rc/2); //50% high, 50% low
	TC_SetRC(tc, channel, rc);
	TC_Start(tc, channel);
	tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
	tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
	//NVIC_EnableIRQ(irq);
}

// default destructor
SAM3Timer::~SAM3Timer()
{
} //~SAM3Timer
