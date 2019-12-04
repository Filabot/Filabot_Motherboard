/*
 * Structs.h
 *
 * Created: 5/23/2019 4:42:21 PM
 *  Author: Anthony
 */ 


#ifndef STRUCTS_H_
#define STRUCTS_H_

typedef struct
{
	uint16_t hardwareType; //see hardwareTypes.h
	char *command;
	char *value;
} SerialCommand;

typedef struct 
{
	char charDiameterNoDecimal[10];
	char charDiameterWithDecimal[10];
	int decimalPointLocation;
	uint16_t intDiameterNoDecimal;
	float floatDiameterWithDecimal;
} SpcDiameter;

typedef struct
{
	char *RPM;
	char Weight[10];
	char Slippage[10];
} Spool;


typedef struct
{
	char *RPM;
} Puller;

typedef struct  
{
	__attribute__((__aligned__(8))) float NominalDiameter;
	__attribute__((__aligned__(8))) float UpperLimit;
	__attribute__((__aligned__(8))) float LowerLimit;
	__attribute__((__aligned__(8))) float SpecificGravity;
	__attribute__((__aligned__(8))) uint32_t SpoolWeight;
	__attribute__((__aligned__(8))) float TraverseInnerOffset;
	__attribute__((__aligned__(8))) uint32_t SpoolWidth;
	__attribute__((__aligned__(8))) byte TravereStart;

} _NVM_Storage;


#endif /* STRUCTS_H_ */