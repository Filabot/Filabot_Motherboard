/*
* hardwareTypes.h
*
* Created: 3/12/2019 4:49:14 PM
* Author: Anthony
*/


#ifndef __HARDWARETYPES_H__
#define __HARDWARETYPES_H__

const int INDICATOR = 0; //indicator
const int PULLER = 1; //spooler
const int EXTRUDER = 2; //extruder
const int TRAVERSE = 3; //traverse
const int SCREEN = 99;
const int INTERNALDEVICE = 100;

typedef struct{
	uint16_t indicator; //indicator
	uint16_t puller; //spooler
	uint16_t extruder; //extruder
	uint16_t traverse; //traverse
	uint16_t screen;
	uint16_t internal; //internal
} struct_hardwareTypes;

const static struct_hardwareTypes hardwareType =
{
	
	.indicator = INDICATOR,
	.puller = PULLER,
	.extruder = EXTRUDER,
	.traverse = TRAVERSE,
	.screen = SCREEN,
	.internal = INTERNALDEVICE
};

static uint16_t int_hardwareTypes[] =
{
	
	
	INDICATOR, //0 = indicator
	PULLER, //1 = spooler
	EXTRUDER, //2 = extruder
	TRAVERSE, //3 = traverse
	SCREEN,
	INTERNALDEVICE, // 100 = internal
};


#endif //__HARDWARETYPES_H__
