/*
 * DataConversions.h
 *
 * Created: 12/28/2018 8:20:17 AM
 *  Author: Anthony
 */ 


#ifndef DATACONVERSIONS_H_
#define DATACONVERSIONS_H_

#include <stdio.h>
#include <avr/dtostrf.h>
#include "SerialProcessing.h"

#define INT_FORMAT "%d"
#define FLOAT_FORMAT "%.2f"
#define DOUBLE_FORMAT "%lf"
#define LL_FORMAT "%lld"
#define OUTPUT_STRING_DSS "%d;%s;%s;"
#define OUTPUT_STRING_DSSS "%d;%s;%s;%d;"
#define OUTPUT_STRING_DS "%d;%s;"
#define STRING_FORMAT "%s"
#define SCREENFORMAT "%s%s"

#define CONVERT_NUMBER_TO_STRING(FORMAT, VARIABLE, OUTPUT_BUFFER) \
do { \
  sprintf(OUTPUT_BUFFER, FORMAT, VARIABLE); \
} while(false)


#define CONVERT_FLOAT_TO_STRING(INPUTNUMBER, OUTPUT_BUFFER) \
do{ \
	dtostrf(INPUTNUMBER, 5, 3, OUTPUT_BUFFER); \
} while(false)
//DEVICENUMBER = INDICATOR/SPOOLER/EXTRUDER, DATA = RAW OUTPUT DATA, OUTPUTBUFFER = THE BUFFER TO STORE THE DATA ON RETURN
//#define BUILD_SERIAL_OUTPUT(DEVICENUMBER, DATA, OUTPUTBUFFER)\
//do {\
  //char serialData[100];\
  //strcpy(serialData, DATA);\
  //sprintf(OUTPUTBUFFER, OUTPUT_STRING_FORMAT, DEVICENUMBER, serialData); \
//} while(false)




#endif /* DATACONVERSIONS_H_ */
