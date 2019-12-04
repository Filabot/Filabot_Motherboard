/* 
* SerialPortExpander.h
*
* Created: 12/27/2018 10:18:01 AM
* Author: Anthony
*/


#ifndef __SERIALPORTEXPANDER_H__
#define __SERIALPORTEXPANDER_H__

#include <Arduino.h>

#include "Structs.h"

class SerialPortExpander
{
//variables
public:
protected:
private:
  byte computer_bytes_received = 0;    //We need to know how many characters bytes have been received
  byte sensor_bytes_received = 0;      //We need to know how many characters bytes have been received
  const static byte numberOfBufferBytes = 32;
  int EXPANDER_CHAN_A = 50;                         //2560 pin 41 (PL6)
  int EXPANDER_CHAN_B = 48;                         //2560 pin 42 (PL7)
  int EXPANDER_CHAN_C = 46;                         //2560 pin 51 (PG0)
  int port = 0;                       //what port to open
  char computerdata[numberOfBufferBytes];               //A 20 byte character array to hold incoming data from a pc/mac/other
  char sensordata[numberOfBufferBytes];                 //A 30 byte character array to hold incoming data from the sensors
  char *channel;                       //Char pointer used in string parsing
  char *cmd;                           //Char pointer used in string parsing
  

//functions
public:
  SerialPortExpander();
  void init(void);
  void ProcessSerialExpander(SerialCommand *sCommand);
  ~SerialPortExpander();
protected:
private:
  SerialPortExpander( const SerialPortExpander &c );
  void Open_channel(SerialCommand *sCommand);
  static SerialPortExpander *firstInstance;
  SerialPortExpander& operator=( const SerialPortExpander &c );

}; //SerialPortExpander

#endif //__SERIALPORTEXPANDER_H__

