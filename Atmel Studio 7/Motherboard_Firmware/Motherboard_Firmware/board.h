#ifndef __BOARD_H__
#define __BOARD_H__

#include <Arduino.h>
#include "HardwareTypes.h"
#include "SerialNative.h"
#include "Structs.h"
#include "NVM_Operations.h"
#include "DmaSerial.h"

#define SERIAL_BAUD (115200) //baud rate for the serial ports

#define INDICATOR_REQ  30 //when set high the CLK pulse train starts
#define INDICATOR_DAT  28
#define INDICATOR_CLK  2//29  CLK is a pulse train provided by the indicator
#define ENCODER_PINA   26
#define ENCODER_PINB   24
#define ENCODER_PB	   51
#define DIAMETER05		31
#define DIAMETER420		33
#define START_PB		47
#define STOP_PB			49


#define MYHARDWARETYPE INTERNALDEVICE

#define MAX_CMD_LENGTH 63
#define DELIMITER ";"

extern bool SIMULATIONACTIVE;
extern _SerialNative SerialNative;
extern DmaSerial dma_serial1;
extern uint32_t SPOOLWEIGHT;
extern uint32_t SPOOLWEIGHTLIMIT;
extern float FILAMENTLENGTH;
extern float FILAMENTDIAMETER;
extern volatile bool HANDSHAKE;
extern int32_t KEEPALIVETIMER;
extern NVM_Operations nvm_operations;
//
//#define GPIO_LOW(pin) {PORT->Group[g_APinDescription[(pin)].pPort].OUTCLR.reg = (1ul << g_APinDescription[(pin)].ulPin);}
//#define GPIO_HIGH(pin) {PORT->Group[g_APinDescription[(pin)].pPort].OUTSET.reg = (1ul << g_APinDescription[(pin)].ulPin);}
//#define GPIO_OUTPUT(pin) {PORT->Group[g_APinDescription[(pin)].pPort].PINCFG[g_APinDescription[(pin)].ulPin].reg &=~(uint8_t)(PORT_PINCFG_INEN) ;  PORT->Group[g_APinDescription[(pin)].pPort].DIRSET.reg = (uint32_t)(1<<g_APinDescription[(pin)].ulPin) ;}
//
//#define PIN_GPIO_OUTPUT(pin) {PORT->Group[g_APinDescription[(pin)].pPort].PINCFG[g_APinDescription[(pin)].ulPin].reg &=~(uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_PMUXEN) ;  PORT->Group[g_APinDescription[(pin)].pPort].DIRSET.reg = (uint32_t)(1<<g_APinDescription[(pin)].ulPin) ;}
//
//#define PIN_GPIO(pin) {PORT->Group[g_APinDescription[(pin)].pPort].PINCFG[g_APinDescription[(pin)].ulPin].reg &=~(uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_PMUXEN);}
//#define GPIO_READ(ulPin) {g_APinDescription[ulPin].pPort->PIO_PDSR & (1ul << g_APinDescription[ulPin].ulPin) != 0}
#define GPIO_READ(pin) {PIO_Get( g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin ) == 1  }
//#define PIN_PERIPH(pin) {PORT->Group[g_APinDescription[(pin)].pPort].PINCFG[g_APinDescription[(pin)].ulPin].reg |= PORT_PINCFG_PMUXEN;}

#endif//__BOARD_H__

