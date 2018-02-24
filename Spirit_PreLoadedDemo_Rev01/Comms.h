#ifndef COMMS_H
#define COMMS_H

#include "Arduino.h"
#include "Navigation.h"
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

// ***************************************************
// Global Variables used for Communication
// ***************************************************

#define I2C_BUFFER_LENGTH 22    //default is 22, allows 1 register address, 1 checksum, and 20 useable bytes
#define SPI_BUFFER_LENGTH 11    //default is 11, allows 1 regsiter address, 1 checksum, 8 useable bytes,
                                //and 1 tail byte as outgoint clocks out one byte behind ingoing

#define SERIAL_SPEED 57600
#define PIC_I2C_ADDRESS 0x32

extern uint8_t I2CData[I2C_BUFFER_LENGTH];     //I2C Data Buffer (in and out are shared). Define length above.
extern volatile byte SPI_BufferIn[SPI_BUFFER_LENGTH];   //SPI Data Incomming Buffer. Define length above.
extern volatile byte SPI_BufferOut[SPI_BUFFER_LENGTH];  //SPI Data Outgoing Buffer. Define length above.
extern volatile byte SPI_BufferPosition;                //
extern volatile boolean SPI_InProgress;                 //
extern unsigned char regTarget;                //Target I2C register on PIC processor
extern signed int regValue;                    //Value to write to I2C target register


// ***************************************************
// Ringo I2C
// ***************************************************

//Note: Ringo uses a custom library called "RingoWire.h" for I2C communication with
//the Accelerometer and Gyroscope which is a chopped down version of the 
//official Arduino "Wire.h" library which eliminates portions of the 
//original Wire.h which are not necessary for Ringo and significantly reduces
//code space required.
extern uint8_t I2CReadRegs(uint8_t Device, uint8_t Reg, uint8_t *RxBuffer, uint8_t Length);
extern uint8_t I2CReadReg(uint8_t Device, uint8_t Reg);
extern void I2CWriteRegs(uint8_t Device, uint8_t Reg, uint8_t *TxBuffer, uint8_t Length);
extern void I2CWriteReg(uint8_t Device, uint8_t Reg, uint8_t TxData);
extern void I2CBegin(void);

// ***************************************************
// end Ringo I2C
// ***************************************************

#endif
