
#include "Hardware.h"
#include "Navigation.h"
#include "Comms.h"
#include <SPI.h>
#include <Adafruit_NeoPixel.h>


// ***************************************************
// Global Variables used for Communication
// ***************************************************

uint8_t I2CData[I2C_BUFFER_LENGTH];     //I2C Data Buffer (in and out are shared). Define length in Comms.h
volatile byte SPI_BufferIn[SPI_BUFFER_LENGTH];   //SPI Data Incomming Buffer. Define length above.
volatile byte SPI_BufferOut[SPI_BUFFER_LENGTH];  //SPI Data Outgoing Buffer. Define length above.
volatile byte SPI_BufferPosition=0;              //

volatile boolean SPI_InProgress;                 //
unsigned char regTarget=0;              //Target I2C register on PIC processor
signed int regValue=0;                  //Value to write to I2C target register


// ***************************************************
// PIC Processor Query & Write Data Functions
// ***************************************************


/*************************************************************
*             GENERAL STATUS REGISTERS
*************************************************************/

/*       
void PIC_ReadDebug(void){                             //TARGET REGISTER: 0    Read Debug Information
  regTarget = 0;
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 6);
  debug[0] = I2CData[2];
  debug[1] = I2CData[3];
  debug[2] = I2CData[4];
  debug[3] = I2CData[5];
}
*/

void PIC_ReadStatus(void){                            //TARGET REGISTER: 1    Read PIC Status Register                           
  regTarget = 1;                                        //                      Returns data from previous 20ms loop on PIC
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 3);  //                      Less overhead to process on PIC using this function
  PIC_StatusBank0  = I2CData[1];                        //                      Voltage returned in millivolts (mV) 1000 mV = 1v
  PIC_StatusBank1  = I2CData[2];
  PIC_CurrentWarningInt  = (PIC_StatusBank0 >> 0) & 1;
  PIC_VoltageWarningInt  = (PIC_StatusBank0 >> 1) & 1;
  PIC_ShutdownNowInt     = (PIC_StatusBank0 >> 2) & 1;
  PIC_MotorStopInt       = (PIC_StatusBank0 >> 3) & 1;
  PIC_SurfaceSenseInt    = (PIC_StatusBank0 >> 4) & 1;
  PIC_PowerSenseInt      = (PIC_StatusBank0 >> 5) & 1;
  PIC_RangeSenseInt      = (PIC_StatusBank0 >> 6) & 1;
  PIC_AmbSenseInt        = (PIC_StatusBank0 >> 7) & 1;
  PIC_UARTRxThreshInt    = (PIC_StatusBank1 >> 0) & 1;
  PIC_UARTRxNullInt      = (PIC_StatusBank1 >> 1) & 1;
  PIC_UARTTxInProg       = (PIC_StatusBank1 >> 2) & 1;
  if (PIC_MotorStopInt){
    motors(0,0);  //stop motors right in the interrupt handler
  }
}


void PIC_ReadInputs(void){                            //TARGET REGISTER: 5    Instant read of key PIC inputs. Auto populates variables.            
  regTarget = 5;                                      //                      Returns single byte which is much faster than calling                      
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 2);       //                      PIC_ReadAllInputPins()
  PIC_Button          = (I2CData[1] >> 0) & 1;
  PIC_Button_Pwr      = (I2CData[1] >> 1) & 1;
  PIC_Accel_Int1      = (I2CData[1] >> 2) & 1;
  PIC_Gyro_Int1       = (I2CData[1] >> 3) & 1;
  PIC_Gyro_Int2       = (I2CData[1] >> 4) & 1;
  PIC_ChgPresent      = (I2CData[1] >> 5) & 1;
  PIC_ChgInProg       = (I2CData[1] >> 6) & 1;
  PIC_XBee_Assoc      = (I2CData[1] >> 7) & 1;

  // Set chargeStatus based on inputs
  if ((PIC_ChgPresent)&&(PIC_ChgInProg)){ //both lines high = chg not present, no charge in prog
    chargeStatus=0;
  }else if ((!PIC_ChgPresent)&&(!PIC_ChgInProg)){ //both lines low = chg present, no charge in prog
    chargeStatus=1;
  }else if ((!PIC_ChgPresent)&&(PIC_ChgInProg)){ //ChgPresent low, InProg high = chg present, charge in progress
    chargeStatus=2;
  }else{
    chargeStatus=3; //an error occured in the circuit. This should not happen.
  }
}
  
void PIC_ReadComparators(void){                       //TARGET REGISTER: 6    Read PIC Threshold Comparators                           
  regTarget = 6;                                      //                      Returns data from previous 20ms loop on PIC
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 3);                             
  PIC_thresholdComparators1 = I2CData[1];                                    
  PIC_thresholdComparators2 = I2CData[2];
  PIC_CurrentComparator     = (PIC_thresholdComparators1 >> 0) & 1;
  PIC_VoltageComparator     = (PIC_thresholdComparators1 >> 1) & 1;
  PIC_LeftOuterComparator   = (PIC_thresholdComparators1 >> 2) & 1;
  PIC_LeftInnerComparator   = (PIC_thresholdComparators1 >> 3) & 1;
  PIC_RightOuterComparator  = (PIC_thresholdComparators1 >> 4) & 1;
  PIC_RightInnerComparator  = (PIC_thresholdComparators1 >> 5) & 1;
  PIC_RearOuterComparator   = (PIC_thresholdComparators1 >> 6) & 1;
  PIC_RearInnerComparator   = (PIC_thresholdComparators1 >> 7) & 1;
  PIC_RangefinderComparator = (PIC_thresholdComparators2 >> 0) & 1;
  PIC_AmbLeftComparator     = (PIC_thresholdComparators2 >> 1) & 1;
  PIC_AmbRightComparator    = (PIC_thresholdComparators2 >> 2) & 1;
  PIC_AmbRearComparator     = (PIC_thresholdComparators2 >> 3) & 1;
  PIC_WallPower             = (PIC_thresholdComparators2 >> 4) & 1;
}

void PIC_ReadUARTBufferCount(void){                   //TARGET REGISTER: 7    Returns of how many bytes are presently in the PIC UART Receive           
  regTarget = 7;                                        //                    buffer and PIC UART Transmit Buffer                      
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 3);  //                      
  PIC_UART_RxBufCount = I2CData[1];
  PIC_UART_TxBufCount = I2CData[2];
}

void PIC_ReadAllInputPins(void){                      //TARGET REGISTER: 8    Returns all 7 input PORT registers (all input pins).                            
  regTarget = 8;                                        //                    These can be parsed to determine the present input 
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 8);  //                    state of all pins on the processor.     
  PIC_PORTA = I2CData[1];
  PIC_PORTB = I2CData[2];
  PIC_PORTC = I2CData[3];
  PIC_PORTD = I2CData[4];
  PIC_PORTE = I2CData[5];
  PIC_PORTF = I2CData[6];
  PIC_PORTG = I2CData[7];
  PIC_Button      = (PIC_PORTB >> 2) & 1;
  PIC_Button_Pwr  = (PIC_PORTE >> 2) & 1;
  PIC_Accel_Int1  = (PIC_PORTB >> 4) & 1;
  PIC_Gyro_Int1   = (PIC_PORTD >> 4) & 1;
  PIC_Gyro_Int2   = (PIC_PORTB >> 5) & 1;
  PIC_38kHzRx     = (PIC_PORTC >> 1) & 1;
  PIC_ChgPresent  = (PIC_PORTE >> 0) & 1;
  PIC_ChgInProg   = (PIC_PORTE >> 7) & 1;
  PIC_XBee_Assoc  = (PIC_PORTG >> 4) & 1;
  PIC_Range_Echo  = (PIC_PORTG >> 0) & 1;
}

void PIC_ReadAllOutputPins(void){                     //TARGET REGISTER: 9    Returns all 7 output LAT registers (all output pins).                            
  regTarget = 9;                                      //                      These can be parsed to determine the present output 
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 8);       //                      state of all pins on the processor.     
  PIC_LATA = I2CData[1];
  PIC_LATB = I2CData[2];
  PIC_LATC = I2CData[3];
  PIC_LATD = I2CData[4];
  PIC_LATE = I2CData[5];
  PIC_LATF = I2CData[6];
  PIC_LATG = I2CData[7];
  PIC_PS5V0_EN    = (PIC_LATA >> 3) & 1;  
  PIC_PS3V3_EN    = (PIC_LATF >> 0) & 1;
  PIC_LED_PWR     = (PIC_LATA >> 7) & 1;  //Note: PWR, PiUp, and COM LED's are ON when LOW, OFF when HIGH
  PIC_LED_PIUP    = (PIC_LATD >> 0) & 1;  //Note: PWR, PiUp, and COM LED's are ON when LOW, OFF when HIGH
  PIC_LED_COM     = (PIC_LATD >> 5) & 1;  //Note: PWR, PiUp, and COM LED's are ON when LOW, OFF when HIGH 
  PIC_MUXA        = (PIC_LATC >> 0) & 1;
  PIC_MUXB        = (PIC_LATC >> 5) & 1;
  PIC_PI_SIG_EN   = (PIC_LATA >> 4) & 1;
  PIC_BATT_SW     = (PIC_LATE >> 1) & 1;
  PIC_WIFI_PD     = (PIC_LATA >> 6) & 1;
  PIC_PIEZO       = (PIC_LATE >> 3) & 1;
  PIC_RANGE_TRIG  = (PIC_LATG >> 3) & 1;
}

void PIC_ReadAllDirectionPins(void){                  //TARGET REGISTER: 10   Returns all 7 TRIS registers (all pin directions).                            
  regTarget = 10;                                     //                      These can be parsed to determine the present direction (input or output)
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 8);       //                      state of all pins on the processor. (1=input, 0=output)     
  //I2CData[1] now holds PIC_TRISA data
  //I2CData[2] now holds PIC_TRISB data
  //I2CData[3] now holds PIC_TRISC data
  //I2CData[4] now holds PIC_TRISD data
  //I2CData[5] now holds PIC_TRISE data
  //I2CData[6] now holds PIC_TRISF data
  //I2CData[7] now holds PIC_TRISG data
}

void PIC_ReadAllPullupPins(void){                     //TARGET REGISTER: 11   Returns all 7 WPU (weak pull up enable) registers.                            
  regTarget = 11;                                     //                      These can be parsed to determine the present weak pull-up 
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 8);       //                      state of all pins on the processor.     
  //I2CData[1] now holds 00 (there is no WPU on PIC Port A)
  //I2CData[2] now holds PIC_WPUB data
  //I2CData[3] now holds 00 (there is no WPU on PIC Port C)
  //I2CData[4] now holds PIC_WPUD data
  //I2CData[5] now holds PIC_WPUE data
  //I2CData[6] now holds 00 (there is no WPU on PIC Port F)
  //I2CData[7] now holds 00 (there is no WPU on PIC Port G)
}

void PIC_ReadAllAnalogPins(void){                     //TARGET REGISTER: 12   Returns all 7 ANSEL (analog enable) registers.                            
  regTarget = 12;                                     //                      These can be parsed to determine the present analog input 
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 8);       //                      state of all pins on the processor. (1=analog enabled, 0=analog disabled)     
  //I2CData[1] now holds PIC_ANSELA data
  //I2CData[2] now holds PIC_ANSELB data
  //I2CData[3] now holds 00 (there is no ANSEL on PIC Port C)
  //I2CData[4] now holds PIC_ANSELD data
  //I2CData[5] now holds PIC_ANSELE data
  //I2CData[6] now holds PIC_ANSELF data
  //I2CData[7] now holds PIC_ANSELG data
}

void PIC_ReadResetStatus(void){                       //TARGET REGISTER: 16   Read copy of PIC PCON register following most recent reset                           
  regTarget = 16;                                     //                     
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 3);                             
  PIC_PCON_Register = I2CData[1];
  PIC_STATUS_Register = I2CData[2];                                    
}

void PIC_SendUART(const String& TxString){                       //TARGET REGISTER: 17   Read copy of PIC PCON register following most recent reset                           
  regTarget = 17;
  byte bufferLength = TxString.length();
  //Serial.println(bufferLength);
  char charBuf[bufferLength+1];
  TxString.toCharArray(charBuf, bufferLength+1);

  for (byte counter = 0; counter <= bufferLength; counter++){
    I2CData[counter] = charBuf[counter];
    //Serial.print(I2CData[counter], HEX); Serial.print(" ");
  }
  //Serial.println();

  I2CWrite(PIC_I2C_ADDRESS, regTarget, bufferLength+1);
  delayMicroseconds(200);
}

String PIC_ReadUART(void){
  String receivedString;
  PIC_ReadUARTBufferCount();
  regTarget = 18;
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, PIC_UART_RxBufCount+2);
  for (unsigned char i = 1; i<= PIC_UART_RxBufCount; i++){
    //Serial.print((char)I2CData[i]); Serial.print(" "); //uncomment for debugging
    receivedString += (char)I2CData[i];
  }
  //Serial.println(); //uncomment for debugging
  return receivedString;
}

void PIC_SetLEDs(byte leds){                                    //TARGET REGISTER: 19   Manually toggles PWR, PiUp, COM LEDs
                                                                //                      Must call PIC_LEDsAuto() first to disable auto LEDs
  regTarget = 19;                                               //                      Pass bools set on(1)/off(0)
  I2CData[0] = leds;                                            //                      bit 2=PWR, 1=PiUp, 0=COM
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 1);                      //                      
}

/*************************************************************
*             ROVER POWER VALUES REGISTERS
*************************************************************/

void PIC_ReadPower(void){                               //TARGET REGISTER: 21   Read battery voltage and current                           
  regTarget = 21;                                       //                      Returns data from previous 20ms loop on PIC
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 5);  //                      Less overhead to process on PIC using this function
  powerVoltage = (I2CData[1] << 8) | I2CData[2];        //                      Voltage returned in millivolts (mV) 1000 mV = 1v
  powerCurrent = (I2CData[3] << 8) | I2CData[4];        //                      Current returned in milliamps (mA) 1000 mA = 1 Amp
                                                        //                      Max safe battery current is 2000 mA
}

void PIC_ReadPower_Instant(void){                       //TARGET REGISTER: 22   Read instant battery voltage and current                           
  regTarget = 22;                                       //                      PIC pauses to get instant readings, then returns readings
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 5);  //                      May delay other functions on PIC during read, usually harmless
  powerVoltage = (I2CData[1] << 8) | I2CData[2];        //                      Voltage returned in millivolts (mV) 1000 mV = 1v
  powerCurrent = (I2CData[3] << 8) | I2CData[4];        //                      Current returned in milliamps (mA) 1000 mA = 1 Amp
                                                        //                      Max safe battery current is 2000 mA
}

void PIC_ReadVoltage_Instant(void){                     //TARGET REGISTER: 23   Read instant battery voltage                          
  regTarget = 23;                                       //                      PIC pauses to get instant reading, then returns reading
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 3);  //                      May delay other functions on PIC during read, usually harmless
  powerVoltage = (I2CData[1] << 8) | I2CData[2];        //                      Voltage returned in millivolts (mV) 1000 mV = 1v
}

void PIC_ReadCurrent_Instant(void){                     //TARGET REGISTER: 24   Read instant current being drawn by entire robot                           
  regTarget = 24;                                       //                      PIC pauses to get instant reading, then returns reading
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 3);  //                      May delay other functions on PIC during read, usually harmless
  powerCurrent = (I2CData[1] << 8) | I2CData[2];        //                      Current returned in milliamps (mA) 1000 mA = 1 Amp
                                                        //                      Max safe battery current is 2000 mA
}

void PIC_SetPowerComparators(void){                     //TARGET REGISTER: 25   Set Power Comparator threshold (voltage/current).
  regTarget = 25;                                       //                      Update PIC_VoltageComparatorSetpoint and
  I2CData[0] = highByte(PIC_VoltageComparatorSetpoint); //                      PIC_CurrentComparatorSetpoint then call this
  I2CData[1] = lowByte(PIC_VoltageComparatorSetpoint);  //                      function.
  I2CData[2] = highByte(PIC_CurrentComparatorSetpoint);
  I2CData[3] = lowByte(PIC_CurrentComparatorSetpoint);
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 4);              //                      Write new values to PIC.
}

void PIC_ReadPowerAverages(void){                       //TARGET REGISTER: 26   Returns running average of current/voltage power readings.
  regTarget = 26;                                       //                      
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 5);
  powerVoltageAverage = (I2CData[1] << 8) | I2CData[2];
  powerCurrentAverage = (I2CData[3] << 8) | I2CData[4];
}

void PIC_ReadServoCurrent(void){                        //TARGET REGISTER: 27   Returns estimated survo current after servo movement
  regTarget = 27;                                       //                      
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 3);
  servoCurrent = (I2CData[1] << 8) | I2CData[2];
}


/*************************************************************
*             LIGHT SENSOR REGISTERS
*************************************************************/

void PIC_ReadAllSurfaceSensors(void){                 //TARGET REGISTER: 31   Returns all surface sensor readings. Results from last read cycle
  regTarget = 31;                                     //                      on the PIC. Accurate to within 20ms.
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 13);
  surfLeft0 = (I2CData[1] << 8) | I2CData[2];
  surfRight0 = (I2CData[3] << 8) | I2CData[4];
  surfRear0 = (I2CData[5] << 8) | I2CData[6];    
  surfLeft1 = (I2CData[7] << 8) | I2CData[8];    
  surfRight1 = (I2CData[9] << 8) | I2CData[10];    
  surfRear1 = (I2CData[11] << 8) | I2CData[12];
}

void PIC_ReadAllAmbientSensors(void){                 //TARGET REGISTER: 32   Returns all ambient sensor readings. Results from last read cycle
  regTarget = 32;                                     //                      on the PIC. Accurate to within 20ms.
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 7);
  ambLeft = (I2CData[1] << 8) | I2CData[2];
  ambRight = (I2CData[3] << 8) | I2CData[4];
  ambRear = (I2CData[5] << 8) | I2CData[6];
}

void PIC_ReadAllSurfaceSensorsInstant(void){          //TARGET REGISTER: 33   Returns all surface sensor readings. PIC will pause to get new
  regTarget = 33;                                     //                      instant read before reply.
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 13);
  surfLeft0 = (I2CData[1] << 8) | I2CData[2];
  surfRight0 = (I2CData[3] << 8) | I2CData[4];
  surfRear0 = (I2CData[5] << 8) | I2CData[6];    
  surfLeft1 = (I2CData[7] << 8) | I2CData[8];    
  surfRight1 = (I2CData[9] << 8) | I2CData[10];    
  surfRear1 = (I2CData[11] << 8) | I2CData[12];
}

void PIC_ReadAllAmbientSensorsInstant(void){          //TARGET REGISTER: 34   Returns all surface sensor readings. PIC will pause to get new
  regTarget = 34;                                     //                      instant read before reply.
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 7);
  ambLeft = (I2CData[1] << 8) | I2CData[2];
  ambRight = (I2CData[3] << 8) | I2CData[4];
  ambRear = (I2CData[5] << 8) | I2CData[6];
}

void PIC_ReadAllAmbientAverages(void){                //TARGET REGISTER: 35   Returns running average of all ambient sensor readings.
  regTarget = 35;                                     //                      
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 7);
  ambLeftAverage = (I2CData[1] << 8) | I2CData[2];
  ambRightAverage = (I2CData[3] << 8) | I2CData[4];
  ambRearAverage = (I2CData[5] << 8) | I2CData[6];
}

void PIC_ReadAllSurfaceAverages(void){                //TARGET REGISTER: 36   Returns all surface sensor readings. Results from last read cycle
  regTarget = 36;                                     //                      on the PIC. Accurate to within 20ms.
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 13);
  surfLeft0Average = (I2CData[1] << 8) | I2CData[2];
  surfRight0Average = (I2CData[3] << 8) | I2CData[4];
  surfRear0Average = (I2CData[5] << 8) | I2CData[6];    
  surfLeft1Average = (I2CData[7] << 8) | I2CData[8];    
  surfRight1Average = (I2CData[9] << 8) | I2CData[10];    
  surfRear1Average = (I2CData[11] << 8) | I2CData[12];
}

void PIC_SetSurfaceComparators(void){                 //TARGET REGISTER: 37   Update Surface Sensor Comparator thresholds. Update each variable
  regTarget = 37;                                          //                 individually in your code, then call this function to write
  I2CData[0] = highByte(PIC_LeftOuterComparatorSetpoint);  //                 all surface sensor comparator values to the PIC processor
  I2CData[1] = lowByte(PIC_LeftOuterComparatorSetpoint);
  I2CData[2] = highByte(PIC_LeftInnerComparatorSetpoint);
  I2CData[3] = lowByte(PIC_LeftInnerComparatorSetpoint);
  I2CData[4] = highByte(PIC_RightOuterComparatorSetpoint);
  I2CData[5] = lowByte(PIC_RightOuterComparatorSetpoint);
  I2CData[6] = highByte(PIC_RearInnerComparatorSetpoint);
  I2CData[7] = lowByte(PIC_RearInnerComparatorSetpoint);
  I2CData[8] = highByte(PIC_RearOuterComparatorSetpoint);
  I2CData[9] = lowByte(PIC_RearOuterComparatorSetpoint);
  I2CData[10] = highByte(PIC_RearInnerComparatorSetpoint);
  I2CData[11] = lowByte(PIC_RearInnerComparatorSetpoint);
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 12);
}

void PIC_SetAllSurfaceComparators(unsigned int setpoint){ //TARGET REGISTER: 37 (shortcut) This function updates all surface sensor comparators to
                                                          //                               the same value at the same time. Also updates local global
                                                          //                               variable values.
  PIC_LeftOuterComparatorSetpoint = setpoint;     //populate passed value into global variables
  PIC_LeftInnerComparatorSetpoint = setpoint;     //..
  PIC_RightOuterComparatorSetpoint = setpoint;    //..
  PIC_RightInnerComparatorSetpoint = setpoint;    //..
  PIC_RearOuterComparatorSetpoint = setpoint;     //..
  PIC_RearInnerComparatorSetpoint = setpoint;     //..
  PIC_SetSurfaceComparators();                    //write the new values to the PIC processor
}

void PIC_SetAmbientComparators(void){                     //TARGET REGISTER: 38   Update Ambient Sensor Comparator thresholds. Update each variable
  regTarget = 38;                                         //                      individually in your code, then call this function to write
  I2CData[0] = highByte(PIC_AmbLeftComparatorSetpoint);   //                      all ambient sensor comparator values to the PIC processor
  I2CData[1] = lowByte(PIC_AmbLeftComparatorSetpoint);
  I2CData[2] = highByte(PIC_AmbRightComparatorSetpoint);
  I2CData[3] = lowByte(PIC_AmbRightComparatorSetpoint);
  I2CData[4] = highByte(PIC_AmbRearComparatorSetpoint);
  I2CData[5] = lowByte(PIC_AmbRearComparatorSetpoint);
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 6);
}

void PIC_SetAllAmbientComparators(unsigned int setpoint){ //TARGET REGISTER: 38 (shortcut) This function updates all ambient sensor comparators to
                                                          //                               the same value at the same time. Also updates local global
                                                          //                               variable values.
  PIC_AmbLeftComparatorSetpoint = setpoint;     //populate passed value into global variables
  PIC_AmbRightComparatorSetpoint = setpoint;    //..
  PIC_AmbRearComparatorSetpoint = setpoint;     //..
  PIC_SetAmbientComparators();                  //write the new values to the PIC processor
}


/*************************************************************
*             PIEZO SOUND CHIRP REGISTERS
*************************************************************/

void playChirp(unsigned int frequency){                   //TARGET REGISTER: 41   Plays tone on piezo, frequency given in Hz. Setting 0 Hz
  regTarget = 41;                                         //                      turns off the chirp pizeo element.
  I2CData[0] = highByte(frequency);
  I2CData[1] = lowByte(frequency);
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);
}

void offChirp(void){
  playChirp(0);       //turn off chirp
}
//42

/*************************************************************
*             SERVO REGISTERS
*************************************************************/

void PIC_ServosInMotion(void){                            //TARGET REGISTER: 50   Query servos in motion
  regTarget = 50;                                         //                      
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 2);
  servosInMotion = I2CData[1];
  servoMotion_Tilt = (I2CData[1] >> 0) & 1;
  servoMotion_Pan  = (I2CData[1] >> 1) & 1;
  servoMotion_Grip = (I2CData[1] >> 2) & 1;
}

void servoSpeed(int degrPerSecond){                       //TARGET REGISTER: 51   Sets servo speed in degrees per second. Set speed to 3000
  regTarget = 51;                                         //                      allows for max rotating speed. Values beteen 20 and 90 make
  I2CData[0] = highByte(degrPerSecond);                   //                      for nice smooth movement. Speed can be updated while servo
  I2CData[1] = lowByte(degrPerSecond);                    //                      is in motion as well.
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);
}

void servoTilt(unsigned char servoValue){                 //TARGET REGISTER: 52   Sets tilt servo (up/down) setpoint. Range is 0 to 180 though limits
  servoPos_Tilt = servoValue;
  I2CData[0] = servoValue;                                //                      will prvent servo from movnig this full range.
  I2CWrite(PIC_I2C_ADDRESS, 52, 1);
}

void servoPan(unsigned char servoValue){                  //TARGET REGISTER: 53   Sets pan servo (left/right) setpoint. Range is 0 to 180 though limits
  servoPos_Pan = servoValue;
  I2CData[0] = servoValue;                                //                      will prvent servo from movnig this full range.
  I2CWrite(PIC_I2C_ADDRESS, 53, 1);
}

void servoGrip(unsigned char servoValue){                 //TARGET REGISTER: 54   Sets grip servo setpoint. Range is 0 to 180 though limits
  if (servoValue < SERVO_GRIP_STOWE){ //catch values above
    servoValue = SERVO_GRIP_STOWE;    //or below maximum
  }                                   //values set at
  if (servoValue > SERVO_GRIP_MAX_GRASP){ //top of Hardware.h
    servoValue = SERVO_GRIP_MAX_GRASP;
  }
  servoPos_Grip = servoValue;
  I2CData[0] = servoValue;                                //                      will prvent servo from movnig this full range.
  I2CWrite(PIC_I2C_ADDRESS, 54, 1);  
}    

void servoTilt_Trim(signed char servoValue){              //TARGET REGISTER: 55   Sets tilt servo trim in degrees. Accepts + and - values.
  I2CData[0] = servoValue+127; //Makes sure value is positive. We subtract 127 on the receiving side.
  I2CWrite(PIC_I2C_ADDRESS, 55, 1);  
}

void servoPan_Trim(signed char servoValue){               //TARGET REGISTER: 56   Sets pan servo trim in degrees. Accepts + and - values.
  I2CData[0] = servoValue+127; //Makes sure value is positive. We subtract 127 on the receiving side.
  I2CWrite(PIC_I2C_ADDRESS, 56, 1);  
}

void servoGrip_Trim(signed char servoValue){              //TARGET REGISTER: 57   Sets grip servo trim in degrees. Accepts + and - values.
  I2CData[0] = servoValue+127; //Makes sure value is positive. We subtract 127 on the receiving side.
  I2CWrite(PIC_I2C_ADDRESS, 57, 1);  
}
//59


/*************************************************************
*             RANGEFINDER REGISTERS
*************************************************************/

void PIC_SetRangefinderAutoInterval(unsigned char interval){     //TARGET REGISTER: 65   Set auto rangefinder interval. Count of how many 20ms cycles between
  I2CData[0] = interval;                                         //                      automatic rangefinder readings. 0 disables auto ranging. 0 is default.  
  I2CWrite(PIC_I2C_ADDRESS, 65, 1);                              //                      Auto range result can be queried by calling PIC_ReadRangefinder()
}

void PIC_RangefinderEnable(void){                                //   Turns on Rangefinder Auto Ranging, at 50 measurements per second
  PIC_SetRangefinderAutoInterval(1);                             //   Adjust sample rate by calling PIC_SetRangefinderAutoInterval()
}                                                                //   Read recent value by calling PIC_ReadRangefinder()

void PIC_RangefinderDisable(void){                               //   Turns off Rangefinder Auto Ranging
  PIC_SetRangefinderAutoInterval(0);                             //
}                                                                //

void PIC_ReadRangefinder(void){                                  //TARGET REGISTER: 66   Returns range in mm from last auto range reading. Also returns number
  regTarget = 66;                                                //                      of sequential "good counts" - readings within expected range of the
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 4);           //                      sensor
  rangeFinder = (I2CData[1] << 8) | I2CData[2];
  rangeGoodCounts = I2CData[3];
}

unsigned int PIC_ReadRangefinderCounts(void){                    //TARGET REGISTER: 67   Returns raw CCP counts from last auto range reading. Also returns number
  unsigned int counts;                                           //                      of sequential "good counts" - readings within expected range of the
  regTarget = 67;                                                //                      sensor
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 3);           
  counts = (I2CData[1] << 8) | I2CData[2];
  rangeGoodCounts = I2CData[3];
  return counts;
}

void PIC_SetRangeComparator(void){                               //TARGET REGISTER: 68   Set Rangefinder Comparator threshold. Update (Range setting
  regTarget = 68;                                                //                      in mm). Default (unless polarity reversed) tiggers comparator
  I2CData[0] = highByte(PIC_RangefinderComparatorSetpoint);      //                      when range measured is less than the setpoint. Update
  I2CData[1] = lowByte(PIC_RangefinderComparatorSetpoint);       //                      PIC_RangefinderComparatorSetpoint then call this function to
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);                       //                      write new value to PIC.
}

void PIC_SetRangeComparatorCounts(unsigned char counts){         //TARGET REGISTER: 69   Sets number of sequential 'good counts' within expected range before
  I2CData[0] = counts;                                           //                      triggering Rangefinder Comparator Threshold and Rangefinder Int0                     
  I2CWrite(PIC_I2C_ADDRESS, 69, 1);  
}

boolean PIC_RangefinderPresent(void){                            //TARGET REGISTER: 70   Query if Ultrasonic Rangefinder is present on PIC
  boolean present;                                               //                      Returns 1 if present, 0 if not present
  regTarget = 70;                                                
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 2);           
  present = I2CData[1];
  return present;
}

/*************************************************************
*             UART CONFIGURATION REGISTERS
*************************************************************/

void PIC_SetComUARTMode(unsigned char mode){                     //TARGET REGISTER: 91   The PIC UART can be set to 1 of 4 possible communication paths.
                                                                 //                      Use this function to set between them. 
  regTarget = 91;                                                //                      Mode 0 = WiFi, Mode 1 = Bluetooth, Mode 2 = XBee, Mode 3 = Pi
  I2CData[0] = mode;                                             //                      Note: Pi must be powered up for Mode 3 to be set. PIC will ignore
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 1);                       //                      request to set Mode 3 if Pi is not powered up.
}

void PIC_UART_Baud(unsigned long baud){                          //TARGET REGISTER: 92   Sets PIC UART pass through baud / serial speed
  unsigned char baudMode = 0;
  if (baud == 2400){baudMode=1;}                                 //                        baudMode = 1 = 2400 baud
  else if (baud == 9600){baudMode=2;}                            //                        baudMode = 2 = 9600 baud
  else if (baud == 19200){baudMode=3;}                           //                        baudMode = 3 = 19200 baud
  else if (baud == 57600){baudMode=4;}                           //                        baudMode = 4 = 57600 baud
  else if (baud == 115200){baudMode=5;}                          //                        baudMode = 5 = 115200 baud
  I2CData[0] = baudMode;                                                                                  
  I2CWrite(PIC_I2C_ADDRESS, 92, 1);
}

void PIC_UART_IntRxCount(unsigned char byteCount){               //TARGET REGISTER: 93   
  I2CData[0] = byteCount;                                        //                                          
  I2CWrite(PIC_I2C_ADDRESS, 93, 1);
}

void PIC_UART_IntRxChar(unsigned char nullCharacter){            //TARGET REGISTER: 94   
  I2CData[0] = nullCharacter;                                    //                                          
  I2CWrite(PIC_I2C_ADDRESS, 94, 1);
}


/*************************************************************
*          OTHER MISC CONFIGURATION REGISTERS
*************************************************************/

void PIC_SetOpMode(void){                                        //TARGET REGISTER: 100  Sets opMode registers on PIC. Set individual flags
  regTarget = 100;                                               //                      in your code, then call this function to update PIC
  if (PIC_Disable_LED_PWR){                                      //                      All values are defalut 0.
    I2CData[0] |= 1 << 0; }else{ I2CData[0] &= ~(1 << 0); } //set bit if true, otherwise, clear bit
  if (PIC_Disable_LED_PIUP){
    I2CData[0] |= 1 << 1; }else{ I2CData[0] &= ~(1 << 1); }
  if (PIC_Disable_LED_COM){
    I2CData[0] |= 1 << 2; }else{ I2CData[0] &= ~(1 << 2); }  
  if (PIC_MotorStop_Surface){
    I2CData[0] |= 1 << 3; }else{ I2CData[0] &= ~(1 << 3); }
  if (PIC_MotorStop_Range){
    I2CData[0] |= 1 << 4; }else{ I2CData[0] &= ~(1 << 4); }
  if (PIC_Polarity_Surface){
    I2CData[0] |= 1 << 5; }else{ I2CData[0] &= ~(1 << 5); }
  if (PIC_Polarity_Ambient){
    I2CData[0] |= 1 << 6; }else{ I2CData[0] &= ~(1 << 6); }
  if (PIC_Polarity_Range){
    I2CData[0] |= 1 << 7; }else{ I2CData[0] &= ~(1 << 7); }

  if (PIC_INT0_SurfaceFront){
    I2CData[1] |= 1 << 0; }else{ I2CData[1] &= ~(1 << 0); }  
  if (PIC_INT0_SurfaceRear){
    I2CData[1] |= 1 << 1; }else{ I2CData[1] &= ~(1 << 1); }
  if (PIC_INT0_Ambient){
    I2CData[1] |= 1 << 2; }else{ I2CData[1] &= ~(1 << 2); }
  if (PIC_INT0_Range){
    I2CData[1] |= 1 << 3; }else{ I2CData[1] &= ~(1 << 3); }  
  if (PIC_INT0_Power){
    I2CData[1] |= 1 << 4; }else{ I2CData[1] &= ~(1 << 4); }
  if (PIC_UART_AddNull){
    I2CData[1] |= 1 << 5; }else{ I2CData[1] &= ~(1 << 5); }
  if (PIC_INT0_UART_Rx_Count){
    I2CData[1] |= 1 << 6; }else{ I2CData[1] &= ~(1 << 6); }
  if (PIC_INT0_UART_Rx_Null){
    I2CData[1] |= 1 << 7; }else{ I2CData[1] &= ~(1 << 7); }
    
  if (PIC_UART_Passthrough){
    I2CData[2] |= 1 << 0; }else{ I2CData[2] &= ~(1 << 0); }
  if (PIC_WiFi_Enable){
    I2CData[2] |= 1 << 1; }else{ I2CData[2] &= ~(1 << 1); }
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 3);                      
}

void PIC_SetAverageIntervals(void){                              //TARGET REGISTER: 101                        
  regTarget = 101;                                               //                      Perform running average every n'th number of 20ms cycles  
  I2CData[0] = PIC_AvgIntervalSurface;                           //                      
  I2CData[1] = PIC_AvgIntervalAmbient;
  I2CData[2] = PIC_AvgIntervalPower;
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 3);                              
}

void PIC_EnablePiSignalPath(unsigned char mode){                 //TARGET REGISTER: 102  Enables(mode 1) or Disables (mode 0) Pi Signal Path
                                                                 //                      Pi must be powered on for signal path to be enabled. Pi Signal
  regTarget = 102;                                               //                      path is disabled (disconnected) when the Pi is not powered on
  I2CData[0] = mode;                                             //                      to protect the Pi's pins from over voltage when Pi is powered off.
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 1);                              
}

void PIC_SetShutdownDelay(unsigned int sdDelay, unsigned char sdIntervals){  //TARGET REGISTER: 103  Set shutdown delay (in milliseconds) PIC will wait when doing
  regTarget = 103;                                               //                      a shutdown sequence. This is the time to allow Pi to safely
  I2CData[0] = highByte(sdDelay);                                //                      shutdown before being powered off. Suggest not to change default,
  I2CData[1] = lowByte(sdDelay);                                 //                      but you may want to make it longer if your Pi is running a
  I2CData[2] = sdIntervals;                                      //                      lot of processes that need time to terminate. Max delay 20 seconds.
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 3);                       //                      sdIntervals is count of 20ms cycles BTN_PWR must be held before
                                                                 //                      a shutdown sequence is initiated.
                                                                 //                      Default sdDelay = 5000, Default sdIntervals = 40
}

void PIC_SetCurrentWarning(unsigned int setpoint){               //TARGET REGISTER: 104  Set setpoint (in milliamps) where Current Warning is asserted.
  regTarget = 104;                                               //                      We suggest not changing this, but this function is included
  I2CData[0] = highByte(setpoint);                               //                      for completeness.
  I2CData[1] = lowByte(setpoint);                                //                      
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);                              //                      
}

void PIC_SetCurrentEmgShutdown(unsigned int setpoint){           //TARGET REGISTER: 105  Set setpoint (in milliamps) emergency shutdown is initiated.
  regTarget = 105;                                               //                      We suggest not changing this, but this function is included
  I2CData[0] = highByte(setpoint);                               //                      for completeness.
  I2CData[1] = lowByte(setpoint);                                //                      
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);                              //                      
}

void PIC_SetVoltageWarning(unsigned int setpoint){               //TARGET REGISTER: 106  Set setpoint (in millivolts) where Low Voltage warning is asserted.
  regTarget = 106;                                               //                      We suggest not changing this, but this function is included
  I2CData[0] = highByte(setpoint);                               //                      for completeness.
  I2CData[1] = lowByte(setpoint);                                //                      
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);                              //                      
}

void PIC_SetVoltageEmgShutdown(unsigned int setpoint){           //TARGET REGISTER: 107  Set setpoint (in millivolts) where Low Voltage shutdown is initiated.
  regTarget = 107;                                               //                      We suggest not changing this, but this function is included
  I2CData[0] = highByte(setpoint);                               //                      for completeness.
  I2CData[1] = lowByte(setpoint);                                //                      
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);                              //                      
}

void PIC_SetWallPowerComparators(void){                          //TARGET REGISTER: 108  Set Wall Power Comparator threshold (the input voltage where
  regTarget = 108;                                               //                      rover assumes it is plugged into external wall power.
  I2CData[0] = highByte(PIC_WallPowerComparatorSetpoint);        //                      Value in millivolts. Update PIC_WallPowerComparatorSetpoint
  I2CData[1] = lowByte(PIC_WallPowerComparatorSetpoint);         //                      then call this function to write new value to PIC.
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);                              //                      
}

void PIC_SetI2CProcessDelay(unsigned char i2cDelay){             //TARGET REGISTER: 109  
  regTarget = 109;                                               //                      
  I2CData[0] = i2cDelay;                                         //                                           
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 1);                              //                      
}

void PIC_SetIntModes(unsigned char int1, unsigned char int2){    //TARGET REGISTER: 110  Sets mode for Int1 and Int2 lines
  regTarget = 110;                                               //                      
  I2CData[0] = int1;                                             //                      
  I2CData[1] = int2;                                             //                      
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);                       //
  if (int2 == 1){   //if int2 line is mode 1 (responds to BTN press)
    int2Button = 1; //set the flag                  
  }else{
    int2Button = 0; //clear the flag
  }
}

void PIC_LEDsAuto(byte leds){                                    //TARGET REGISTER: 111  Toggles PWR, PiUp, COM LEDs auto mode
  regTarget = 111;                                               //                      Pass bools set auto(1)/manual(0)
  I2CData[0] = leds;                                             //                      bit 2=PWR, 1=PiUp, 0=COM
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 1);                       //                      
}

void PIC_ArduinoButtonReset(unsigned char pwr, unsigned int btn){//TARGET REGISTER: 112  Number of 20ms cycles before Arduino is reset                      
  regTarget = 112;                                               //                      on PWR or BTN press hold. Value of 0 (default) 
  I2CData[0] = pwr;                                              //                      disables this function individually for each button.                      
  I2CData[1] = btn;
  I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);                              
}

unsigned int PIC_SoftwareVersion(void){                          //TARGET REGISTER: 120  Returns software version of the PIC processor
  unsigned int PIC_SoftwareVersion;                              //                      Use example:  version = PIC_SoftwareVersion();
  regTarget = 120;                                              
  PICReadRegs(PIC_I2C_ADDRESS, regTarget, I2CData, 3);
  PIC_SoftwareVersion = (I2CData[1] << 8) | I2CData[2];
  return PIC_SoftwareVersion;
}

// ***************************************************
// end PIC Processor Query & Write Data Functions
// ***************************************************

void motorsFromPi(void){
  motors((SPI_BufferIn[1]-128)*2, (SPI_BufferIn[2]-128)*2);  
}




    //example
    //I2CData[0] = highByte(regValue);
    //I2CData[1] = lowByte(regValue);
    //I2CWrite(PIC_I2C_ADDRESS, regTarget, 2);


// ***************************************************
// end PIC Processor Write Data Functions
// ***************************************************


// ***************************************************
// SPI Slave Functions
// ***************************************************

void SPI_Handler(void){       //add hooks to additional SPI commands to this function below
  if (SPI_Data_Received()){
    
    for (int i = 0; i<= SPI_BUFFER_LENGTH; i++){
      //  Serial.print(SPI_BufferIn[i], HEX); Serial.print(" ");  //for debugging
    }
      switch (SPI_BufferIn[0]){
        
        case 130:
          runningMode = 0;  //clear local running mode - can be used to stop local Arduino program and only respond to
                            //motor commands (or other commands) from the Pi via SPI. Assumes Pi is controlling the rover.
          motors((SPI_BufferIn[1]-128)*2, (SPI_BufferIn[2]-128)*2);
          break;
        
        case 255:     //reserved case. used to catch SPI communication error
          // Serial.println("SPI Data Error");  //for debugging  
          break;

        default:
          // Serial.println("SPI data received. No matching command.");  //for debugging
          break;
      } // end of switch (SPI_BufferIn[0])

      SPI_Enable();   //re-enable the SPI port

    } // end of if (SPI_Data_Received)
}

// ************ SPI SLAVE COMMUNICATION FUNCTIONS *************
// These functions handle incomming SPI data from an attached Raspberry Pi board

// The example below can be copied and modified to send different data back to the Raspberry Pi board
// This example zeros out the outgoing buffer
void SPI_ClearOutgoingBuffer(void){   //load SPI Outgoing buffer for next SPI transaction
  //Load zeros into all positions of SPI return data buffer. Will be sent during next SPI transaction
  SPI_BufferOut[1]= 0;    //value for array position 1
  SPI_BufferOut[2]= 0;    //value for array position 2
  SPI_BufferOut[3]= 0;    //value for array position 3
  SPI_BufferOut[4]= 0;    //value for array position 4
  SPI_BufferOut[5]= 0;    //value for array position 5
  SPI_BufferOut[6]= 0;    //value for array position 6
  SPI_BufferOut[7]= 0;    //value for array position 7
  SPI_BufferOut[8]= 0;    //value for array position 8
  SPI_BufferOut[9]= 0;    //value for array position 9
  computeSPIChecksum(0);  //compute checksum for outgoing SPI buffer & append checksum to end
}

bool SPI_Data_Received(void){
  //if SPI transaction has begun and SS line has returned high... (end of packet indication)
  if (SPI_InProgress && (digitalRead(SPI_SS)==HIGH)){
    SPI_Disable();  //disable SPI port while processing data. Must be re-enabled manually in code
                    //by calling SPI_Enable() after processing received data
    if (computeSPIChecksum(1) && (SPI_BufferIn[SPI_BUFFER_LENGTH-2] != 0)){ //checksum is good and checksum
                                                                            //byte is not zero
      return 1; //good data has been received
    }else{
      SPI_BufferIn[0] = 255;  //over-write address byte to 255, which indicates a data error has occured
      return 1;
    }
  }else{
    return 0; //no SPI packet has been received
  }
}

bool checkSPIData(void){
  if (computeSPIChecksum(1) && (SPI_BufferIn[SPI_BUFFER_LENGTH-2] != 0)){ //checksum is good and checksum
                                                                          //byte is not zero
    return 1;
  }else{
    return 0;
  }
}

void SPI_Disable(void){       //disables the slave SPI port
  SPCR = 0x00;                // turn off SPI port
  SPI.detachInterrupt();      //turn off SPI interrupt
}

void SPI_Enable(void){        //Re-enables the SPI port, but does not alter the buffer
  SPDR = 0x65;                //load outgoing SPI register for next send
  SPI_BufferPosition = 0;     //reset buffer position to the start
  SPI_InProgress = false;     //clear SPI in progress flag
  SPCR |= bit (SPE);          //turn on SPI in slave mode
  SPI.attachInterrupt();      //turn on interrupt for SPI port
}

void SPI_Reset(void){         //enables the slave SPI port, clears the outgoing SPI buffer to all zeros
  SPDR = 0x65;                //load outgoing SPI register for next send
  SPI_ClearOutgoingBuffer();  //clear the outgoing buffer
  SPI_BufferPosition = 0;     //reset buffer position to the start
  SPI_InProgress = false;     //clear SPI in progress flag
  SPCR |= bit (SPE);          //turn on SPI in slave mode
  SPI.attachInterrupt();      //turn on interrupt for SPI port
}

byte computeSPIChecksum(byte mode){//Ver. 1.0, Kevin King
  //pass mode = 0 to run checksum on SPIBuffOut (outgoing) buffer
  //pass mode = 1 to run checksum on SPIBuffIn (incoming) buffer
  //Running on incomming buffer simply returns 1 for true (if checks match) and 0 for fales (if checks don't match)
  //Running on outgoing automatically adds computed checksum to end of buffer, ready to send, and also
  //returns the computed checksum as well
  byte checksum;
  if (mode){  //mode "true" (same as mode=1). mode = 1 is "incomming" SPI buffer
              //this mode will return "true" if checksum matches
    checksum = SPI_BufferIn[0] ^ SPI_BufferIn[1] ^ SPI_BufferIn[2] ^ SPI_BufferIn[3] ^ SPI_BufferIn[4] 
        ^ SPI_BufferIn[5] ^ SPI_BufferIn[6] ^ SPI_BufferIn[7] ^ SPI_BufferIn[8];
    if (checksum == SPI_BufferIn[9]){  //if computed checksum matches the received checksum...
      return 1; //return true
    }
    else{
      return 0; //else, return false (if computed checksum does not match received checksum)
    }
  }
  else{       //mode "false" (same as mode=0). mode = 0 is "outgoing" SPI buffer
    checksum = SPI_BufferOut[1] ^ SPI_BufferOut[2] ^ SPI_BufferOut[3] ^ SPI_BufferOut[4] 
        ^ SPI_BufferOut[5] ^ SPI_BufferOut[6] ^ SPI_BufferOut[7] ^ SPI_BufferOut[8] ^ SPI_BufferOut[9];
    SPI_BufferOut[10] = checksum; //plug checksum into last byte, ready to send
    return checksum;
  }
}

// SPI interrupt routine
ISR (SPI_STC_vect)
{

  SPI_BufferIn[SPI_BufferPosition] = SPDR;        //copy incoming data byte from SPI port register
  SPDR = SPI_BufferOut[SPI_BufferPosition+1];     //copy next outgoing data byte to SPI port register
  
  SPI_InProgress = true;                          //set flag that a new packet is now in progress
  SPI_BufferPosition++;                           //set buffer index to next position of array

  if(SPI_BufferPosition > SPI_BUFFER_LENGTH-1){   //makes sure we don't over-run buffer and corrupt other data
    SPI_Disable();                                //turn off the SPI port so we don't get any more data
    SPI_BUFFER_LENGTH-1;                          //make sure subsequent bytes are written to end arry location
  }

}  // end of interrupt routine SPI_STC_vect

// ***************************************************
// end SPI Slave Functions
// ***************************************************


// ***************************************************
// Ringo I2C
// ***************************************************

#include <RingoWire.h> 
//put "RingoWireLibrary\RingoWire" into "C:\Program Files\Arduino\libraries"

//Accelerometer:
//http://cache.freescale.com/files/sensors/doc/data_sheet/MMA8451Q.pdf
//p.18
//
//Gyroscope:
//http://www.st.com/web/en/resource/technical/document/datasheet/DM00036465.pdf
//p.23-24
//
//Gyro: "Use same format for read/write single/multiple bytes"
//Accel: "The MMA8451Q automatically increments the received register address commands after a write command is received"


void I2CBegin(void){//Ver. 1.0, Dustin Soodak
  Wire.begin();
}

uint8_t I2CReadRegs(uint8_t Device, uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  uint8_t i;
  Wire.beginTransmission(Device); // transmit to device (note: actually just starts filling of buffer)
  Wire.write(Reg);
  Wire.endTransmission(0);//send data without stop at end
  Wire.requestFrom(Device, Length);
  i=0;
  while(Wire.available()){
    RxBuffer[i]=Wire.read(); 
    i++;
  }
  return i;  
}

uint8_t PICReadRegs(uint8_t Device, uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  uint8_t i;
//  while(digitalRead(I2C_Ready) == 0){
    // digitalWrite(SPI_SS, HIGH);
    // do nothing (wait for I2C_Ready to be released by PIC)
//  }
  //digitalWrite(SPI_SS, LOW);
  Wire.beginTransmission(Device); // transmit to device (note: actually just starts filling of buffer)
  Wire.write(Reg);
  Wire.endTransmission();//send data WITH stop at end
  delayMicroseconds(50);  //short delay to allow other side to process. Will replace with "wait for Ready" in production rover.
//  while(digitalRead(I2C_Ready) == 0){
    // digitalWrite(SPI_SS, HIGH);
    // do nothing (wait for I2C_Ready to be released by PIC)
//  }
  //digitalWrite(SPI_SS, LOW);
  Wire.requestFrom(Device, Length);
  i=0;
  while(Wire.available()){
    RxBuffer[i]=Wire.read(); 
    i++;
  }
  return i;  
}

uint8_t I2CReadReg(uint8_t Device, uint8_t Reg){//Ver. 1.0, Dustin Soodak
  uint8_t dat=0;
  I2CReadRegs(Device, Reg, &dat, 1);
  return dat;
}

void I2CWriteRegs(uint8_t Device, uint8_t Reg, uint8_t *TxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  char i;
  Wire.beginTransmission(Device); // transmit to device (note: actually just starts filling of buffer)
  Wire.write(Reg);                // sends one byte (the target register) 
  for(i=0;i<Length;i++){
    Wire.write(TxBuffer[i]); 
  }
  Wire.endTransmission();    //send data with stop at end 
}

void PICWriteReg(uint8_t Device, uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  while(digitalRead(I2C_Ready) == 0){
  //  digitalWrite(SPI_SS, HIGH);
  //  // do nothing (wait for I2C_Ready to be released by PIC)
  }
  //digitalWrite(SPI_SS, LOW);
  I2CWriteRegs( Device, Reg, &TxData, 1);
}

void I2CWriteReg(uint8_t Device, uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  I2CWriteRegs( Device, Reg, &TxData, 1);
}

void I2CWrite(uint8_t Device, uint8_t Reg, uint8_t Length){//Ver. 1.0, Dustin Soodak
  char i;
  Wire.begin();
  Wire.beginTransmission(Device); // transmit to device (note: actually just starts filling of buffer)
  Wire.write(Reg);              // sends one byte 
  for(i=0;i<Length;i++){
    Wire.write(I2CData[i]); 
  }
  Wire.endTransmission();    //send data with stop at end 
}

//Simple CRC Handler, provided by Pololu
//https://www.pololu.com/docs/0J44/6.7.6
const unsigned char CRC7_POLY = 0x91;
unsigned char getCRC(unsigned char message[], unsigned char length)
{
  unsigned char i, j, crc = 0;
 
  for (i = 0; i < length; i++)
  {
    crc ^= message[i];
    for (j = 0; j < 8; j++)
    {
      if (crc & 1)
        crc ^= CRC7_POLY;
      crc >>= 1;
    }
  }
  return crc;
}

// ***************************************************
// end Ringo I2C
// ***************************************************

