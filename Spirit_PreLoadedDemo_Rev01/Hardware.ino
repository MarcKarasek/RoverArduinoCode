/*

Spirit Robot:  Hardware  Rev01.01  08/2017

Significant portions of this code written by
Dustin Soodak for Plum Geek LLC. Some portions
contributed by Kevin King.
Portions from other open source projects where noted.
This code is licensed under:
Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
https://creativecommons.org/licenses/by-sa/2.0/

Visit http://www.plumgeek.com for Ringo information.
Visit http://www.arduino.cc to learn about the Arduino.

*/

#include "Hardware.h"
#include "Navigation.h"
#include "Comms.h"
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

volatile signed char  servoTrim_Tilt  =25  ;     //Tilt servo trim (-127 to +127)
volatile signed char  servoTrim_Pan   =0   ;     //Pan servo trim (-127 to +127)
volatile signed char  servoTrim_Grip  =-1  ;     //Grip servo trim (-127 to +127)


// ***************************************************
// Ringo Hardware
// ***************************************************

// ***************************************************
// General Global Variables
// ***************************************************

volatile unsigned char debug[4];            //holder for general debug values
volatile int ambLeft=0;                     //Updated by calling PIC_ReadAllAmbientSensors()
volatile int ambRight=0;                    //Updated by calling PIC_ReadAllAmbientSensors()
volatile int ambRear=0;                     //Updated by calling PIC_ReadAllAmbientSensors()
volatile int ambLeftAverage=0;
volatile int ambRightAverage=0;
volatile int ambRearAverage=0;
volatile int surfLeft0=0;                   //Updated by calling PIC_ReadAllSurfaceSensors()
volatile int surfLeft1=0;                   //Updated by calling PIC_ReadAllSurfaceSensors()
volatile int surfRight0=0;                  //Updated by calling PIC_ReadAllSurfaceSensors()
volatile int surfRight1=0;                  //Updated by calling PIC_ReadAllSurfaceSensors()
volatile int surfRear0=0;                   //Updated by calling PIC_ReadAllSurfaceSensors()
volatile int surfRear1=0;                   //Updated by calling PIC_ReadAllSurfaceSensors()
volatile int surfLeft0Average=0;
volatile int surfLeft1Average=0;
volatile int surfRight0Average=0;
volatile int surfRight1Average=0;
volatile int surfRear0Average=0;
volatile int surfRear1Average=0;
volatile int powerCurrent=0;                //Current into power supply in milliamps
volatile int powerVoltage=0;                //Power supply/battery voltage in millivolts
volatile int servoCurrent=0;                //Estimated current into servos after servo movement
volatile int powerCurrentAverage=0;
volatile int powerVoltageAverage=0;
volatile int rangeFinder=1000;              //Present range in mm
volatile unsigned char rangeGoodCounts=0;   //Count of sequential Rangefinder readings that have been within acceptable range (maxes out at 250)
volatile signed char  servoPos_Tilt=0;      //Tilt servo position (-127 to +127)
volatile signed char  servoPos_Pan=0;       //Pan servo position (-127 to +127)
volatile signed char  servoPos_Grip=0;      //Grip servo position (-127 to +127)
volatile boolean      servoMotion_Tilt=0;   //Tilt servo in motion
volatile boolean      servoMotion_Pan=0;    //Pan servo in motion
volatile boolean      servoMotion_Grip=0;   //Grip servo in motion
volatile unsigned char chargeStatus=0;      //0=No Chg Present, Not Charging; 1=Chg Present, Not Charging; 2= Chg Present, Charging in Progress
volatile unsigned char servosInMotion=0;    //Bool flags of the above. Indicagtes at least one servo has not yet arrived at target position
volatile boolean      int2Button=0;         //if the int2 line is set to respond to button press (used in buttonPressed handler)
volatile byte         runningMode = 1;      //can be used to denote local Arduino control (1) or Pi control (0), or other uses in user code
volatile unsigned char crc=0;               //crc used in I2C packet
volatile unsigned long timeStamp;           //used to hold millis() results

volatile unsigned char PIC_StatusBank0           =0;  //Updated by calling PIC_ReadStatus(), contains bool status values, assigned to individual flags below
volatile unsigned char PIC_StatusBank1           =0;  //Updated by calling PIC_ReadStatus(), contains bool status values, assigned to individual flags below
volatile boolean       PIC_CurrentWarningInt     =0;  //Updated by calling PIC_ReadStatus(), flag that over current warning has been set
volatile boolean       PIC_VoltageWarningInt     =0;  //Updated by calling PIC_ReadStatus(), flag that under voltage (low battery) warning has been set
volatile boolean       PIC_ShutdownNowInt        =0;  //Updated by calling PIC_ReadStatus(), flag that imminent shutdown is in progress (issue warning and shutdown Pi immeidately)
volatile boolean       PIC_MotorStopInt          =0;  //Updated by calling PIC_ReadStatus(), flag that motors should be stopped immediately
volatile boolean       PIC_SurfaceSenseInt       =0;  //Updated by calling PIC_ReadStatus(), flag that a surface sensor has crossed a setpoint
volatile boolean       PIC_PowerSenseInt         =0;  //Updated by calling PIC_ReadStatus(), flag that a power sensor (current or voltage) has crossed a setpoint  
volatile boolean       PIC_RangeSenseInt         =0;  //Updated by calling PIC_ReadStatus(), flag that a rangefinder measurement has crossed a setpoint
volatile boolean       PIC_AmbSenseInt           =0;  //Updated by calling PIC_ReadStatus(), flag that an ambient sensor has crossed a setpoint
volatile boolean       PIC_UARTRxThreshInt       =0;  //Updated by calling PIC_ReadStatus(), UART Rx Buffer byte count over threshold
volatile boolean       PIC_UARTRxNullInt         =0;  //Updated by calling PIC_ReadStatus(), UART Rx Buffer has received specified null character
volatile boolean       PIC_UARTTxInProg          =0;  //Updated by calling PIC_ReadStatus(), UART Tx Buffer byte count not zero (UART Tx in progress)


volatile unsigned char PIC_thresholdComparators1 =0;  //Bool flags of whether 8 different metrics are above or below a given setpoint
volatile boolean       PIC_CurrentComparator     =0;  //bit 0 - PIC_CurrentComparator
volatile boolean       PIC_VoltageComparator     =0;  //bit 1 - PIC_VoltageComparator
volatile boolean       PIC_LeftOuterComparator   =0;  //bit 2 - PIC_LeftOuterComparator
volatile boolean       PIC_LeftInnerComparator   =0;  //bit 3 - PIC_LeftInnerComparator
volatile boolean       PIC_RightOuterComparator  =0;  //bit 4 - PIC_RightOuterComparator
volatile boolean       PIC_RightInnerComparator  =0;  //bit 5 - PIC_RightInnerComparator
volatile boolean       PIC_RearOuterComparator   =0;  //bit 6 - PIC_RearOuterComparator
volatile boolean       PIC_RearInnerComparator   =0;  //bit 7 - PIC_RearInnerComparator

volatile unsigned char PIC_thresholdComparators2 =0;  //Bool flags of whether 8 different metrics are above or below a given setpoint
volatile boolean       PIC_RangefinderComparator =0;  //bit 0 - PIC_RangefinderComparator
volatile boolean       PIC_AmbLeftComparator     =0;  //bit 1 - PIC_AmbLeftComparator
volatile boolean       PIC_AmbRightComparator    =0;  //bit 2 - PIC_AmbRightComparator
volatile boolean       PIC_AmbRearComparator     =0;  //bit 3 - PIC_AmbRightComparator
volatile boolean       PIC_WallPower             =0;  //bit 4 - PIC_WallPower

volatile unsigned char PIC_AvgIntervalSurface    =6;  //
volatile unsigned char PIC_AvgIntervalAmbient    =6;  //
volatile unsigned char PIC_AvgIntervalPower      =6;  //


////////////////////////////////////////////////////////
// OpMode Registers. Update values, then call PIC_SetOpMode() to write values to the PIC.
////////////////////////////////////////////////////////
volatile boolean       PIC_Disable_LED_PWR       =0;  //Disables Pwr LED even while robot is powered on
volatile boolean       PIC_Disable_LED_PIUP      =0;  //Disables PiUp LED
volatile boolean       PIC_Disable_LED_COM       =0;  //Disables Com LED
volatile boolean       PIC_MotorStop_Surface     =0;  //Issues interrupt to stop motors if a surface sensor crosses a threshold
volatile boolean       PIC_MotorStop_Range       =0;  //Issues interrupt to stop motors if rangefinder reading crosses a threshold
volatile boolean       PIC_Polarity_Surface      =0;  //Reverses polarity of surface sensor comparators (so they trigger when *above* the setpoint rather than below)
volatile boolean       PIC_Polarity_Ambient      =0;  //Reverses polarity of ambient sensor comparators (so they trigger when *below* the setpoint rather than above)
volatile boolean       PIC_Polarity_Range        =0;  //Reverses polarity of range sensor comparators (so it triggers when *further* from the setpoint rather than closer)
volatile boolean       PIC_INT0_SurfaceFront     =0;  //Causes PIC to pull INT0 line when either front surface sensor crosses the comparator threshold
volatile boolean       PIC_INT0_SurfaceRear      =0;  //Causes PIC to pull INT0 line when either rear surface sensor crosses the comparator threshold
volatile boolean       PIC_INT0_Ambient          =0;  //Causes PIC to pull INT0 line when either ambient sensor crosses the comparator threshold
volatile boolean       PIC_INT0_Range            =0;  //Causes PIC to pull INT0 line when the rangefinder sensor crosses the comparator threshold
volatile boolean       PIC_INT0_Power            =0;  //Causes PIC to pull INT0 line when either voltage or current sensor crosses the comparator threshold
volatile boolean       PIC_UART_AddNull          =0;  //When set (1), adds null character (0x00) to end of UART->i2c pass through (not set / no null is default)
volatile boolean       PIC_INT0_UART_Rx_Count    =0;  //Causes PIC to pull INT0 line when PIC UART Rx buffer exceeds certain threshold of received bytes
volatile boolean       PIC_INT0_UART_Rx_Null     =0;  //Causes PIC to pull INT0 line when PIC UART Rx receives a designated Null Character
volatile boolean       PIC_UART_Passthrough      =0;  //Causes PIC to relay UART from Comm modules to external UART port
volatile boolean       PIC_WiFi_Enable           =0;  //Enables optional attached ESP8266 WiFi Module (1= enabled, 0= disabled)

////////////////////////////////////////////////////////////
// Threshold Comparator Setpoint Values
////////////////////////////////////////////////////////////
volatile unsigned int  PIC_CurrentComparatorSetpoint      =2500;    //Setpoint. Update this value, then write it to the PIC by calling PIC_SetPowerComparators()      
volatile unsigned int  PIC_VoltageComparatorSetpoint      =3000;    //Setpoint. Update this value, then write it to the PIC by calling PIC_SetPowerComparators()  
volatile unsigned int  PIC_LeftOuterComparatorSetpoint    =0;       //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()    
volatile unsigned int  PIC_LeftInnerComparatorSetpoint    =0;       //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()    
volatile unsigned int  PIC_RightOuterComparatorSetpoint   =0;       //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()   
volatile unsigned int  PIC_RightInnerComparatorSetpoint   =0;       //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()  
volatile unsigned int  PIC_RearOuterComparatorSetpoint    =0;       //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()  
volatile unsigned int  PIC_RearInnerComparatorSetpoint    =0;       //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators() 
volatile unsigned int  PIC_RangefinderComparatorSetpoint  =0;       //Setpoint. Write new value to PIC by calling PIC_SetRangeComparator()
volatile unsigned int  PIC_AmbLeftComparatorSetpoint      =1023;    //Setpoint. Update this value, then write it to the PIC by calling PIC_SetAmbientComparators()
volatile unsigned int  PIC_AmbRightComparatorSetpoint     =1023;    //Setpoint. Update this value, then write it to the PIC by calling PIC_SetAmbientComparators()
volatile unsigned int  PIC_AmbRearComparatorSetpoint      =1023;    //Setpoint. Update this value, then write it to the PIC by calling PIC_SetAmbientComparators()
volatile unsigned int  PIC_WallPowerComparatorSetpoint    =4400;    //Setpoint. Update this value, then write it to the PIC by calling PIC_SetWallPowerComparators()

volatile unsigned char PIC_PCON_Register;    //Copy of PIC PCON register immediatley following the most recent reset
volatile unsigned char PIC_STATUS_Register;  //Copy of PIC STATUS register immediatley following the most recent reset (bit4 is itneresting, 0= PIC WDT reset occured)
volatile unsigned char PIC_UART_RxBufCount;  //Number of bytes presently in PIC UART receive buffer
volatile unsigned char PIC_UART_TxBufCount;  //Number of bytes presently in PIC UART transmit buffer

volatile unsigned char PIC_PORTA=0;       //populated by calling PIC_ReadInputPins()
volatile unsigned char PIC_PORTB=0;       //populated by calling PIC_ReadInputPins()
volatile unsigned char PIC_PORTC=0;       //populated by calling PIC_ReadInputPins()
volatile unsigned char PIC_PORTD=0;       //populated by calling PIC_ReadInputPins()
volatile unsigned char PIC_PORTE=0;       //populated by calling PIC_ReadInputPins()
volatile unsigned char PIC_PORTF=0;       //populated by calling PIC_ReadInputPins()
volatile unsigned char PIC_PORTG=0;       //populated by calling PIC_ReadInputPins()
volatile boolean       PIC_Button=0;      //populated by calling PIC_ReadInputPins()
volatile boolean       PIC_Button_Pwr=0;  //populated by calling PIC_ReadInputPins()
volatile boolean       PIC_Accel_Int1=0;  //populated by calling PIC_ReadInputPins()
volatile boolean       PIC_Gyro_Int1=0;   //populated by calling PIC_ReadInputPins()
volatile boolean       PIC_Gyro_Int2=0;   //populated by calling PIC_ReadInputPins()
volatile boolean       PIC_38kHzRx=0;     //populated by calling PIC_ReadInputPins()
volatile boolean       PIC_ChgPresent=0;  //populated by calling PIC_ReadInputPins()
volatile boolean       PIC_ChgInProg=0;   //populated by calling PIC_ReadInputPins()
volatile boolean       PIC_XBee_Assoc=0;  //populated by calling PIC_ReadInputPins()
volatile boolean       PIC_Range_Echo=0;  //populated by calling PIC_ReadInputPins()

volatile unsigned char PIC_LATA=0;        //populated by calling PIC_ReadOutputPins()
volatile unsigned char PIC_LATB=0;        //populated by calling PIC_ReadOutputPins()
volatile unsigned char PIC_LATC=0;        //populated by calling PIC_ReadOutputPins()
volatile unsigned char PIC_LATD=0;        //populated by calling PIC_ReadOutputPins()
volatile unsigned char PIC_LATE=0;        //populated by calling PIC_ReadOutputPins()
volatile unsigned char PIC_LATF=0;        //populated by calling PIC_ReadOutputPins()
volatile unsigned char PIC_LATG=0;        //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_PS5V0_EN;      //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_PS3V3_EN;      //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_LED_PWR;       //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_LED_PIUP;      //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_LED_COM;       //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_MUXA;          //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_MUXB;          //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_PI_SIG_EN;     //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_BATT_SW;       //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_WIFI_PD;       //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_PIEZO;         //populated by calling PIC_ReadOutputPins()
volatile boolean       PIC_RANGE_TRIG;    //populated by calling PIC_ReadOutputPins()





// ***************************************************
// end General Global Variables
// ***************************************************


void hardwareBegin(void){//Ver. 1.0, Dustin Soodak

  pinMode(Pixel_Bus,OUTPUT);      //prepare pixel bus
  digitalWrite(Pixel_Bus,LOW);    //prepare pixel bus
  setAllPixelsRGB(0,0,0);         //turn off all pixels
  
  pinMode(SPI_SS,INPUT);
  pinMode(SPI_MOSI,INPUT);
  pinMode(SPI_SCK,INPUT);
  pinMode(SPI_MISO,OUTPUT);
  digitalWrite(SPI_MISO,LOW);
  pinMode(_38kHz_Rx,INPUT_PULLUP);
  
  pinMode(I2C_Ready,INPUT);
  
  Serial.begin(SERIAL_SPEED);               // startup normal serial port. Default data rate is 57600
  
  digitalWrite(Pixel_Bus,LOW);
  pinMode(Pixel_Bus,OUTPUT);
  
  I2CBegin();                               // startup I2C
  GyroWriteRegister(GYR_CTRL_REG3,0x10);    // turn off gyro open drain (drains current via int line from accel if not set)
  GyroWriteRegister(GYR_CTRL_REG1,0x0f);    // place gyro in power down mode (saves 6mA of current)
                                            // call NavigationBegin(); to wake up and configure Gyro and Accelerometer  
  
  setServoTrim();                           // Set servo trims
  GetGyroCalibrationMultiplier();           // Set gyro calibration
  offChirp();                               // Make sure chirp is off (in case it was stuck on from a previous program)
  
  PIC_ReadPower();                          // get power values during startup
  
  //delay(50);   //leave this in so power sequence happens properly on PiBot board
  timeStamp = millis();   //get starting timestamp  
  restartTimer();
}



void gripStow(void){    //move grip servo to "stowed" position
  servoGrip(10);  
}

void gripReady(void){    //move grip servo to "ready" position
  servoGrip(90);  
}

boolean gripGrasp(int currentThreshold){
  constrain(currentThreshold,0,250);  //make sure passed threshold is within expected range
  boolean objectFound = 1;  //flag if we found an object
  delay(21);                //make sure we're in subsequent PIC cycle interval
  PIC_ServosInMotion();     //get servos in motion
  while(servosInMotion){    //wait for any moving servos to arrive at final setpoints
    PIC_ServosInMotion();
    delay(20);
  }
  PIC_ReadServoCurrent();   //read starting servo current
  while(servoCurrent < currentThreshold){ //close servo until threshold exceeded
    servoGrip(servoPos_Grip+2);
    delay(20);              //wait for next PIC cycle
    PIC_ReadServoCurrent();
    //Serial.println(servoCurrent); //for debugging
    if(servoPos_Grip > 125){  //break if we exceed max travel of servo (no object found)
      objectFound = 0;
      break;
    }
  }
  servoGrip(servoPos_Grip-5); //slack servo a bit
  return objectFound;
}

void setServoTrim(void){  //reads servo trims from EEPROM, updates EEPROM if values don't match
                          //sets servo trim values to PIC processor
  signed char eeprom_tilt, eeprom_pan, eeprom_grip;                        
  EEPROM_readAnything(1017, eeprom_tilt);
  EEPROM_readAnything(1018, eeprom_pan);
  EEPROM_readAnything(1019, eeprom_grip);
  //Serial.print for debugging...
  //Serial.print(eeprom_tilt); Serial.print("\t"); Serial.print(eeprom_pan); Serial.print("\t"); Serial.println(eeprom_grip);
  
  if (servoTrim_Tilt == 0){         //if the trim value in the code above is zero...
     servoTrim_Tilt = eeprom_tilt;  //use the value in EEPROM (unwritten eeprom defaults to -1, which is basically zero for this purpose)
     //Serial.print("Tilt using "); Serial.println(eeprom_tilt);    
  }
  else if (eeprom_tilt != servoTrim_Tilt){           //if the trim value in the code above is different from the value already in eeprom...
      EEPROM_writeAnything(1017, servoTrim_Tilt); //then update the eeprom to the value from the code above
      //Serial.print("Tilt updated from "); Serial.print(eeprom_tilt); Serial.print(" to "); Serial.println(servoTrim_Tilt);
  }
  
  if (servoTrim_Pan == 0){         //if the trim value in the code above is zero...
     servoTrim_Pan = eeprom_pan;  //use the value in EEPROM (unwritten eeprom defaults to -1, which is basically zero for this purpose)     
     //Serial.print("Pan using "); Serial.println(eeprom_pan);
  }
  else if (eeprom_pan != servoTrim_Pan){           //if the trim value in the code above is different from the value already in eeprom...
      EEPROM_writeAnything(1018, servoTrim_Pan); //then update the eeprom to the value from the code above
      //Serial.print("Pan updated from "); Serial.print(eeprom_pan); Serial.print(" to "); Serial.println(servoTrim_Pan);
  }
  
  if (servoTrim_Grip == 0){         //if the trim value in the code above is zero...
     servoTrim_Grip = eeprom_grip;  //use the value in EEPROM (unwritten eeprom defaults to -1, which is basically zero for this purpose)     
     //Serial.print("Grip using "); Serial.println(eeprom_grip);
  }
  else if (eeprom_grip != servoTrim_Grip){           //if the trim value in the code above is different from the value already in eeprom...
      EEPROM_writeAnything(1019, servoTrim_Grip); //then update the eeprom to the value from the code above
      //Serial.print("Grip updated from "); Serial.print(eeprom_grip); Serial.print(" to "); Serial.println(servoTrim_Grip);
  }
  
  servoTilt_Trim(servoTrim_Tilt);   // send trim values for servos
  servoPan_Trim(servoTrim_Pan);     // send trim values for servos
  servoGrip_Trim(servoTrim_Grip);   // send trim values for servos

}

void servoCenters(void){
  servoSpeed(SERVO_SPEED_DEFAULT);    //these values can be adjusted at the top of Hardware.h file
  servoTilt(SERVO_TILT_CENTER);       //these values can be adjusted at the top of Hardware.h file
  servoPan(SERVO_PAN_CENTER);         //these values can be adjusted at the top of Hardware.h file
  servoGrip(SERVO_GRIP_STOWE);        //these values can be adjusted at the top of Hardware.h file
  //Note: The Rover assembly process uses servoGrip(110) to set servo in position for assembly
  //of the servo arms.
}


boolean buttonPressed(void){//Ver. 1.0, Dustin Soodak
  boolean pressed=0;
  if(!int2Button){          //if INT2 line not already set to respond to button press...
    PIC_SetIntModes(0,1);   //set INT2 line to respond to button press
    delay(5);               //short delay
  }
  if (digitalRead(INT2) == 0){ //button is being pressed
    pressed = 1;
  }else{
    pressed = 0;
  }
  return pressed;
}


void MotorsBegin(void){//Ver. 1.0, Dustin Soodak
  LeftMotor=0;
  RightMotor=0;
  pinMode(MotorDirection_Left, OUTPUT);
  pinMode(MotorDirection_Right, OUTPUT);
  analogWrite(MotorDrive_Left,0);
  analogWrite(MotorDrive_Right,0);  

}


int LeftMotor;
int RightMotor;
void motors(int LeftMotorSpeed, int RightMotorSpeed){//Ver. 1.0, Dustin Soodak
  if(LeftMotorSpeed>MOTOR_MAX)
    LeftMotorSpeed=MOTOR_MAX;
  if(LeftMotorSpeed<-MOTOR_MAX)
    LeftMotorSpeed=-MOTOR_MAX;
  if(RightMotorSpeed>MOTOR_MAX)
    RightMotorSpeed=MOTOR_MAX;
  if(RightMotorSpeed<-MOTOR_MAX)
    RightMotorSpeed=-MOTOR_MAX;
  LeftMotor=LeftMotorSpeed;
  RightMotor=RightMotorSpeed;
  if(LeftMotor<0){
    digitalWrite(MotorDirection_Left,0); //0
    //Serial.print("left - ");
  }
  else
    digitalWrite(MotorDirection_Left,1); //1
  if(RightMotor<0){
    digitalWrite(MotorDirection_Right,1); //0
    //Serial.print("right - ");
  }
  else
    digitalWrite(MotorDirection_Right,0); //1
    
  analogWrite(MotorDrive_Left,abs(LeftMotor));
  analogWrite(MotorDrive_Right,abs(RightMotor));
}

void Motors(int LeftMotorSpeed, int RightMotorSpeed){//Ver. 1.0, Dustin Soodak
  motors(LeftMotorSpeed,RightMotorSpeed);
}

void initializeRandom(void){
  PIC_ReadAllAmbientSensors();  //get ambient sensor readings
  randomSeed(ambRear);          //seed the random number generator based on light level to rear sensor, which is somewhat random
}

void TxIR(unsigned char *Data, int Length){//Ver. 1.2, Dustin Soodak
    int i;
    char j;
    const uint16_t Freq=38000,UsOn=5; //For R2=2k pull up, 8 us delay before pin falls. Inputs (28000,5) give a decent square wave in this case. 
    RxIRStop();
    EnableIROutputs(1);
    ModulateIR(Freq,UsOn);
    delayMicroseconds(9000);
    EnableIROutputs(0);               //EnableIROutputs(0) turns off IR 38khz out but this makes receiver voltage go high.
    delayMicroseconds(4500);
    EnableIROutputs(1);
    delayMicroseconds(520);
    for(i=0;i<Length;i++){      
      for(j=0;j<8;j++){
        EnableIROutputs(0);
        if((Data[i])&(1<<j))
          delayMicroseconds(1610);
        else
          delayMicroseconds(580);
        EnableIROutputs(1);
        delayMicroseconds(520);
      }      
    }//end for(i=0;i<Length;i++)    
    ModulateIR(38000, 0);
    EnableIROutputs(1); 
}//end TxIR()

void TxIRKey(byte key){//Ver. 1.0, Kevin King
  if(key<1){
    // do nothing
  }
  else if(key>21){
    // do nothing
  }
  else{     // actually send IR key if it was within correct range
  key-=1;   //subract 1 from key number so it matches array IRRemoteButtons
  irData[0]=0x00;irData[1]=0xFF;     // all remote keys begin with 0x00, 0xFF
  irData[2]=IRRemoteButtons[key][0]; // add 3rd value
  irData[3]=IRRemoteButtons[key][1]; // add 4th value
  TxIR(irData,4);                    // actually transmit via any enabled IR sources
  }
}//end TxIRKey()

int IRTransitionCount=0;
char IRBitNum=0,IRByte=0,IRNumOfBytes=0;
unsigned char IRBytes[20];
uint16_t IRPrevTime=0,IRTime;
uint32_t MsAtLastIR=0;//used for end of communication timeout
volatile char IRReceiving=0;
char IRActive=0;
char IRMaxByteCount=4;


void RxIRRestart(char BytesToLookFor){//Ver. 1.2, Dustin Soodak
  int i;
  detachInterrupt(0);//interrupt 0 is I/O 2 which is _38kHz_R
  TCCR1A = 0x00;          // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1 -- PWM11=0,PWM10=0 => PWM Operation disabled
  // ICNC1=0 => Capture Noise Canceler disabled -- ICES1=0 => Input Capture Edge Select (not used) -- CTC1=0 => Clear Timer/Counter 1 on Compare/Match
  // CS12=0 CS11=1 CS10=1 => Set prescaler to clock/64
  TCCR1B = 0x03;          // 8MHz clock with prescaler 0x03 means TCNT1 increments every 8uS
  // ICIE1=0 => Timer/Counter 1, Input Capture Interrupt Enable -- OCIE1A=0 => Output Compare A Match Interrupt Enable -- OCIE1B=0 => Output Compare B Match Interrupt Enable
  // TOIE1=0 => Timer 1 Overflow Interrupt Enable
  TIMSK1 = 0x00;          
  pinMode(_38kHz_Rx, INPUT_PULLUP);  
  IRReceiving=0;  
  IRTransitionCount=0;
  IRPrevTime=0;
  MsAtLastIR=0;IRBitNum=0;IRByte=0;IRNumOfBytes=0;
  IRActive=1;
  IRMaxByteCount=BytesToLookFor;
  attachInterrupt(0, IRHandler, CHANGE);//interrupt 0 is I/O 2 which is _38kHz_Rx  
}
void RxIRStop(void){//Ver. 1.2, Dustin Soodak
  detachInterrupt(0);//interrupt 0 is I/O 2 which is _38kHz_Rx    
  TCCR1B=0;//turn off timer        
  pinMode(_38kHz_Rx, INPUT_PULLUP);  
  
  MsAtLastIR=0;//so IRHandler() recognizes it as first falling edge of next transition
  IRReceiving=0;
  IRActive=0;
  IRTransitionCount=0;//so IsIRDone() does not expect anything just because RxIRStop() called.
}

// Infrared transmit and recieve functions are based on:
//http://playground.arduino.cc/Code/InfraredReceivers by Paul Malmsten 
//https://github.com/z3t0/Arduino-IRremote by Ken Shirriff

void IRHandler(void){//Ver. 2.0, Dustin Soodak, Kevin King
  //using interrupt 1 is I/O 3 which is _38kHz_Rx  
  int16_t dTime;
  char Level;
  noInterrupts();
  IRTime=TCNT1;
   
  Level=digitalRead(_38kHz_Rx);  

  if(!Level){//note; 38khz IR signal makes level go low
    IRReceiving=1;
  }
  if((millis()-MsAtLastIR>15) && (IRNumOfBytes<IRMaxByteCount)) {//should never be more than 9 inside valid signal
    TCNT1=0;
    IRPrevTime=0;
    IRTransitionCount=0;
    IRBitNum=0;IRNumOfBytes=0;IRByte=0;  
    //IRReceiving=1;  
  }  
  else{  
    if(IRTime>IRPrevTime)
      dTime=IRTime-IRPrevTime;
    else
      dTime=0xFFFF-IRPrevTime+1+IRTime;   
    IRPrevTime=IRTime;    
    dTime=dTime<<3;
    if(IRTransitionCount>=3 && (IRTransitionCount&1)){//should be high
      if(dTime>1000){
        IRByte|=(1<<IRBitNum); 
      }
      if(dTime<300){//error
         IRNumOfBytes=0;
         IRReceiving=0;
      }
      IRBitNum++;
      if(IRBitNum>7){
        if(IRNumOfBytes<IRMaxByteCount){
          IRBytes[IRNumOfBytes]=IRByte;
          IRNumOfBytes++;
        }        
        else{
           IRReceiving=0;           
        }
        IRBitNum=0; 
        IRByte=0;               
      }      
    }
    IRTransitionCount++;
  }
  MsAtLastIR=millis();
  interrupts();
}

char IsIRDone(void){//Ver. 1.0, Dustin Soodak
  return ((millis()-MsAtLastIR>40 || IRNumOfBytes==IRMaxByteCount) && IRTransitionCount);  
}

//Note: first and second bytes are always 0x00 and 0xFF. IRRemoteButtons[] contains the third and fourth bytes.
const byte IRRemoteButtons[][2]={
  {0x0C,0xF3},//"1" key: 1
  {0x18,0xE7},//"2" key: 2
  {0x5E,0xA1},//"3" key: 3
  {0x08,0xF7},//"4" key: 4
  {0x1C,0xE3},//"5" key: 5
  {0x5A,0xA5},//"6" key: 6
  {0x42,0xBD},//"7" key: 7
  {0x52,0xAD},//"8" key: 8
  {0x4A,0xB5},//"9" key: 9
  {0x16,0xE9},//"0" key: 10
  {0x40,0xBF},//"FORWARD" key: 11
  {0x07,0xF8},//"LEFT" key: 12
  {0x09,0xF6},//"RIGHT" key: 13
  {0x19,0xE6},//"BACKWARD" key: 14
  {0x45,0xBA},//"POWER" key: 15
  {0x46,0xB9},//"PLUM LOGO" key: 16
  {0x47,0xB8},//"MENU" key: 17
  {0x44,0xBB},//"A" key: 18
  {0x43,0xBC},//"B" key: 19
  {0x15,0xEA},//"PLAY" key: 20
  {0x0D,0xF2}//"X" key: 21
};
byte GetIRButton(void){//Ver. 2.0, Dustin Soodak, Kevin King
  byte ButtonNumber=0,i;

    for(i=0;i<sizeof(IRRemoteButtons)/2;i++){
      if(IRBytes[2]==IRRemoteButtons[i][0] && IRBytes[3]==IRRemoteButtons[i][1]){
        ButtonNumber=i+1;
        break; 
      }
    } 

    return ButtonNumber;       //return the button number that was pressed
    
}// end GetIRButton()


// ***************************************************
// end Ringo Hardware
// ***************************************************

// ***************************************************
// Pixels
// ***************************************************

// These functions use the Adafruit NeePixel Libraray https://github.com/adafruit/Adafruit_NeoPixel

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS, Pixel_Bus, NEO_GRB + NEO_KHZ800);
void setPixelRGB(int Pixel, int Red, int Green, int Blue){//Ver. 1.0, Dustin Soodak
  if(Pixel>NUM_PIXELS)
    Pixel=NUM_PIXELS;
  if(Pixel<0)
    Pixel=0;
  pixels.setPixelColor(Pixel, pixels.Color(Red,Green,Blue));
  pixels.show();//Remove this line and use RefreshPixels() below 
  //              if you want to set several at once for high-speed patterns.
}
void setAllPixelsRGB(int Red, int Green, int Blue){//Ver. 1.1, Dustin Soodak
  char i;
  for(i=0;i<NUM_PIXELS;i++){
    pixels.setPixelColor(i, pixels.Color(Red,Green,Blue)); 
  }
  pixels.show();//added Ver.1.1
}
void RefreshPixels(void){//Ver. 1.0, Dustin Soodak
  pixels.show();  
}

void eyesOff(void){                           //turns eyes off
  setPixelRGB(1,0,0,0);                       //turn off left eye
  setPixelRGB(2,0,0,0);                       //turn off right eye
}

void eyesOn(int red, int green, int blue){    //turns both eyes on same color
  setPixelRGB(1,red,green,blue);              //left eye
  setPixelRGB(2,red,green,blue);              //right eye
}

void leftEye(int red, int green, int blue){
  setPixelRGB(1,red,green,blue);              //control left eye
}

void rightEye(int red, int green, int blue){
  setPixelRGB(2,red,green,blue);              //control right eye
}

void leftWing(int red, int green, int blue){  //set entire wing to the same color
  for (char i=3; i<15; i++){
    setPixelRGB(i,red,green,blue);
  }
}

void rightWing(int red, int green, int blue){ //set entire wing to same color
  for (char i=15; i<27; i++){
    setPixelRGB(i,red,green,blue);
  }
}



// ***************************************************
// end Pixels
// ***************************************************


// ***************************************************
// MovementFunctions
// ***************************************************

//
//MaintainHeading:
//
//This is a simplified PID (proportiona-Integral-Derivative) function for
//maintaining your present heading. A complete example can be found below
//the MaintainHeading(degrees,speed,wiggle) function.
//
int MaintTainHeadingOffsetDir=1;
int MaintainHeadingIntegral=0;
uint32_t MaintainHeadingPrevTimeUs;

void MaintainHeadingReset(){//Ver. 1.0, Dustin Soodak
  MaintainHeadingIntegral=0;
  MaintainHeadingPrevTimeUs=micros();
}

char MaintainHeading(int Heading, int Speed, int Wiggle){//Ver. 1.0, Dustin Soodak
  int Input,Output,Proportional,left,right;
  char ret=0;
  float dt;
  if(Wiggle>0){
    if(MaintTainHeadingOffsetDir>0){
      if(GetDegrees()>=Heading+Wiggle)
        MaintTainHeadingOffsetDir=-1;    
    }
    else{
      if(GetDegrees()<=Heading-Wiggle)
        MaintTainHeadingOffsetDir=1;
    }
  }  
  Input=GetDegrees()+GetDegreesToStop();
  Proportional=(Heading+MaintTainHeadingOffsetDir*Wiggle-Input);//make it try to turn to the wiggle value
  //Note: "MaintainHeadingAverageDerivative" no longer needs to be needed. Code left here in case necessary for modded bot.
  //TimeAdjustedAverage=NewValue*(dt/totalT)+AverageValue*((totalT-dt)/totalT)
  //In this case, we are doing a 1/10th second (100000us) time average.
  //if(dt>100000)
  //  MaintainHeadingAverageDerivative=GetDegreesPerSecond();
  //else
  //  MaintainHeadingAverageDerivative=(((float)dt)*.00001)*GetDegreesPerSecond()+((100000-(float)dt)*.00001)*MaintainHeadingAverageDerivative;
  dt=micros()-MaintainHeadingPrevTimeUs;
  MaintainHeadingPrevTimeUs=MaintainHeadingPrevTimeUs+dt;
  if(dt>100000)
    dt=100000;  
    MaintainHeadingIntegral+=(((float)dt)*.0001)*Proportional;
  if(MaintainHeadingIntegral>20*MAINTAIN_HEADING_MAX_INTEGRAL_TERM)
    MaintainHeadingIntegral=20*MAINTAIN_HEADING_MAX_INTEGRAL_TERM;
  if(MaintainHeadingIntegral<-20*MAINTAIN_HEADING_MAX_INTEGRAL_TERM)
    MaintainHeadingIntegral=-20*MAINTAIN_HEADING_MAX_INTEGRAL_TERM;   
  Output=Proportional+MaintainHeadingIntegral/20;
  //Use this version in case MaintainHeadingAverageDerivative has to brought back:
  //Output=Proportional+MaintainHeadingIntegral/20-(Wiggle==0?1*MaintainHeadingAverageDerivative/4:0*MaintainHeadingAverageDerivative/4);
  if(Output>150)
    Output=150;
  if(Output<-150)
    Output=-150;
  left=Speed+Output;
  right=Speed-Output;
  //Note: only include this section if you know what your robots min motor speed is (below which it doesn't move at all):
  if(abs(left)<MIN_MOTOR_SPEED){
    left=(left>0?MIN_MOTOR_SPEED:left<0?-MIN_MOTOR_SPEED:0);
  }
  if(abs(right)<MIN_MOTOR_SPEED){
    right=(right>0?MIN_MOTOR_SPEED:right<0?-MIN_MOTOR_SPEED:0);
  }
  motors(left,right);
  //a debug/test section:
  /*RecordedDataRow.Prop=Proportional;
  RecordedDataRow.Int=MaintainHeadingIntegral;
  RecordedDataRow.Der=MaintainHeadingAverageDerivative;
  RecordedDataRefresh();*/
  return ret;
}
//A complete example: (CTRL-/ to un-comment)
//#include "Hardware.h"
//int Heading;
//int Speed=100;
//int Wiggle=0;
//void setup(){
//  HardwareBegin();
//  PlayStartChirp(); 
//  while(!ButtonPressed());
//  delay(1000);
//  NavigationBegin();
//  ResumeNavigation();
//  Heading=PresentHeading();
//  MaintainHeadingReset();
//}
//void loop(){
//  SimpleGyroNavigation();//or SimpleNavigation(), or NavigationXY()
//  MaintainHeading(Heading,Speed,Wiggle);
//  if(PresentHeading()>Heading)
//    SetPixelRGB(BODY_TOP,0,0,10);
//  else if(PresentHeading()==Heading)
//    SetPixelRGB(BODY_TOP,0,10,0);
//  else
//    SetPixelRGB(BODY_TOP,10,0,0);
//}




void DriveArc(int TurnDegrees, int left, int right, int MaxExpectedTurnTime, int MaxExpectedSkidTime){//Ver. 1.0, Dustin Soodak
  uint32_t timeout;
  int degr,DegrPredict,DegrInit;
  if(NavigationOn){
    CalibrateNavigationSensors();
    ResumeNavigation();
  }
  else
    NavigationBegin();
  
  DegrInit=GetDegrees();     
  motors(left,right);
  timeout=millis()+MaxExpectedTurnTime;
  restartTimer();
  while(millis()<timeout){
    SimpleGyroNavigation();
    degr=GetDegrees()-DegrInit;
    DegrPredict=degr+GetDegreesToStop(); //DegrPredict=degr+GyroDegreesToStopFromRaw(rate);//(((float)(-rate))*2000/32768)*0.1029-(24);        
    if(TurnDegrees>=0?(DegrPredict>=TurnDegrees):(DegrPredict<=TurnDegrees)){ 
      break; 
    }
  }
  motors(0,0);     
  timeout=millis()+MaxExpectedSkidTime;
  while(millis()<timeout){
    SimpleNavigation();
  }
  PauseNavigation();
}

char RotateAccurate(int Heading, int MaxExpectedTurnTime){//Ver. 1.0, Dustin Soodak
  uint32_t timeout,timestart;
  int degr,skid,motor,degrprev;
  char res=0;
  char reverses=0;
  //RecordedDataReset(100000);
  if(NavigationOn){
    CalibrateNavigationSensors();
    ResumeNavigation();
  }
  else
    NavigationBegin();    
  timestart=millis();
  timeout=timestart+MaxExpectedTurnTime;
  degrprev=GetDegrees()-Heading;
  while(1){
    SimpleGyroNavigation();//NavigationXY(100,800);//SimpleGyroNavigation();
    //Data.degr=GetDegrees();
    //Data.ms=RecordedDataTime();
    //if(!RecordedDataFull())
    //  RecordedDataRefresh();
    degr=GetDegrees()-Heading;
    if((degr>0)!=(degrprev>0) && millis()>timestart+300)
      reverses++;
    degrprev=degr;
    skid=GetDegreesToStop();    
    if(abs(degr)<=1){
      motors(0,0);
      if(GetDegreesPerSecond()==0){        
        timeout=millis()+50;
        while(millis()<timeout){
          SimpleGyroNavigation();//NavigationXY(100,800);//SimpleGyroNavigation();
        }
        degr=GetDegrees()-Heading;
        if(abs(degr)<=1){
          res=1;
          break;
        }
      }
    }
    else{
      motor=MIN_MOTOR_SPEED+abs(degr+skid);
      if(motor>200)
        motor=200;      
      if(degr+skid>0)
        motors(-motor,motor); 
      else
        motors(motor,-motor); 
    }
    if(millis()>timeout || reverses>3){      
      res=0;
      motors(0,0);
      while(GetDegreesPerSecond()!=0 && millis()<timeout+500){
        SimpleGyroNavigation();//NavigationXY(100,800);//SimpleGyroNavigation(); 
      }
      break;
    }
  }//end while(1)
  return res;
}//end Rotate()



void MoveWithOptions(int Heading, int Distance, int Speed, int MaxExpectedRunTime, int MaxExpectedSkidTime, void (*EdgeFunction)(char), char Wiggle){//Ver. 1.0, Dustin Soodak
  
  uint32_t timeout;
  int xrelative,yrelative,xrelativeinit,yrelativeinit,X,Y;
  int Input,Output,Proportional,Integral=0,Derivative;
  int MinLeftEdge,MinRightEdge;
  char left,right,edge;
  signed char BackAway=0;
  signed char OffsetDir=1;
  char StartedInRightDirection=0;
  char DirectionCount=0;
  float theta;
  
  if(!NavigationOn){//put this back (so doesn't reset navigation each time) when it works with accurate rotation function.
    NavigationBegin();
    PauseNavigation();
  }
  //ResetLookAtEdge();    
  
  theta=((float)(90-Heading))*3.14159/180;
  X=cos(theta)*Distance+GetPositionX();
  Y=sin(theta)*Distance+GetPositionY();
  
    
  xrelative=GetPositionX()-X;
  xrelativeinit=xrelative;
  yrelative=GetPositionY()-Y; 
  yrelativeinit=yrelative;
    
  ResumeNavigation(); 
  motors(Speed,Speed);
  timeout=millis()+MaxExpectedRunTime;
  Input=GetDegrees()+GetDegreesToStop();
 
  restartTimer();
  while(millis()<timeout){
    NavigationXY(100,1000);  
    
    xrelative=GetPositionX()-X;
    yrelative=GetPositionY()-Y;
    if(abs(GetDegrees())-Heading>30 && Wiggle==0){
      Heading=90-atan2(-yrelative,-xrelative)*180/3.14159;
      xrelativeinit=xrelative;//finish line is in new direction
      yrelativeinit=yrelative;
    }

    MaintainHeading(Heading,Speed,Wiggle);

    
    /*
    if(OffsetDir>0){
      if(GetDegrees()>Heading+Wiggle)
        OffsetDir=-1;    
    }
    else{
      if(GetDegrees()<Heading-Wiggle)
        OffsetDir=1;
    }
    Input=GetDegrees()+GetDegreesToStop();
    Proportional=(Heading+OffsetDir*Wiggle-Input);//make it try to turn to the wiggle value
    Derivative=GetDegreesPerSecond();
    Integral+=Proportional;
    if(Integral/20>100)
      Integral=20*100;
    if(Integral/20<-100)
      Integral=-20*100; 
    Output=Proportional+Integral/20-(Wiggle==0?3*Derivative/4:Derivative/4);
    if(Output>100)
      Output=100;
    if(Output<-100)
      Output=-100; 
    motors(Speed+Output,Speed-Output);  
    */
    //If dot product of initial and current relative positions is negative, then their orientations differ by more than 90 degrees.
    //This is how we determine if we have passed the imaginary "finish line".
    if(((int32_t)xrelative)*xrelativeinit+((int32_t)yrelative)*yrelativeinit<0){
      
      break;
    }

    if(EdgeFunction){
      edge=LookForEdge();
      if(edge){
        EdgeFunction(edge);        
        break;//exit
      }
    }
  }//end while(millis()<timeout)
  motors(0,0);     
  //SetPixelRGB(5,H_Bright,H_Bright/3,H_Bright/3);SetPixelRGB(6,H_Bright,H_Bright/3,H_Bright/3);RefreshPixels();//for behavior 030
  timeout=millis()+MaxExpectedSkidTime;
  while(millis()<timeout){
    NavigationXY(80,800);//lower values to be sure it really is stationary
    if(IsStationary)
      break;
  }
  
}


void MoveXYWithOptions(int X, int Y, int Speed, int MaxExpectedRunTime, int MaxExpectedSkidTime, void (*EdgeFunction)(char), char Wiggle){//Ver. 1.0, Dustin Soodak
  int32_t xrelative=GetPositionX()-X;
  int32_t yrelative=GetPositionY()-Y;
  int Heading=90-atan2(-yrelative,-xrelative)*180/3.14159;
  int Distance=sqrt(((int32_t)xrelative)*xrelative+((int32_t)yrelative)*yrelative);
  Heading=MinTurn(Heading-GetDegrees())+GetDegrees();
  //Serial.print("heading ");Serial.print(Heading);Serial.print(" Distance ");Serial.println(Distance);
  if(Distance>0)
    MoveWithOptions(Heading,Distance,Speed,MaxExpectedRunTime,MaxExpectedSkidTime,EdgeFunction,Wiggle);
}



// ***************************************************
// end MovementFunctions
// ***************************************************


// ***************************************************
// Short Animations
// ***************************************************


void playStartChirp(void){//Ver. 1.0, Kevin King   
  // This is the startup sequence that plays when Ringo is reset(put in begin())
  playChirp(1000);
  onEyes(40,0,50);  // blue
  delay(50);
  playChirp(2000);
  delay(50);
  playChirp(3000);
  delay(50);
  playChirp(4000);
  delay(100);
  offChirp();
  offEyes();
}

void playQuickChirp(void){//Ver. 1.0, Kevin King   
  // This is the startup sequence that plays when Ringo is reset(put in begin())
  playChirp(2000);
  onEyes(40,0,50);  // blue
  delay(30);
  offChirp();
  offEyes();
}

void playAck(void){//Ver. 1.0, Kevin King
  playChirp(NOTE_C7);
  setPixelRGB(BODY_TOP,0,30,0);//RefreshPixels();
  delay(50);
  offChirp();
  setPixelRGB(BODY_TOP,0,0,0);//RefreshPixels();
}


void playNonAck(void){//Ver. 1.0, Kevin King
  playChirp(NOTE_CS6);
  setPixelRGB(BODY_TOP,30,0,0);//RefreshPixels();
  delay(75);
  offChirp();
  setPixelRGB(BODY_TOP,0,0,0);//RefreshPixels();
}


void playAnger(void){//Ver. 1.0, Dustin Soodak
  char i;
  for(i=0;i<6;i++){
    playChirp(NOTE_C6);
    delay(20);
    playChirp(NOTE_CS6);   
    delay(20);  
  }
  offChirp();  
}


void playBoredom(void){//Ver. 1.0, Dustin Soodak
  char i;
  unsigned int freq,dfreq;
  dfreq=(NOTE_DS6-NOTE_C6)>>4;
  playChirp(NOTE_C6);
  delay(50);
  for(i=0;i<16;i++){    
    playChirp(NOTE_C6+dfreq*i);
    delay(10);
  }
  playChirp(NOTE_DS6);
  delay(100);
    for(i=0;i<16;i++){    
    playChirp(NOTE_DS6-dfreq*i);
    delay(10);
  }
  playChirp(NOTE_C6);
  delay(50);
  offChirp();
}


void playExcited(void){//Ver. 1.0, Dustin Soodak, Kevin King (note sequence)
  playChirp(NOTE_G6);
  delay(100);
  playChirp(NOTE_C7);
  delay(100);
  playChirp(NOTE_A6);
  delay(100);
  playChirp(NOTE_C7);
  delay(100);
  playChirp(NOTE_B6);
  delay(100);  
  playChirp(NOTE_G6);
  delay(100);
  offChirp();
}


// ***************************************************
// end Short Animations
// ***************************************************


// ***************************************************
// Chirp, Sound, Piezo Core Functions
// ***************************************************

void playSweep(int StartNote, int EndNote, int DwellTime){//Ver. 1.0, Kevin King
  if(StartNote<EndNote){
     for(StartNote; StartNote<=EndNote; StartNote++){
      playChirp(StartNote);
      delayMicroseconds(DwellTime);
     }
  }
   else{
     for(StartNote; StartNote>=EndNote; StartNote--){
      playChirp(StartNote);
      delayMicroseconds(DwellTime);
     }  
  }
 offChirp(); 
}



void offPixels(void){//Ver. 1.0, Kevin King              //turns off all pixels
  setAllPixelsRGB(0,0,0);                                //set all pixels to off
}

void offPixel(byte Pixel){//Ver. 1.0, Kevin King         //turns off a specific pixel
  setPixelRGB(Pixel,0,0,0);                              //set this pixel to off
}

void onEyes(byte Red, byte Green, byte Blue){//Ver. 1.0, Kevin King     //makes eyes the given color, automatically calls RefreshPixels()
  setPixelRGB(EYE_RIGHT,Red,Green,Blue);                 //set right eye
  setPixelRGB(EYE_LEFT,Red,Green,Blue);                  //set left eye
}

void leftEye(byte Red, byte Green, byte Blue){//Ver. 1.0, Kevin King     //makes left eye the given color, automatically calls RefreshPixels()
  setPixelRGB(EYE_LEFT,Red,Green,Blue);                  //set left eye
}

void rightEye(byte Red, byte Green, byte Blue){//Ver. 1.0, Kevin King     //makes right eye the given color, automatically calls RefreshPixels()
  setPixelRGB(EYE_RIGHT,Red,Green,Blue);                 //set right eye
}

void offEyes(void){//Ver. 1.0, Kevin King                //turns off eye pixels
  setPixelRGB(EYE_RIGHT,0,0,0);                          //blank right eye
  setPixelRGB(EYE_LEFT,0,0,0);                           //blank left eye
}

void randomEyes(void){//Ver. 1.0, Kevin King             //Sets the pair of eyes to a random color
  int red, green, blue;
  red = random(120);
  green = random(120);
  blue = random(120);
  onEyes(red,green,blue);
}

// ***************************************************
// end Chirp, Sound, Piezo Core Functions
// ***************************************************



// ***************************************************
// Additional Notes:  Chirp / Sound / Piezo
// ***************************************************

  //Terms:
  //TCNT1 is counter
  // goes from BOTTOM = 0x0000 to MAX = 0xFFFF in normal mode.
  //TOP can be either OCR1A or ICR1
  //
  //Table 15-4. Waveform Generation Mode Bit Description(16 different pwm modes, 2 of which are phase & freq correct)
  //Want "phase and frequency correct PWM mode" on p.130
  //Mode is set by WGM 13:0 bits.
  //Mode WGM: 13 12 11 10
  //10         1  0  1  0 PWM, Phase Correct, TOP is ICR1  [so can use OCR1A as duty cycle and OC1A as pin output]
  //
  //TCNT1 goes from BOTTOM to TOP then TOP to BOTTOM
  //TOP can be OCR1A or ICR1
  //Output Compare OC1x cleared when TCNT1=OCR1x while upcounting, and set when down counting.
  //
  // "The Output Compare Registers contain a 16-bit value that is continuously compared with the
  //counter value (TCNT1). A match can be used to generate an Output Compare interrupt, or to
  //generate a waveform output on the OC1x pin."
  //
  //Table 15-3. Compare Output Mode, when in Phase Correct and Phase and Frequency Correct mode
  //(what these bits do depends on what WGM 13:0 are set to)
  //COM1A1 COM1A0
  //     1      0    Clear OC1A on Compare Match when upcounting. Set OC1AB on Compare Match when downcounting.
  //If we want this single pin mode, can't use OCR1A for TOP since will be used for duty cycle of PWM
  //
  //"...note that the Data Direction Register (DDR) bit corresponding
  //to the OC1A or OC1B pin must be set in order to enable the output driver."
  //
  //CS12 CS11 CS10
  //off mode(all 0), set prescvaler, or external clock source
  //
  //COM1A1 COM1A0 are bits 7:6 in                   TCCR1A
  //data direction register: bit 1 of               DDRB 
  //WGM13 WGM12 are bits 4:3 in                     TCCR1B
  //WGM11 WGM10 are bits 1:0 in                     TCCR1A
  //CS12,11,10 are bits 2:0 in                      TCCR1B
  //total count (/2-1 to get period of waveform)is  ICR1
  //duty cycle (/2 to get period it is on) is       OCR1A
  //
  //TCCR1A=(TCCR1A&0b00111111)|0b01000000;
  //
  //
  //If we had the pwm output to the chirper on another pin:
  //"Using the ICR1 Register for defining TOP works well when using fixed TOP values. By using
  //ICR1, the OCR1A Register is free to be used for generating a PWM output on OC1A. However,
  //if the base PWM frequency is actively changed by changing the TOP value, using the OCR1A as
  //TOP is clearly a better choice due to its double buffer feature."
  //
  //Another doc for R/W of 16 bit registers: 
  //http://www.embedds.com/programming-16-bit-timer-on-atmega328/

// ***************************************************
// end Additional Notes:  Chirp / Sound / Piezo
// ***************************************************



// ***************************************************
// Timer
// ***************************************************

int32_t Timer_InitTime=0,Timer_StoppedTime=0;
char Timer_Running=0;
int32_t GetTime(void){//Ver. 1.0, Dustin Soodak
  if(Timer_Running){
    return millis()-(uint32_t)Timer_InitTime;
  }
  else
    return Timer_StoppedTime;
}
void restartTimer(void){//Ver. 1.0, Dustin Soodak
  Timer_InitTime=millis();
  Timer_Running=1;
}
void stopTimer(void){//Ver. 1.0, Dustin Soodak
  if(Timer_Running){
    Timer_StoppedTime=millis()-(uint32_t)Timer_InitTime;
    Timer_Running=0;
  }
}

// ***************************************************
// end Timer
// ***************************************************


// ***************************************************
// IR Data Sending
// ***************************************************



void ModulateIR(unsigned int Frequency, unsigned int OnTime){//Ver. 1.0, Dustin Soodak 
  //ModulateIR(38000,6) seems to produce best square wave for 38kHz.
  //Frequency is in Hz
  //OnTime is in units of UsOn
  uint16_t temp;
  uint16_t period,dutycycle;
  uint8_t prescalerbits;
  if(OnTime>100)
    OnTime=100;
  if (F_CPU/Frequency/2>=0x10000){
    if(F_CPU/Frequency/2>=0x10000*8){
        prescalerbits=0b011;//prescaler 64
        period=F_CPU/Frequency/(2*16);
        dutycycle=F_CPU/1000000*OnTime/2/64;               
    }
    else{
      prescalerbits = 0b010;// prescaler 8
      period=F_CPU/Frequency/(2*8);
      dutycycle=F_CPU/1000000*OnTime/2/8;
    }
  }
  else{
    prescalerbits = 0b001;  //on but no prescaling
    period=F_CPU/Frequency/(2*1);
    dutycycle=F_CPU/1000000*OnTime/2/1;
  }
  if(OnTime==0){
    TCCR1A=0;
    TCCR1B=0;
   // pinMode(Chirp,INPUT);
    
    //Serial.println("off");
  }
  else{
    TCCR1B&=~0b00000111;//turn off timer    
    ICR1=period;
    OCR1B=dutycycle;
    TCCR1A = (0b10<<4) | 0b10;//COM1B1 COM1B0, and WGM11 WGM10
    TCCR1B = (0b10<<3) | prescalerbits;//WGM13 WGM12, and off/(on with prescaler)    
  }
}

void PlayChirpIR(unsigned int Frequency, unsigned int OnTime){//Ver. 1.0, Dustin Soodak
  // ModulateIR used to be called PlayChirpIR, left in for backward compatibility.
   ModulateIR(Frequency,OnTime); 
}

char CheckMenuButton(void){
  byte button;
  if(IsIRDone()){              //wait for an IR remote control command to be received
      button = GetIRButton();  // read which button was pressed, store in "button" variable
      RxIRRestart(4);          // restart looking for another buttom press
      if(button == 17){         // button 17 is the "MENU" button on the IR remote
      return 1;
      }
      else{
      return 0;  
      }
  }
  return 0;
}

// ***************************************************
// end IR Data Sending
// ***************************************************


// ***************************************************
// Recorded Data
// ***************************************************

  //Ver. 1.0, Dustin Soodak
  //Global variables:
  RecordedDataStruct RecordedDataRow;
  RecordedDataStruct RecordedDataArray[RECORDED_DATA_ARRAY_LENGTH];
  unsigned char RecordedDataLength=0;
  unsigned char RecordedDataPosition=0;
  int RecordedDataN;
  uint32_t RecordedDataStart,RecordedDataPrev;
  uint16_t RecordedDataMinDelay;
      
  //RecordedDataRow.ms=RecordedDataTime()/1000;RecordedDataRow.degr=GetDegrees();RecordedDataRefresh();
  
// ***************************************************
// end Recorded Data
// ***************************************************




