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

#ifndef HARDWARE_H
#define HARDWARE_H

#include "Arduino.h"
#include "Navigation.h"
#include "Comms.h"
#include <Adafruit_NeoPixel.h>

#define SERVO_SPEED_DEFAULT 3000
#define SERVO_TILT_CENTER 100//125//75
#define SERVO_PAN_CENTER 90
#define SERVO_GRIP_STOWE 10
#define SERVO_GRIP_MAX_GRASP 115

// ***************************************************
// Pin defines
// ***************************************************

#define I2C_Ready 9   //PIC brings high when done processing I2C packets
#define INT0 4        //PIC pulls low when urgent status is present
#define INT1 A2       //Other user defined interrupt uses. Connects to Raspberry Pi GPIO17
                      //Pulling INT1 low for too long will cause Raspberry Pi to shutdown
#define INT2 8        //Extra interrupt between PIC and Arduino for user defined functions

#define SPI_SS 10     //SPI Select Line
#define SPI_MOSI 11   //SPI MOSI (Master Out / Slave In) Line
#define SPI_MISO 12   //SPI MISO (Master In / Slave Out) Line
#define SPI_SCK 13    //SPI Clock Line

#define _38kHz_Rx 2   //38kHz IR Receiver Pin
#define UART_RX 0     //UART Receive Line
#define UART_TX 1     //UART Transmit Line

#define MotorDirection_Right A1
#define MotorDirection_Left A0
#define MotorDrive_Left 6
#define MotorDrive_Right 5

#define IR_Send 3

#define Pixel_Bus 7 //for 6 neo pixel RGB

// ***************************************************
// end Pin defines
// ***************************************************

// ***************************************************
// General hardware
// ***************************************************


extern void hardwareBegin(void);
//Note: at some point might want to incorperate "Switch" functions directly
//into pixel/button functions. Could do this with motor/serial as well but that
//would require modification of the standard Arduino serial library.
extern boolean buttonPressed(void);

// ***************************************************
// end General hardware
// ***************************************************

// ***************************************************
// General Global Variables
// ***************************************************

extern volatile unsigned char debug[4];          //holder for general debug values
extern volatile int ambLeft;                     //Updated by calling PIC_ReadAllAmbientSensors()
extern volatile int ambRight;                    //Updated by calling PIC_ReadAllAmbientSensors()
extern volatile int ambRear;                     //Updated by calling PIC_ReadAllAmbientSensors()
extern volatile int ambLeftAverage;
extern volatile int ambRightAverage;
extern volatile int ambRearAverage;
extern volatile int surfLeft0;                   //Updated by calling PIC_ReadAllSurfaceSensors()
extern volatile int surfLeft1;                   //Updated by calling PIC_ReadAllSurfaceSensors()
extern volatile int surfRight0;                  //Updated by calling PIC_ReadAllSurfaceSensors()
extern volatile int surfRight1;                  //Updated by calling PIC_ReadAllSurfaceSensors()
extern volatile int surfRear0;                   //Updated by calling PIC_ReadAllSurfaceSensors()
extern volatile int surfRear1;                   //Updated by calling PIC_ReadAllSurfaceSensors()
extern volatile int surfLeft0Average;
extern volatile int surfLeft1Average;
extern volatile int surfRight0Average;
extern volatile int surfRight1Average;
extern volatile int surfRear0Average;
extern volatile int surfRear1Average;
extern volatile int powerCurrent;                //Current into power supply in milliamps
extern volatile int powerVoltage;                //Power supply/battery voltage in millivolts
extern volatile int servoCurrent;                //Estimated current into servos after servo movement
extern volatile int powerCurrentAverage;
extern volatile int powerVoltageAverage;
extern volatile int rangeFinder;                 //Present range in mm
extern volatile unsigned char rangeGoodCounts;   //Count of sequential Rangefinder readings that have been within acceptable range (maxes out at 250)
extern volatile signed char  servoPos_Tilt;      //Tilt servo position (-127 to +127)
extern volatile signed char  servoPos_Pan;       //Pan servo position (-127 to +127)
extern volatile signed char  servoPos_Grip;      //Grip servo position (-127 to +127)
extern volatile signed char  servoTrim_Tilt;     //Tilt servo trim (-127 to +127)
extern volatile signed char  servoTrim_Pan;      //Pan servo trim (-127 to +127)
extern volatile signed char  servoTrim_Grip;     //Grip servo trim (-127 to +127)
extern volatile boolean      servoMotion_Tilt;   //Tilt servo in motion
extern volatile boolean      servoMotion_Pan;    //Pan servo in motion
extern volatile boolean      servoMotion_Grip;   //Grip servo in motion
extern volatile unsigned char servosInMotion;    //Bool flags of the above. Indicagtes at least one servo has not yet arrived at target position
extern volatile unsigned char chargeStatus;      //0=No Chg Present, Not Charging; 1=Chg Present, Not Charging; 2= Chg Present, Charging in Progress
extern volatile boolean      int2Button;         //if the int2 line is set to respond to button press (used in buttonPressed handler)
extern volatile byte         runningMode;        //can be used to denote local Arduino control (1) or Pi control (0), or other uses in user code
extern volatile unsigned char crc;               //crc used in I2C packet
extern volatile unsigned long timeStamp;         //used to hold millis() results

extern volatile unsigned char PIC_StatusBank0;            //Updated by calling PIC_ReadStatus(), contains bool status values, assigned to individual flags below
extern volatile unsigned char PIC_StatusBank1;            //Updated by calling PIC_ReadStatus(), contains bool status values, assigned to individual flags below
extern volatile boolean       PIC_CurrentWarningInt;      //Updated by calling PIC_ReadStatus(), flag that over current warning has been set
extern volatile boolean       PIC_VoltageWarningInt;      //Updated by calling PIC_ReadStatus(), flag that under voltage (low battery) warning has been set
extern volatile boolean       PIC_ShutdownNowInt;         //Updated by calling PIC_ReadStatus(), flag that imminent shutdown is in progress (issue warning and shutdown Pi immeidately)
extern volatile boolean       PIC_MotorStopInt;           //Updated by calling PIC_ReadStatus(), flag that motors should be stopped immediately
extern volatile boolean       PIC_SurfaceSenseInt;        //Updated by calling PIC_ReadStatus(), flag that a surface sensor has crossed a setpoint
extern volatile boolean       PIC_PowerSenseInt;          //Updated by calling PIC_ReadStatus(), flag that a power sensor (current or voltage) has crossed a setpoint  
extern volatile boolean       PIC_RangeSenseInt;          //Updated by calling PIC_ReadStatus(), flag that a rangefinder measurement has crossed a setpoint
extern volatile boolean       PIC_AmbSenseInt;            //Updated by calling PIC_ReadStatus(), flag that an ambient sensor has crossed a setpoint
extern volatile boolean       PIC_UARTRxThreshInt;        //Updated by calling PIC_ReadStatus(), UART Rx Buffer byte count over threshold
extern volatile boolean       PIC_UARTRxNullInt;          //Updated by calling PIC_ReadStatus(), UART Rx Buffer has received specified null character
extern volatile boolean       PIC_UARTTxInProg;           //Updated by calling PIC_ReadStatus(), UART Tx Buffer byte count not zero (UART Tx in progress)

extern volatile unsigned char PIC_thresholdComparators1;  //Bool flags of whether 8 different metrics are above or below a given setpoint
extern volatile boolean       PIC_CurrentComparator;      //bit 0 - PIC_CurrentComparator
extern volatile boolean       PIC_VoltageComparator;      //bit 1 - PIC_VoltageComparator
extern volatile boolean       PIC_LeftOuterComparator;    //bit 2 - PIC_LeftOuterComparator
extern volatile boolean       PIC_LeftInnerComparator;    //bit 3 - PIC_LeftInnerComparator
extern volatile boolean       PIC_RightOuterComparator;   //bit 4 - PIC_RightOuterComparator
extern volatile boolean       PIC_RightInnerComparator;   //bit 5 - PIC_RightInnerComparator
extern volatile boolean       PIC_RearOuterComparator;    //bit 6 - PIC_RearOuterComparator
extern volatile boolean       PIC_RearInnerComparator;    //bit 7 - PIC_RearInnerComparator

extern volatile unsigned char PIC_thresholdComparators2;  //Bool flags of whether 8 different metrics are above or below a given setpoint
extern volatile boolean       PIC_RangefinderComparator;  //bit 0 - PIC_RangefinderComparator
extern volatile boolean       PIC_AmbLeftComparator;      //bit 1 - PIC_AmbLeftComparator
extern volatile boolean       PIC_AmbRightComparator;     //bit 2 - PIC_AmbRightComparator
extern volatile boolean       PIC_AmbRearComparator;      //bit 3 - PIC_AmbRightComparator
extern volatile boolean       PIC_WallPower;              //bit 4 - PIC_WallPower

extern volatile unsigned char PIC_AvgIntervalSurface;     //
extern volatile unsigned char PIC_AvgIntervalAmbient;     //
extern volatile unsigned char PIC_AvgIntervalPower;       //

////////////////////////////////////////////////////////
// OpMode Registers. Update values, then call PIC_SetOpMode() to write values to the PIC.
////////////////////////////////////////////////////////
extern volatile boolean       PIC_Disable_LED_PWR;        //Disables Pwr LED even while robot is powered on
extern volatile boolean       PIC_Disable_LED_PIUP;       //Disables PiUp LED
extern volatile boolean       PIC_Disable_LED_COM;        //Disables Com LED
extern volatile boolean       PIC_MotorStop_Surface;      //Issues interrupt to stop motors if a surface sensor crosses a threshold
extern volatile boolean       PIC_MotorStop_Range;        //Issues interrupt to stop motors if rangefinder reading crosses a threshold
extern volatile boolean       PIC_Polarity_Surface;       //Reverses polarity of surface sensor comparators (so they trigger when *above* the setpoint rather than below)
extern volatile boolean       PIC_Polarity_Ambient;       //Reverses polarity of ambient sensor comparators (so they trigger when *below* the setpoint rather than above)
extern volatile boolean       PIC_Polarity_Range;         //Reverses polarity of range sensor comparators (so it triggers when *further* from the setpoint rather than closer)
extern volatile boolean       PIC_INT0_SurfaceFront;      //Causes PIC to pull INT0 line when either front surface sensor crosses the comparator threshold
extern volatile boolean       PIC_INT0_SurfaceRear;       //Causes PIC to pull INT0 line when either rear surface sensor crosses the comparator threshold
extern volatile boolean       PIC_INT0_Ambient;           //Causes PIC to pull INT0 line when either ambient sensor crosses the comparator threshold
extern volatile boolean       PIC_INT0_Range;             //Causes PIC to pull INT0 line when the rangefinder sensor crosses the comparator threshold
extern volatile boolean       PIC_INT0_Power;             //Causes PIC to pull INT0 line when either voltage or current sensor crosses the comparator threshold
extern volatile boolean       PIC_UART_AddNull;           //When set (1), adds null character (0x00) to end of UART->i2c pass through (not set / no null is default)
extern volatile boolean       PIC_INT0_UART_Rx_Count;     //Causes PIC to pull INT0 line when PIC UART Rx buffer exceeds certain threshold of received bytes
extern volatile boolean       PIC_INT0_UART_Rx_Null;      //Causes PIC to pull INT0 line when PIC UART Rx receives a designated Null Character
extern volatile boolean       PIC_UART_Passthrough;       //Causes PIC to relay UART from Comm modules to external UART port
extern volatile boolean       PIC_WiFi_Enable;            //Enables optional attached ESP8266 WiFi Module (1= enabled, 0= disabled)

////////////////////////////////////////////////////////////
// Threshold Comparator Setpoint Values
////////////////////////////////////////////////////////////
extern volatile unsigned int  PIC_CurrentComparatorSetpoint;    //Setpoint. Update this value, then write it to the PIC by calling PIC_SetPowerComparators()       
extern volatile unsigned int  PIC_VoltageComparatorSetpoint;    //Setpoint. Update this value, then write it to the PIC by calling PIC_SetPowerComparators()  
extern volatile unsigned int  PIC_LeftOuterComparatorSetpoint;  //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()    
extern volatile unsigned int  PIC_LeftInnerComparatorSetpoint;  //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()    
extern volatile unsigned int  PIC_RightOuterComparatorSetpoint; //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()   
extern volatile unsigned int  PIC_RightInnerComparatorSetpoint; //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()  
extern volatile unsigned int  PIC_RearOuterComparatorSetpoint;  //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()  
extern volatile unsigned int  PIC_RearInnerComparatorSetpoint;  //Setpoint. Update this value, then write it to the PIC by calling PIC_SetSurfaceComparators()  
extern volatile unsigned int  PIC_RangefinderComparatorSetpoint;//Setpoint. 
extern volatile unsigned int  PIC_AmbLeftComparatorSetpoint;    //Setpoint. Update this value, then write it to the PIC by calling PIC_SetAmbientComparators()
extern volatile unsigned int  PIC_AmbRightComparatorSetpoint;   //Setpoint. Update this value, then write it to the PIC by calling PIC_SetAmbientComparators()
extern volatile unsigned int  PIC_AmbRearComparatorSetpoint;    //Setpoint. Update this value, then write it to the PIC by calling PIC_SetAmbientComparators()
extern volatile unsigned int  PIC_WallPowerComparatorSetpoint;  //Setpoint. Update this value, then write it to the PIC by calling PIC_SetWallPowerComparators()

extern volatile unsigned char PIC_PCON_Register;    //Copy of PIC PCON register immediatley following the most recent reset
extern volatile unsigned char PIC_STATUS_Register;  //Copy of PIC STATUS register immediatley following the most recent reset (bit4 is itneresting, 0= PIC WDT reset occured)
extern volatile unsigned char PIC_UART_RxBufCount;  //Number of bytes presently in PIC UART receive buffer
extern volatile unsigned char PIC_UART_TxBufCount;  //Number of bytes presently in PIC UART transmit buffer
//extern volatile String PIC_UART_TxMessage;          //String

extern volatile unsigned char PIC_PORTA;            //populated by calling PIC_ReadInputPins()
extern volatile unsigned char PIC_PORTB;            //populated by calling PIC_ReadInputPins()
extern volatile unsigned char PIC_PORTC;            //populated by calling PIC_ReadInputPins()
extern volatile unsigned char PIC_PORTD;            //populated by calling PIC_ReadInputPins()
extern volatile unsigned char PIC_PORTE;            //populated by calling PIC_ReadInputPins()
extern volatile unsigned char PIC_PORTF;            //populated by calling PIC_ReadInputPins()
extern volatile unsigned char PIC_PORTG;            //populated by calling PIC_ReadInputPins()
extern volatile boolean       PIC_Button;           //populated by calling PIC_ReadInputPins()
extern volatile boolean       PIC_Button_Pwr;       //populated by calling PIC_ReadInputPins()
extern volatile boolean       PIC_Accel_Int1;       //populated by calling PIC_ReadInputPins()
extern volatile boolean       PIC_Gyro_Int1;        //populated by calling PIC_ReadInputPins()
extern volatile boolean       PIC_Gyro_Int2;        //populated by calling PIC_ReadInputPins()
extern volatile boolean       PIC_38kHzRx;          //populated by calling PIC_ReadInputPins()
extern volatile boolean       PIC_ChgPresent;       //populated by calling PIC_ReadInputPins()
extern volatile boolean       PIC_ChgInProg;        //populated by calling PIC_ReadInputPins()
extern volatile boolean       PIC_XBee_Assoc;       //populated by calling PIC_ReadInputPins()
extern volatile boolean       PIC_Range_Echo;       //populated by calling PIC_ReadInputPins()

extern volatile unsigned char PIC_LATA;             //populated by calling PIC_ReadOutputPins()
extern volatile unsigned char PIC_LATB;             //populated by calling PIC_ReadOutputPins()
extern volatile unsigned char PIC_LATC;             //populated by calling PIC_ReadOutputPins()
extern volatile unsigned char PIC_LATD;             //populated by calling PIC_ReadOutputPins()
extern volatile unsigned char PIC_LATE;             //populated by calling PIC_ReadOutputPins()
extern volatile unsigned char PIC_LATF;             //populated by calling PIC_ReadOutputPins()
extern volatile unsigned char PIC_LATG;             //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_PS5V0_EN;         //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_PS3V3_EN;         //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_LED_PWR;          //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_LED_PIUP;         //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_LED_COM;          //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_MUXA;             //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_MUXB;             //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_PI_SIG_EN;        //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_BATT_SW;          //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_WIFI_PD;          //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_PIEZO;            //populated by calling PIC_ReadOutputPins()
extern volatile boolean       PIC_RANGE_TRIG;       //populated by calling PIC_ReadOutputPins()


// ***************************************************
// end General Global Variables
// ***************************************************


// ***************************************************
// PIC Processor Query/Write Data Functions
// ***************************************************

void PlayChirp(unsigned int frequency);

// ***************************************************
// end PIC Processor Query Data/Write Functions
// ***************************************************

// ***************************************************
// Simple Timer
// ***************************************************
  //Ultra-simple stop watch functions using the built-in arduino millis() function.
  extern int32_t GetTime(void);
  extern void restartTimer(void);
  extern void stopTimer(void);
// ***************************************************
// end Simple Timer
// ***************************************************


// ***************************************************
// Pixels
// ***************************************************
extern void SetPixelRGB(int Pixel, int Red, int Green, int Blue); // primary function for controlling NeoPixel lights
extern void SetAllPixelsRGB(int Red, int Green, int Blue);//0-255 // sets all pixels to same values. automatically refreshes pixels
extern void RefreshPixels(void);

#define NUM_PIXELS 27
#define BODY_TOP 0
#define EYE_LEFT 1
#define EYE_RIGHT 2
#define WING_LEFT_START 3 
#define WING_LEFT_END 14
#define WING_RIGHT_START 15 
#define WING_RIGHT_END 26
 


//
//low level/misc:
//
extern Adafruit_NeoPixel pixels;// = Adafruit_NeoPixel(NUM_PIXELS, Pixel_Bus, NEO_GRB + NEO_KHZ800);
// ***************************************************
// end Pixels
// ***************************************************


// ***************************************************
// Motor
// ***************************************************
#define MOTOR_MAX 255 //motor goes from -255 to 255
extern int LeftMotor;//current motor speed
extern int RightMotor;//current motor speed
extern void MotorsBegin(void);
extern void motors(int LeftMotorSpeed, int RightMotorSpeed);//-255 to 255: negative values to make it go backwards
extern void Motors(int LeftMotorSpeed, int RightMotorSpeed);//-255 to 255: negative values to make it go backwards
// ***************************************************
// end Motor
// ***************************************************


// ***************************************************
// Movement functions
// ***************************************************
//
//To use movement functions, user code must first call "NavigationBegin()", which initializes
//the navigation sensors (though some of them call it automatically). 
//Most of these call "CalibrateNavigationSensors()" in beginning.
//NavigationBegin() calls CalibrateNavigationSensors() for you.
//Navigation will be innacurate if robot is moving when calling NavigationBegin() or
//CalibrateNavigationSensors(). 
//
//Since gravity is a much larger acceleration than it will usually accelerate in
//the XY plane, even slight tipping forwards and backwards will have a huge effect
//if the average tilt while running is significantly different than when the
//nagigation sensors were calibrated.
//
//If 0 entered for EdgeFunction, the move functions will not look for edges.
//If something like Backup with definition void BackUp(void){Motors(-200,-200);}
//is entered, then it will be called when an edge is detected.
//

#define MIN_MOTOR_SPEED 50
#define MAINTAIN_HEADING_MAX_INTEGRAL_TERM 20
extern void MaintainHeadingReset();
extern char MaintainHeading(int Heading, int Speed, int Wiggle);

extern int HeadingWithShortestDistance(int Heading);
//Figures out which of 2 directions (left or right) to turn (in order to get 
//from current direction to Heading.
//Ex: if currently 0 degrees, then HeadingWithShortestDistance(300) will
//return -60 degrees.

extern void MoveXYWithOptions(int X, int Y, int Speed, int MaxExpectedRunTime, int MaxExpectedSkidTime, void (*EdgeFunction)(char), char Wiggle);
//  Goes towards destination {X,Y}.
//    However, this function calculates with both x and y acceleration so is not
//    affected by the Wiggle parameter.
//  ex: ZeroNavigation();MoveXYWithOptions(0,200,150,3000,500,0,50);
//    Since navigation zeroed, it is at coordinates {0,0} and facing 0 degrees (Pi/2 radians) along y-axis.
//    It will wiggle back and forth but still should stop after its y-coordinate reaches
//    approximately 200mm. 
//  In general, this function draws a finish line that is perpendicular to
//    the route from the initial to final {x,y} position. It then continuously
//    resets the "heading" parameter so that it is facing towards the final position
//    and stops when the original finish line is crossed.
//

void MoveWithOptions(int Heading, int Distance, int Speed, int MaxExpectedRunTime, int MaxExpectedSkidTime, void (*EdgeFunction)(char), char Wiggle);
//  Translates Heading & Distance to X,Y and calls MoveXYWithOptions()
//  Ex: MoveWithOptions(0, 100, 150, 2000, 500,0,0);//heading: 0 degrees, for 100mm at speed 150 for maximum of 2 seconds with 500ms timeout for coming to a halt. 
//      No response to edge and no wiggle (so goes in straight line)

extern void DriveArc(int TurnDegrees, int left, int right, int MaxExpectedTurnTime, int MaxExpectedSkidTime);
//  ex: RotateSimple(GetDegrees()+90, 150, 0, 1000, 250);//approximate rotation
//    Set left motor to 150 and right motor to 0. Stop both motors when the 
//    heading we expect to be at after skidding to a halt is 90 degrees to the
//    right of its initial heading.
//  Note: this uses GetDegreesToStop() which looks at how fast it is currently
//    rotating (GetDegreesPerSecond()) and estimates how much further it will
//    go if motors are turned off right now.
//  
extern char RotateAccurate(int Heading, int MaxExpectedTurnTime);
//  ex: RotateAccurate(45,1000);
//    exact rotation (45 degrees to right if ZeroNavigation() just called) with timeout of 1000ms
//

// ***************************************************
// end Movement functions
// ***************************************************



// ***************************************************
// IR
// ***************************************************


//The following was used as reference code in the development of the IR receive functions:
//http://playground.arduino.cc/Code/InfraredReceivers by Paul Malmsten 
//https://github.com/z3t0/Arduino-IRremote by Ken Shirriff
extern void TxIR(unsigned char *Data, int Length);
extern void TxIRKey(byte key);
//
extern void RxIRStop(void);
extern void RxIRRestart(char BytesToLookFor);
extern char IsIRDone(void);
extern byte GetIRButton(void);
extern char IRNumOfBytes;
extern char CheckMenuButton(void);
//
extern byte GetIRButton(void);
extern const uint8_t IRRemoteButtons[][2];
#define IR_1 1
#define IR_2 2
#define IR_3 3
#define IR_4 4
#define IR_5 5
#define IR_6 6
#define IR_7 7
#define IR_8 8
#define IR_9 9
#define IR_0 10
#define IR_Forward 11
#define IR_Left 12
#define IR_Right 13
#define IR_Backward 14
#define IR_Power 15
#define IR_PlumLogo 16
#define IR_Menu 17
#define IR_A 18
#define IR_B 19
#define IR_Play 20
#define IR_X 21
//


//Low level functions:
extern int IRTransitionCount;
extern unsigned char IRBytes[20];
extern byte irData[]={0x00,0xFF,0x00,0x00};
extern char IRActive;
extern volatile char IRReceiving;//note: IRReceiving turned off if IsIRDone() in regular or auto NavigationHandler() and causes ReadSideSensors() to repeat
//
extern void EnableIROutputs(char Level);  //enables or disables the 3 IR outputs. Pass 0 to disable, 1 to enable all 3 IR outputs.
extern void ModulateIR(unsigned int Frequency, unsigned int OnTime); //ModulateIR(38000,5) seems to produce best square wave for 38kHz.
extern void PlayChirpIR(unsigned int Frequency, unsigned int OnTime); //wrapper for IRCarrierWave, bkwd compatibility. use ModulateIR() instead of PlayChirpIR going forward
//
extern void IRHandler(void);
//
// ***************************************************
// end IR
// ***************************************************

// ***************************************************
// Light & Edge Sensing
// ***************************************************
//
#define LIGHT_SENSOR_STABLIZATION_TIME 200  //us
//
#define SIDE_SENSOR_AVER_RISE_TIME 1000000 //us
#define SIDE_SENSOR_AVER_FALL_TIME  100000 //us

//
//
//Stuff used in LookForEdge():
//
//Unfortunately, surfaces can vary in reflectivity from 20 to 800 counts, and some (such as concrete) may vary up to
//a couple hundred counts even when it is smooth. Th
//
//Zeroes: 
//Set all these to 0, then look at average values of sensors from calling LookAtEdge() when unit is 
//nowhere near the surface. Then set them to these average values.
#define LEFT_ZERO 0   //usually close enough to zero that it makes no difference
#define RIGHT_ZERO 0  //usually close enough to zero that it makes no difference
#define REAR_ZERO 20  //usually around 30-40
//
//This feature can eliminate false edge detection triggering for surfaces with uneven reflectivity, though it might be easier
//to do initial testing of LookForEdge() if commented out.
#define CHECK_EDGE_TWICE //default: un-commented
//
//Un-comment exactly one of the following(time adjusted version is default):
//#define RUNNING_AVERAGE //8 value running average stored in memory
#define TIME_ADJUSTED_AVERAGE //approximates a running average over a period of LookAtEdgeStabilizationTime
//
#define STABILIZATION_TIME_DEFAULT 200.  //in ms (used if TIMER_ADJUSTED_AVERAGE is defined).
                                         //Can't make this too short or won't detect an edge.
                                         //Don't make too long or it will be triggered by gradual variations in the
                                         //surface the robot is exploring.
                                         //Technical notes:
                                         //Be aware that if it is called once every 2ms then the averages will only
                                         //change by 1/100 of the difference between the old and new values so we
                                         //only get 1% of the resolution if using ints.
                                         //This version uses "AverageTimes32" units to increase it (for this example)
                                         //to about 30%.
//
//Normally, we compare readings to a running average to account for surfaces that have uneven reflectivity.
//These constants set absolute limits before deciding a reading is too light or dark.
//The dark value for the rear sensor is different since some light always reflects off of the coaster.
//Comment out either of these to disable them.
#define DARK_MIN 5  //default of 5 is mostly disabled
#define LIGHT_MAX 1000 //default of 1000 is mostly disabled
//
//
//Especially important for not getting false positives for surfaces that don't reflect very much (since random
//jumping of reading up and down will probably be bigger than the approximately 20% change we are usually looking for.
#define MAX_EDGE_SENSOR_NOISE 10  //default: 10.  usually in 5 to 15 range
//
//These functions & constants check for CHANGE in reading, and are proportional to the value of the average.
//Make the dark constants 0 and the light constants 2000 in order to disable this feature (do not comment them out).
//*Functions for fast multiplication by floating point number:
#define MultiplyBy7over8(val) ((val)-((val)>>3)) //"val-(val>>3)"  -> "val*7/8" (.75)
#define MultiplyBy6over8(val) ((val)-((val)>>2)) //"val-(val>>2)"  -> "val*6/8" (.875)
#define MultiplyBy9over8(val) ((val)+((val)>>3)) //"val+(val>>3)"  -> "val*9/8" (1.125 or 1 1/8)
#define MultiplyBy10over8(val) ((val)+((val)>>2)) //"val+(val>>2)"  -> "val*10/8" (1.25 or 1 2/8)
//*Constants for regular floating point multiplication(easy to read and adjust):
#define DARK_EDGE_MULT_1 .50//.85 default
#define DARK_EDGE_MULT_2 .50//.85 default
#define LIGHT_EDGE_MULT_1 2.00//1.15 default
#define LIGHT_EDGE_MULT_2 2.00//1.15 default
//Un-comment one of either "Fast" or "Easy to read and adjust" sections below:
////If CHECK_EDGE_TWICE is not defined, then DarkEdgeMult2 and LightEdgeMult2 are not used
////Fast: 
/*#define DarkEdgeMult1(val) (MultiplyBy9over8(val)-MAX_EDGE_SENSOR_NOISE)
#define DarkEdgeMult2(val) (MultiplyBy10over8(val)-MAX_EDGE_SENSOR_NOISE)
#define LightEdgeMult1(val) (MultiplyBy7over8(val)+MAX_EDGE_SENSOR_NOISE)
#define LightEdgeMult2(val) (MultiplyBy6over8(val)+MAX_EDGE_SENSOR_NOISE) */
////Easy to understand and adjust: (use this version by default)
#define DarkEdgeMult1(val) ((((float)(val))*DARK_EDGE_MULT_1)-MAX_EDGE_SENSOR_NOISE)
#define DarkEdgeMult2(val) ((((float)(val))*DARK_EDGE_MULT_2)-MAX_EDGE_SENSOR_NOISE)
#define BrightEdgeMult1(val) ((((float)(val))*LIGHT_EDGE_MULT_1)+MAX_EDGE_SENSOR_NOISE)
#define BrightEdgeMult2(val) ((((float)(val))*LIGHT_EDGE_MULT_2)+MAX_EDGE_SENSOR_NOISE)
//
extern char IsOverEdge(void);
extern char LookForEdge(void);
//
#define RIGHT_DARK 0x01
#define RIGHT_BRIGHT 0x02
#define REAR_DARK 0x04
#define REAR_BRIGHT 0x08
#define LEFT_DARK 0x10
#define LEFT_BRIGHT 0x20
#define EXTRA_DARK 0x40
#define FrontEdgeDetected(edge) ((edge)&(RIGHT_DARK | RIGHT_BRIGHT | LEFT_DARK | LEFT_BRIGHT))
#define RightFrontEdgeDetected(edge) ((edge)&(RIGHT_DARK | RIGHT_BRIGHT))
#define LeftFrontEdgeDetected(edge) ((edge)&(LEFT_DARK | LEFT_BRIGHT))
#define BackEdgeDetected(edge) ((edge)&(REAR_DARK | REAR_BRIGHT))
#define BrightDetected(edge) ((edge)&(RIGHT_BRIGHT | REAR_BRIGHT | LEFT_BRIGHT))
#define FrontDarkDetected(edge) ((edge)&(RIGHT_DARK | LEFT_DARK))
#define RightDetected(edge) ((edge)&(RIGHT_BRIGHT | RIGHT_DARK))
#define LeftDetected(edge) ((edge)&(LEFT_BRIGHT | LEFT_DARK))
//
#define LeftDarkDetected(edge) ((edge)&LEFT_DARK)
#define LeftBrightDetected(edge) ((edge)&LEFT_BRIGHT)
#define RearDarkDetected(edge) ((edge)&REAR_DARK)
#define RearBrightDetected(edge) ((edge)&REAR_BRIGHT)
#define RightDarkDetected(edge) ((edge)&RIGHT_DARK)
#define RightBrightDetected(edge) ((edge)&RIGHT_BRIGHT)
//
extern float LookAtEdgeTimeBetweenReadings;
#ifdef RUNNING_AVERAGE
#define LeftEdgeSensorValuePrev() (LeftEdgeArray[(EdgeArrayPos+7)&7]) //((EdgeArrayPos+7) mod 8) gives previous EdgeArrayPos
#define RearEdgeSensorValuePrev() (RearEdgeArray[(EdgeArrayPos+7)&7]) //((EdgeArrayPos+7) mod 8) gives previous EdgeArrayPos
#define RightEdgeSensorValuePrev() (RightEdgeArray[(EdgeArrayPos+7)&7]) //((EdgeArrayPos+7) mod 8) gives previous EdgeArrayPos
#endif
#ifdef TIME_ADJUSTED_AVERAGE
#define LeftEdgeSensorValuePrev() (LeftEdgeSensorValuePrevious)
#define RearEdgeSensorValuePrev() (RearEdgeSensorValuePrevious)
#define RightEdgeSensorValuePrev() (RightEdgeSensorValuePrevious)
#endif
//
// LookForEdge() calls LookAtEdge(), and uses running average to detect dark tape/edges or white tape.
// dark edges are sensed as the edge sensor moves off the edge of a surface, as the surface is
// no longer present to reflect light to the sensor. dark edges are also sensed when driving
// on a light surface (like white paper) when coming in contact with a black line/tape/etc.
// bright/white edges are detected when moving the sensor from a relatively dark surface to
// a brighter surface. For example, encountering a piece of white paper when driving on a
// dark colored desk surface.
// Output bit num: 0: right dark edge (0x01)(0b000001)  1: right bright edge (0x02)(0b000010)   
//                 2: rear dark edge (0x04)(0b000100)   3: rear bright edge (0x08)(0b001000)  
//                 4: left dark edge (0x10)(0b010000)   5: left bright edge (0x20)(0b100000)
//
// ResetLookAtEdge();//call once in beginning
// while(1){edge=LookForEdge();if(edge&0x04){MoveForwardFast();}}//call LookForEdge() periodically
//
// Will not get stuck in on mode, but this means that if hovering over a dark edge or bright edge, it 
//  eventually "gets used' to it and trigger will go off.
// If not at least few milliseconds pause between calls, LookForEdge() might be less likely 
//  to detect an edge.
//Example for waiting till off of dark edge (or bright edge):
// If a left edge was encountered, you could store the current value of
//  LeftEdgeSensorAverage, then keep calling LookForEdge() (or just LookAtEdge()) until
//  LeftEdgeSensorValue is greater than the stored average value.
//



//
//Low level:
//
//For LookAtEdge()
#ifdef RUNNING_AVERAGE
extern int LeftEdgeArray[8];
extern int RearEdgeArray[8];
extern int RightEdgeArray[8];
extern char EdgeArrayPos;
#endif
#ifdef TIME_ADJUSTED_AVERAGE
extern float LookAtEdgeStabilizationTime;
extern int LeftEdgeSensorValuePrevious,RightEdgeSensorValuePrevious,RearEdgeSensorValuePrevious;
#endif
//
extern void EdgeLightsOn(void);
extern void EdgeLightsOff(void);
//
extern void SwitchAmbientToEdge(void);
extern void SwitchEdgeToAmbient(void);
//
extern int ReadLeftLightSensor(void);
extern int ReadRightLightSensor(void);
extern int ReadBackLightSensor(void);
//
//Legacy:
#define LeftEdgeLightLevel LeftEdgeSensorValue
#define RearEdgeLightLevel RearEdgeSensorValue
#define RightEdgeLightLevel RightEdgeSensorValue
//
//
// ***************************************************
// end Light & Edge Sensing
// ***************************************************


// ***************************************************
// Short Animations
// ***************************************************

extern void PlayStartChirp(void); //put in begin() to indicate that unit has restarted.
extern void PlayAck(void);        //simple acknowledgement chirp. flashes middle pixel green.
extern void PlayNonAck(void);     //simple non-acknowledgement chirp. flashes middle pixel red.
extern void PlayAnger(void);
extern void PlayBoredom(void);
extern void PlayExcited(void);

// ***************************************************
// end Short Animations
// ***************************************************



// ***************************************************
// Chirp, Sound, Piezo, Lighting Functions
// ***************************************************

// Sound functions
extern void PlaySweep(int StartNote, int EndNote, int DwellTime);   //allows to sweep between two tones at a given rate

// Pixel wrapper functions - easy functions for common lighting effects
extern void offPixels(void);                                 //turns off all pixels
extern void offPixel(byte Pixel);                            //turns off a specific pixel
extern void onEyes(byte Red, byte Green, byte Blue);         //makes eyes the given color, automatically calls RefreshPixels()
extern void leftEye(byte Red, byte Green, byte Blue);        //sets left eye to given color, automatically calls RefreshPixels()
extern void rightEye(byte Red, byte Green, byte Blue);       //sets left eye to given color, automatically calls RefreshPixels()
extern void offEyes(void);                                   //turns off eye lights
extern void randomEyes(void);                                //Sets the pair of eyes to a random color


// ***************************************************
// end Chirp, Sound, Piezo, Lighting Functions
// ***************************************************


// ***************************************************
// Note to Tone Mapping
// ***************************************************

//This list maps keyboard notes like "middle C" (NOTE_C4) to
//tones in hertz. These can be used with 
//Copied from http://arduino.cc/en/Tutorial/tone

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

// ***************************************************
// end Note to Tone Mapping
// ***************************************************


// ***************************************************
// EEPROM Functions
// ***************************************************

//Copied from http://playground.arduino.cc/Code/EEPROMWriteAnything  Visit page for use instructions.
#include <EEPROM.h>
#include <Arduino.h>  // for type definitions

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}

// ***************************************************
// end EEPROM Functions
// ***************************************************



// ***************************************************
// Recorded Data
// ***************************************************
  //Ver. 1.1, Dustin Soodak
  //
  //Edit this section for each use:
  typedef struct RecordedDataStruct{int left; int leftaver; int leftamb;  uint32_t t;};//can edit number, names, and data types
  #define RECORDED_DATA_ARRAY_LENGTH 1 //set to 1 if not using
  #define RecordedDataPrintRow() do{Serial.print(RecordedDataRow.left,DEC);Serial.print("\t");Serial.print(RecordedDataRow.leftaver,DEC);Serial.print("\t");Serial.print(RecordedDataRow.leftamb,DEC);Serial.print("\t");Serial.print(RecordedDataRow.t,DEC);}while(0)
 //
  
   //Don't need to edit this section:
  extern RecordedDataStruct RecordedDataRow;
  extern RecordedDataStruct RecordedDataArray[RECORDED_DATA_ARRAY_LENGTH];
  extern unsigned char RecordedDataLength;
  extern unsigned char RecordedDataPosition;
  extern int RecordedDataN;
  extern uint32_t RecordedDataStart,RecordedDataPrev;
  extern uint16_t RecordedDataMinDelay;
  #define RecordedDataReset(uSDelay) \
    do{for(RecordedDataN=0;RecordedDataN<sizeof(RecordedDataArray);RecordedDataN++)  \
      ((char*)RecordedDataArray)[RecordedDataN]=0;  \
    RecordedDataPosition=0;RecordedDataLength=0;RecordedDataMinDelay=uSDelay;RecordedDataStart=micros();RecordedDataPrev=RecordedDataStart-RecordedDataMinDelay;}while(0)
   #define RecordedDataRefresh() \ 
    do{if(RecordedDataMinDelay==0 || micros()-RecordedDataPrev>=RecordedDataMinDelay){  \
           RecordedDataArray[RecordedDataPosition]=RecordedDataRow;  \
           if(RecordedDataPosition<RECORDED_DATA_ARRAY_LENGTH-1) {RecordedDataPosition++;} else {RecordedDataPosition=0;} \
           if(RecordedDataLength<RECORDED_DATA_ARRAY_LENGTH){RecordedDataLength++;}  \
           RecordedDataPrev+=RecordedDataMinDelay;}  \
     }while(0)
  
  #define RecordedDataFull() (RecordedDataLength==RECORDED_DATA_ARRAY_LENGTH)
  #define RecordedDataTime() (micros()-RecordedDataStart)
  #define RecordedDataPrint() do{for(RecordedDataN=0;RecordedDataN<RecordedDataLength;RecordedDataN++){  \
    RecordedDataRow=RecordedDataArray[(RecordedDataN+(RecordedDataFull()?RecordedDataPosition:0))%RECORDED_DATA_ARRAY_LENGTH];RecordedDataPrintRow();Serial.println(); \
    }}while(0)
  //
  
  //Eample:  (tip:  TAB indents selected lines, SHIFT-TAB de-indents, CTRL-/ adds or removes line comments)
  ////Edited header file code:
  //typedef struct RecordedDataStruct{int t; int x;};
  //#define RECORDED_DATA_ARRAY_LENGTH 10
  //#define RecordedDataPrintRow() do{Serial.print(RecordedDataRow.t);Serial.print("_");Serial.print(RecordedDataRow.x);}while(0)
  ////In setup() or loop():
  //  Serial.println("manual delay:");
  //  RecordedDataReset(0);i=0;
  //  while(!RecordedDataFull()){
  //    RecordedDataRow.t=RecordedDataTime();RecordedDataRow.x=i;i++;//set fields of Data
  //    RecordedDataRefresh();//add Data to data array
  //    delay(1);//pause 1ms
  //  }
  //  RecordedDataPrint();
  //  Serial.println("automatic delay:");
  //  RecordedDataReset(1000);i=0;
  //  while(!RecordedDataFull()){
  //    RecordedDataRow.t=RecordedDataTime();RecordedDataRow.x=i;i++;//set fields of Data
  //    RecordedDataRefresh();//add Data to data array
  //  }
  //  RecordedDataPrint();
  
    
// ***************************************************
// end Recorded Data
// ***************************************************


// ***************************************************
// end Hardware
// ***************************************************


#endif
