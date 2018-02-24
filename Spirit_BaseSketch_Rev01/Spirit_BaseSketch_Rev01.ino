
/* 

Spirit Rover Robot
Spirit_BaseSketch_Rev01
Version 1.1 10/2017

Significant portions of this code written by
Dustin Soodak for Plum Geek LLC. Some portions
contributed by Kevin King.
Portions from other open source projects where noted.
This code is licensed under:
Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
https://creativecommons.org/licenses/by-sa/2.0/
Visit http://www.plumgeek.com for PiSpirit information.
Visit http://www.arduino.cc to learn about the Arduino.

*/

#include "Hardware.h"


void setup(){

  // ************ THE SETUP ITEMS BELOW SHOULD GENERALLY BE USED FOR ALL SPIRIT SKETCHES **************
  
  hardwareBegin();                    //initialize Spirit's Arduino processor to work with his circuitry
  SPI_Reset();                        //resets and turns on the SPI port
  playStartChirp();                   //Play startup chirp and blink eyes
  servoCenters();                     //Place all servos back in default position
  
  // ************ THE SETUP ITEMS BELOW CAN BE CUSTOMIZED FOR YOUR SPECIFIC SPIRIT SKETCH **************


  // add your own setup code here

  
}


void loop(){

  // add your own loop code here

} // end of loop() function





