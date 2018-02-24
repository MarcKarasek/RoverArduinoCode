
/* 

Spirit Rover Robot
Spirit_PreLoadedDemo_Rev01
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

int surfaceBaselineLeft, surfaceBaselineRight, surfaceBaselineRear0, surfaceBaselineRear1; 
boolean servoInstallMode = 0; //sets to 1 if BTN held down during power-on

void setup(){

  // ************ THE SETUP ITEMS BELOW SHOULD GENERALLY BE USED FOR ALL SPIRIT SKETCHES **************
  
  hardwareBegin();                    //initialize Spirit's Arduino processor to work with his circuitry
  SPI_Reset();                        //resets and turns on the SPI port
  setAllPixelsRGB(0,0,0);             //turn off all pixels before we get started
  playStartChirp();                   //Play startup chirp and blink eyes  
  servoCenters();                     //Place all servos back in default position
  
  // ************ THE SETUP ITEMS BELOW CAN BE CUSTOMIZED FOR YOUR SPECIFIC SPIRIT SKETCH **************

  initializeRandom();           //initialize the random number genrator based on ambient light level
  getSurfaceBaseline();         //get surface baseline readings on present surface (for auto roaming behavior)
  PIC_RangefinderEnable();      //turn on rangefinder

  setAllPixelsRGB(0,0,5);
  setPixelRGB(WING_LEFT_END,0,0,10);
  setPixelRGB(WING_RIGHT_END,0,0,10);

  if(buttonPressed()){      //if button is being pressed during installation...
    servoInstall();         //run the servo install code
    servoInstallMode = 1;   //set flag to stay in sevro install mode
    runningMode = 255;      //make sure none of the other behaviors below based on runningMode are run
  }
  
}



///////////////// Setup for RUNNING MODE 1 (Generaly stationary, randomly runs behviors when rangefinder senses an object) ////////////////
//Distance thresholds where rangefinder will trigger a behavior...
//int rangefinder_threshold_min = 30;
//int rangefinder_threshold_max = 125;
int rangeCountsToTriggerAction = 800;
int exampleCount = 11; //how many examples we have loaded

/////////////////////// Setup for RUNNING MODE 2 (Autonymous Roaming) ////////////////
//Roaming behavior values
int roamMotorSpeed = 90;
int bottomSensorThreshold = 50;
int rangeCountsToSeeObject = 2000; //if raw range counts below this threshold, perform rangeSweep and change direction
//int rangefinderThreshold = 90;    //anything closer than this on rangefinder will make rover reverse direction
//int rangeGoodThreshold = 2;       //there must be at least this number of range good counts on range before a reverse will happen
int runBeforeRandomStuff = 2000;  //run time (ms) before stop to re-evaluate if we should do a random behavior

/////////////////////// Setup for RUNNING MODE 3 (Light Seeking) ////////////////
//boolean lightSeeker =0;         //should rover seek light?
int lightSeekMotorSpeed = 90;
int lightSeekTurnStrength = 50;   //difference between motor speeds - larger number makes tighter more aggressive turns

///////////// General setup defines /////////////////
#define CHIRP_TONE_RANGE_THRESH 1500
#define CHIRP_DELAY 25                    //how long quick range chirp plays
#define RANGE_DWELL_DELAY 85              //well time before range reading after servo move


void loop(){
  
  while (runningMode == 1){

        if(PIC_ReadRangefinderCounts() < rangeCountsToTriggerAction){

          int exampleToRun = random(1, exampleCount+1);   //randomly choose an example to run
          
          switch (exampleToRun){
            case 1:
              chirpAndEyes();
              break;
            case 2:
              getJiggy();//twinkleEyes();
              break;
            case 3:
              cycleGripper();
              break;  
            case 4:
              dance01();
              break;
            case 5:
              sayNo();
              break;
            case 6:
              sayYes();
              break;
            case 7:
              hypnotize();
              break;  
            case 8:
              jumpForward();
              break;
            case 9:
              jumpBackward();
              break;
            case 10:
              spinCheer();
              break;
            case 11:
              doublePump();
              break;
            default:
              // do nothing
              break;
          } // end switch
          
        } // end if rangefinder

        //check for button press to advance to the next running mode
        if (buttonPressed()){
        setAllPixelsRGB(0,10,10);
        runningMode = 2;
        delay(100);   //short delay to debounce button
        motors(0,0);  //make sure motors are off when leaving this mode
        offChirp();   //mkae sure chirp is off when leaving this mode
        while (buttonPressed()){  //wait for button to be released
            // do nothing, wait for button to be released
            }
        delay(100);   //short delay to debounce button
        } //end if (buttonPressed())

        SPI_Handler();  //checks to see if SPI communication (usually motor command signals from Raspberry Pi) has been received
                        //if a motor control message is received from Pi, it will set runningMode to zero and break out of this behavior
        delay(50);

  } //end while (runningMode == 1)

  
  while (runningMode == 2){

      motors(roamMotorSpeed,roamMotorSpeed);
      
      timeStamp = millis();   //get starting timestamp 
      while(millis() < (timeStamp+runBeforeRandomStuff)){
           
         //Evaluate if we're over an edge
         PIC_ReadAllSurfaceSensorsInstant();
         delay(5);
    
            if ((surfLeft1 <= (surfaceBaselineLeft - bottomSensorThreshold)) || (surfLeft1 >= (surfaceBaselineLeft + bottomSensorThreshold))){
              //Serial.print("1"); Serial.print("Backup Left "); Serial.print(surfLeft1); Serial.print("\t"); Serial.print(surfaceBaselineLeft);
              //Serial.print("\t"); Serial.print(bottomSensorThreshold); Serial.print("\t"); Serial.println(surfaceBaselineLeft - bottomSensorThreshold);
              //playNonAck();
              backAwayFromLeftEdge();
              timeStamp = millis();   //reset timestamp
              motors(roamMotorSpeed,roamMotorSpeed); //turn the motors back on
            }
          
            if ((surfRight1 <= (surfaceBaselineRight - bottomSensorThreshold)) || (surfRight1 >= (surfaceBaselineRight + bottomSensorThreshold))){
              //Serial.print("1"); Serial.print("Backup Right "); Serial.print(surfRight1); Serial.print("\t"); Serial.print(surfaceBaselineRight);
              //Serial.print("\t"); Serial.print(bottomSensorThreshold); Serial.print("\t"); Serial.println(surfaceBaselineRight - bottomSensorThreshold);
              //playNonAck();
              backAwayFromRightEdge();
              timeStamp = millis();   //reset timestamp
              motors(roamMotorSpeed,roamMotorSpeed); //turn the motors back on
            }

          if(PIC_ReadRangefinderCounts() < rangeCountsToSeeObject){
                 motors(0,0);
                 if(rangeSweep()){ //will cause head to sweep range. If barrier is found to be right, this If will be true. Barrier left, it will be false.
                     //it was true that barrier was more likely on the right
                     motors(-100,100); //rotate slightly left (away from barrier)
                 }else{
                     //it was false that barrier was more likely on the right, so it must be more to the left
                     motors(100,-100); //rotate slightly right (away from barrier)
                 }
                 delay(750);      
                 motors(0,0);
                 timeStamp = millis();   //reset timestamp
                 motors(roamMotorSpeed,roamMotorSpeed); //turn the motors back on
          }// end if(PIC_ReadRangefinderCounts()  
           
          
          //check for button press to advance to the next running mode
          if (buttonPressed()){
              setAllPixelsRGB(0,10,10);
              runningMode = 3;
              setAllPixelsRGB(0,0,0);
              setPixelRGB(WING_LEFT_END,10,10,0);
              setPixelRGB(WING_RIGHT_END,10,10,0);
              onEyes(10,10,0);
              
              delay(100);   //short delay to debounce button
              motors(0,0);  //make sure motors are off when leaving this mode
              offChirp();   //mkae sure chirp is off when leaving this mode
              while (buttonPressed()){  //wait for button to be released
                  // do nothing, wait for button to be released
                  }
              delay(100);   //short delay to debounce button
              break;        //get out of while loop
          } //end if (buttonPressed())

          SPI_Handler();  //checks to see if SPI communication (usually motor command signals from Raspberry Pi) has been received
                          //if a motor control message is received from Pi, it will set runningMode to zero and break out of this behavior
        
      } //end while(millis()
      
      byte randomAction;            //occasionally do some random action
      randomAction = random(0,12);  //larger number will make the action less likely to happen

      switch (randomAction){
            case 1:
              motors(-120,-120);  //back up a short bit in case rover was nearing an edge
              delay(300);
              motors(0,0);
              spinCheer();
              break;
            case 2:
              motors(-120,-120);  //back up a short bit in case rover was nearing an edge
              delay(300);
              motors(0,0);
              getJiggy();
              break;
            case 3:
              motors(-60,-60);  //back up a short bit in case rover was nearing an edge
              delay(150);
              motors(0,0);
              doublePump();
              break;
            default:
              // do nothing
              break;
          } // end switch
 
      delay(50);
      timeStamp = millis();   //reset timestamp
      motors(0,0);  // in case we exit this behavior (motors are re-started immediatley at the top, no normally motors won't actually stop)
  
  } //end while (runningMode == 2)

  while (runningMode == 3){

       onEyes(10,10,0);
              
       PIC_ReadAllAmbientSensors();
       if(ambLeft > ambRight){
              motors(lightSeekMotorSpeed-lightSeekTurnStrength,lightSeekMotorSpeed+lightSeekTurnStrength);    
         }else{
              motors(lightSeekMotorSpeed+lightSeekTurnStrength,lightSeekMotorSpeed-lightSeekTurnStrength);
         } 

       if(PIC_ReadRangefinderCounts() < rangeCountsToSeeObject){
              motors(0,0);
              if(rangeSweep()){ //will cause head to sweep range. If barrier is found to be right, this If will be true. Barrier left, it will be false.
                  //it was true that barrier was more likely on the right
                  motors(-100,100); //rotate slightly left (away from barrier)
              }else{
                  //it was false that barrier was more likely on the right, so it must be more to the left
                  motors(100,-100); //rotate slightly right (away from barrier)
              }
              delay(750);      
              motors(0,0);
       }// end if(PIC_ReadRangefinderCounts()  
          
        //check for button press to advance to the next running mode
        if (buttonPressed()){
            setAllPixelsRGB(0,10,10);
            motors(0,0);  //make sure motors are stopped
            offChirp();   //make sure chirp is turned off
            runningMode = 1;  //back to default running mode
            delay(100);   //short delay to debounce button
            while (buttonPressed()){  //wait for button to be released
                // do nothing, wait for button to be released
                }
            delay(100);   //short delay to debounce button
        } //end if (buttonPressed())

        SPI_Handler();  //checks to see if SPI communication (usually motor command signals from Raspberry Pi) has been received
                        //if a motor control message is received from Pi, it will set runningMode to zero and break out of this behavior
        delay(50);
        
  } //end while (runningMode == 3) 

  if (servoInstallMode == 1){
    while(!buttonPressed()){  //while button is not pressed...
      // do nothing
    }
    servoInstall();           //re-run the servo install code if button pressed again
  
  } //end if (servoInstallMode == 1)
      
} // end of loop() function




void hypnotize(void){
  int chirpTone;
  int leftRed,leftGreen,leftBlue;
  int rightRed,rightGreen,rightBlue;
  
  for (int i=0; i<15; i++){
    chirpTone = random(1000, 5000);
    leftRed = random(0, 120);
    leftGreen = random(0, 120);
    leftBlue = random(0, 120);
    rightRed = random(0, 120);
    rightGreen = random(0, 120);
    rightBlue = random(0, 120);
    leftEye(leftRed,leftGreen,leftBlue);
    rightEye(rightRed,rightGreen,rightBlue);
    playChirp(chirpTone);
    delay(50);
  }
  offChirp();
  setAllPixelsRGB(0,0,0);
  onEyes(0,0,20);  
}

void dance01(void){
  int danceMotorSpeed, eyeRed, eyeGreen, eyeBlue, chirpTone;
  int wingsRed, wingsGreen, wingsBlue;
  boolean dir = 0;
  
  for (int i=0; i<10; i++){
      danceMotorSpeed = random(50, 150);
      chirpTone = random(1000, 4500);
      eyeRed = random(0, 120);
      eyeGreen = random(0, 120);
      eyeBlue = random(0, 120);
      wingsRed = random(0, 120);
      wingsGreen = random(0, 120);
      wingsBlue = random(0, 120);
      setAllPixelsRGB(wingsRed,wingsGreen,wingsBlue);
      onEyes(eyeRed,eyeGreen,eyeBlue);
      playChirp(chirpTone);
      if(dir){  //positive direction
        dir = 0;
        motors(danceMotorSpeed,-danceMotorSpeed);
      }else{    //negative direction
        dir = 1;
        motors(-danceMotorSpeed,danceMotorSpeed);
      }
      delay(100);
    
  } //end for
  motors(0,0);
  offChirp();
  setAllPixelsRGB(0,0,0);
  onEyes(0,0,20);
}

void jumpForward(void){
  playChirp(3500);
  onEyes(50,50,50);
  motors(250,250);
  delay(100);
  motors(-250,-250);
  delay(120);
  motors(0,0);
  offChirp();
  onEyes(0,0,15);
  delay(500);
}

void jumpBackward(void){
  playChirp(1500);
  onEyes(50,10,0);
  motors(-120,-120);
  delay(120);
  motors(100,100);
  delay(150);
  motors(0,0);
  offChirp();
  onEyes(0,0,15);
  delay(500);
}

void sayYes(void){
  
  onEyes(0,200,0);
  servoSpeed(1000);
  playChirp(3000);
  for(int i=0; i<2; i++){
    servoTilt(SERVO_TILT_CENTER+50);
    waitForServos();
    servoTilt(SERVO_TILT_CENTER-50);
    waitForServos();
  }
  servoTilt(SERVO_TILT_CENTER);
  waitForServos();
  servoSpeed(3000);
  onEyes(0,0,15);
  offChirp();
}

void sayNo(void){
  
  onEyes(200,0,0);
  servoSpeed(1000);
  playChirp(1000);
  for(int i=0; i<2; i++){
    servoPan(140);
    waitForServos();
    servoPan(40);
    waitForServos();
  }
  servoPan(90);
  waitForServos();
  servoSpeed(3000);
  onEyes(0,0,15);
  offChirp();
}

void cycleGripper(void){
  servoSpeed(150);
  onEyes(100,0,60);
  playChirp(4000);
  servoTilt(15);
  waitForServos();
  //delay(200);
  offChirp();
  servoSpeed(SERVO_SPEED_DEFAULT);
  servoGrip(SERVO_GRIP_STOWE+30);
  delay(750);
  servoSpeed(150);
  servoGrip(SERVO_GRIP_MAX_GRASP);
  waitForServos();  //wait for servo to reach target
  servoGrip(SERVO_GRIP_STOWE+30);
  waitForServos();  //wait for servo to reach target
  servoSpeed(SERVO_SPEED_DEFAULT);
  servoGrip(SERVO_GRIP_STOWE);
  servoTilt(SERVO_TILT_CENTER);
  onEyes(0,0,20);
  delay(300);
}

void chirpAndEyes(void){  //blinks eyes to different colors while chirping
  playChirp(1000);
  onEyes(0,0,120);  // blue
  delay(50);
  playChirp(2000);
  delay(50);
  playChirp(3000);
  delay(50);
  playChirp(4000);
  delay(100);

  onEyes(0,120,120);  // blue
  delay(50);
  playChirp(4000);
  delay(50);
  playChirp(5000);
  delay(50);
  playChirp(6000);
  delay(100);
  
  offChirp();
  onEyes(0,0,10);
  delay(500);
}

void twinkleEyes(void){
  for (int i=0; i<5; i++){
    leftEye(0,100,100);
    rightEye(100,0,100);
    delay(100);
    rightEye(0,100,100);
    leftEye(100,0,100);
    delay(100);
  }
  onEyes(0,0,10);
}

void spinCheer(void){

  motors(-255,255);

  setAllPixelsRGB(100,0,100);
  onEyes(0,50,150);
  for(int p=1000; p<=4000; p=p+100){
    playChirp(p);
    delay(100);
  }
  motors(0,0);
  offChirp();
  setAllPixelsRGB(0,0,0);
  onEyes(0,0,20);

}

void getJiggy(void){

  byte red,green,blue;
  int pitch;
  for (int x=0; x<6; x++){
    pitch = random(1000,4000);
    playChirp(pitch);
    red = random(0,80);green = random(0,80);blue = random(0,80);
    setAllPixelsRGB(red,green,blue);
    red = random(0,100);green = random(0,100);blue = random(0,100);
    onEyes(red,green,blue);
    motors(-250,250);
    delay(60);

    pitch = random(1000,4000);
    playChirp(pitch);
    red = random(0,80);green = random(0,80);blue = random(0,80);
    setAllPixelsRGB(red,green,blue);
    red = random(0,100);green = random(0,100);blue = random(0,100);
    onEyes(red,green,blue);
    motors(250,-250);
    delay(60);
  }
  
  motors(0,0);
  offChirp();
  setAllPixelsRGB(0,0,0);
  onEyes(0,0,15);
  
}

void doublePump(void){

  byte red,green,blue;
  int pitch;
  byte dir; //used to determine random direction
  dir = random(0,2);
  servoSpeed(3000);
  for (byte p=0; p<2; p++){
    pitch = random(1000,4000);
    playChirp(pitch);
    red = random(0,80);green = random(0,80);blue = random(0,80);
    setAllPixelsRGB(red,green,blue);
    red = random(0,100);green = random(0,100);blue = random(0,100);
    onEyes(red,green,blue);
    servoTilt(150);

    if (dir){           //determine motor and head direction based on random 1 or 0 from above
      servoPan(150);
      motors(250,-250);  
    }else{
      servoPan(30);
      motors(-250,250);
    }
    
    delay(80);
    offChirp();
    motors(0,0);
    delay(150);
    setAllPixelsRGB(0,0,0);
    servoPan(SERVO_PAN_CENTER);
    servoTilt(SERVO_TILT_CENTER);
    delay(250);
  }
  setAllPixelsRGB(0,0,0);
  onEyes(0,0,15);
  
}

void backAwayFromLeftEdge(void){
  byte red,green,blue;
  motors(-120,-120);
  delay(200);
  motors(-60, -90);
  playChirp(NOTE_CS6);
  
  servoSpeed(120);
  servoPan(45);
  servoTilt(30);
  
  red = random(0,80);green = random(0,80);blue = random(0,80);
  setAllPixelsRGB(red,green,blue);
  red = random(0,100);green = random(0,100);blue = random(0,100);
  onEyes(red,green,blue);

  for (int m=0; m<=50; m++){
      PIC_ReadAllSurfaceSensorsInstant();
      if ((surfRear0 <= (surfaceBaselineRear0 - bottomSensorThreshold)) || (surfRear0 >= (surfaceBaselineRear0 + bottomSensorThreshold))
          || (surfRear1 <= (surfaceBaselineRear1 - bottomSensorThreshold)) || (surfRear1 >= (surfaceBaselineRear1 + bottomSensorThreshold))){
            motors(50,50);
            delay(200);
            motors(0,0);
            offChirp();
            getSurfaceBaseline();
            servoPan(90);
            servoTilt(SERVO_TILT_CENTER);
            waitForServos();
            servoSpeed(3000);
            setAllPixelsRGB(0,0,0);
            onEyes(0,20,20);
            return;
          }
      delay(8);
    }
  
  //delay(75);
  offChirp();
  //delay(500);
  motors(80,-80);
  
  for (int m=0; m<=70; m++){
      PIC_ReadAllSurfaceSensorsInstant();
      if ((surfRear0 <= (surfaceBaselineRear0 - bottomSensorThreshold)) || (surfRear0 >= (surfaceBaselineRear0 + bottomSensorThreshold))
          || (surfRear1 <= (surfaceBaselineRear1 - bottomSensorThreshold)) || (surfRear1 >= (surfaceBaselineRear1 + bottomSensorThreshold))){
            motors(50,50);
            delay(200);
            motors(0,0);
            offChirp();
            getSurfaceBaseline();
            servoPan(90);
            servoTilt(SERVO_TILT_CENTER);
            waitForServos();
            servoSpeed(3000);
            setAllPixelsRGB(0,0,0);
            onEyes(0,20,20);
            return;
          }
      delay(8);
    }
  
  //delay(700);
  motors(0,0);
  
  servoPan(90);
  servoTilt(SERVO_TILT_CENTER);
  waitForServos();
  servoSpeed(3000);
  delay(50);            //short delay between PIC communications
  setAllPixelsRGB(0,0,0);
  getSurfaceBaseline();
  onEyes(0,0,20);
}

void backAwayFromRightEdge(void){
  byte red,green,blue;
  motors(-120,-120);
  delay(200);
  motors(-90, -60);
  playChirp(NOTE_CS6);
  
  servoSpeed(120);
  servoPan(135);
  servoTilt(30);
  
  red = random(0,80);green = random(0,80);blue = random(0,80);
  setAllPixelsRGB(red,green,blue);
  red = random(0,100);green = random(0,100);blue = random(0,100);
  onEyes(red,green,blue);

    for (int m=0; m<=50; m++){
      PIC_ReadAllSurfaceSensorsInstant();
      if ((surfRear0 <= (surfaceBaselineRear0 - bottomSensorThreshold)) || (surfRear0 >= (surfaceBaselineRear0 + bottomSensorThreshold))
          || (surfRear1 <= (surfaceBaselineRear1 - bottomSensorThreshold)) || (surfRear1 >= (surfaceBaselineRear1 + bottomSensorThreshold))){
            motors(50,50);
            delay(200);
            motors(0,0);
            offChirp();
            getSurfaceBaseline();
            servoPan(90);
            servoTilt(SERVO_TILT_CENTER);
            waitForServos();
            servoSpeed(3000);
            setAllPixelsRGB(0,0,0);
            onEyes(0,20,20);
            return;
          }
      delay(8);
    }
  
  //delay(75);
  offChirp();
  //delay(500);
  motors(-80,80);

    for (int m=0; m<=70; m++){
      PIC_ReadAllSurfaceSensorsInstant();
      if ((surfRear0 <= (surfaceBaselineRear0 - bottomSensorThreshold)) || (surfRear0 >= (surfaceBaselineRear0 + bottomSensorThreshold))
          || (surfRear1 <= (surfaceBaselineRear1 - bottomSensorThreshold)) || (surfRear1 >= (surfaceBaselineRear1 + bottomSensorThreshold))){
            motors(50,50);
            delay(200);
            motors(0,0);
            offChirp();
            getSurfaceBaseline();
            servoPan(90);
            servoTilt(SERVO_TILT_CENTER);
            waitForServos();
            servoSpeed(3000);
            setAllPixelsRGB(0,0,0);
            onEyes(0,20,20);
            return;
          }
      delay(8);
    }
  //delay(700);
  motors(0,0);
  
  servoPan(90);
  servoTilt(SERVO_TILT_CENTER);
  waitForServos();
  servoSpeed(3000);
  delay(50);          //short delay between PIC communications
  setAllPixelsRGB(0,0,0);
  getSurfaceBaseline();
  onEyes(0,0,20);
}

void getSurfaceBaseline(void){
  delay(30);
  PIC_ReadAllSurfaceSensors();
  surfaceBaselineLeft = surfLeft1;
  surfaceBaselineRight = surfRight1;
  surfaceBaselineRear0 = surfRear0;
  surfaceBaselineRear1 = surfRear1; 
  Serial.print("baseline: "); Serial.print(surfLeft1); Serial.print("\t"); Serial.println(surfRight1);
  delay(100);
}

void waitForServos(void){ //looks at "servosInMotion" and returns once servos have reached their targets
    for (int i=0; i<200; i++){  //loop will only run 200 times (in case it gets stuck for some reason)
    delay(50);
    PIC_ServosInMotion();       //query which servos are in motion
      if(!servosInMotion){        //if no servos are in motion, break out of this loop
        break;
      }
    }
}

boolean rangeSweep(void){
  
  boolean barrierDirection = 0;
  int r30,r40,r50,r60,r70,r80,r90,r100,r110,r120,r130,r140,r150;
  long rangeRunningTotalLeft, rangeRunningTotalRight;  
           
          rangeRunningTotalLeft=0;
          rangeRunningTotalRight=0;
          
          servoSpeed(1000);
          servoPan(30);
          waitForServos();  
        
          servoSpeed(3000);
          delay(40);
          r30 = PIC_ReadRangefinderCounts();
          rangeRunningTotalLeft = rangeRunningTotalLeft + r30;
          //onEyes(100,80,0);
          quickRangeChirp(r30,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(40);
          delay(RANGE_DWELL_DELAY);
          r40 = PIC_ReadRangefinderCounts();
          rangeRunningTotalLeft = rangeRunningTotalLeft + r40;
          //onEyes(100,80,0);
          quickRangeChirp(r40,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
                    
          servoPan(50);
          delay(RANGE_DWELL_DELAY);
          r50 = PIC_ReadRangefinderCounts();
          rangeRunningTotalLeft = rangeRunningTotalLeft + r50;
          //onEyes(100,80,0);
          quickRangeChirp(r50,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(60);
          delay(RANGE_DWELL_DELAY);
          r60 = PIC_ReadRangefinderCounts();
          rangeRunningTotalLeft = rangeRunningTotalLeft + r60;
          //onEyes(100,80,0);
          quickRangeChirp(r60,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(70);
          delay(RANGE_DWELL_DELAY);
          r70 = PIC_ReadRangefinderCounts();
          rangeRunningTotalLeft = rangeRunningTotalLeft + r70;
          //onEyes(100,80,0);
          quickRangeChirp(r70,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(80);
          delay(RANGE_DWELL_DELAY);
          r80 = PIC_ReadRangefinderCounts();
          rangeRunningTotalLeft = rangeRunningTotalLeft + r80;
          //onEyes(100,80,0);
          quickRangeChirp(r80,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(90);
          delay(RANGE_DWELL_DELAY);
          r90 = PIC_ReadRangefinderCounts();
          //onEyes(100,80,0);
          quickRangeChirp(r90,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(100);
          delay(RANGE_DWELL_DELAY);
          r100 = PIC_ReadRangefinderCounts();
          rangeRunningTotalRight = rangeRunningTotalRight + r100;
          //onEyes(100,80,0);
          quickRangeChirp(r100,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(110);
          delay(RANGE_DWELL_DELAY);
          r110 = PIC_ReadRangefinderCounts();
          rangeRunningTotalRight = rangeRunningTotalRight + r110;
          //onEyes(100,80,0);
          quickRangeChirp(r110,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(120);
          delay(RANGE_DWELL_DELAY);
          r120 = PIC_ReadRangefinderCounts();
          rangeRunningTotalRight = rangeRunningTotalRight + r120;
          //onEyes(100,80,0);
          quickRangeChirp(r120,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(130);
          delay(RANGE_DWELL_DELAY);
          r130 = PIC_ReadRangefinderCounts();
          rangeRunningTotalRight = rangeRunningTotalRight + r130;
          //onEyes(100,80,0);
          quickRangeChirp(r130,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(140);
          delay(RANGE_DWELL_DELAY);
          r140 = PIC_ReadRangefinderCounts();
          rangeRunningTotalRight = rangeRunningTotalRight + r140;
          //onEyes(100,80,0);
          quickRangeChirp(r140,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoPan(150);
          delay(RANGE_DWELL_DELAY);
          r150 = PIC_ReadRangefinderCounts();
          rangeRunningTotalRight = rangeRunningTotalRight + r150;
          //onEyes(100,80,0);
          quickRangeChirp(r150,CHIRP_TONE_RANGE_THRESH); //play quick high/low chirp based on range
          //offEyes();
          
          servoSpeed(1000);
          servoPan(90);
          waitForServos();

          // For debugging....
          //Serial.print(r30);Serial.print("\t");Serial.print(r40);Serial.print("\t");
          //Serial.print(r50);Serial.print("\t");Serial.print(r60);Serial.print("\t");
          //Serial.print(r70);Serial.print("\t");Serial.print(r80);Serial.print("\t");
          //Serial.print(r90);Serial.print("\t");Serial.print(r100);Serial.print("\t");
          //Serial.print(r110);Serial.print("\t");Serial.print(r120);Serial.print("\t");
          //Serial.print(r130);Serial.print("\t");Serial.print(r140);Serial.print("\t");
          //Serial.println(r150);
          
          //Serial.print(rangeRunningTotalLeft);Serial.print("\t");Serial.println(rangeRunningTotalRight);
          if(rangeRunningTotalLeft<rangeRunningTotalRight){
            //Serial.println("xxxxxxxx _________");
            barrierDirection = 0; //indicate the barrier is most likely present on the left side
          }else{
            //Serial.println("________ xxxxxxxxx");
            barrierDirection = 1; //indicate the barrier is most likely present on the right side
          }
          //Serial.println();
          return barrierDirection;
}


void quickRangeChirp(int range, int thresh){
   if (range < thresh){
      playChirp(2000);
      onEyes(100,80,0); 
   }else{
      //do something else? Add your own code here.
      //playChirp(1000);
   }
   delay(CHIRP_DELAY);
   offChirp();
   offEyes();
}
            

void servoInstall(void){  //sets servo positions for installation / rover build process

    PIC_LEDsAuto(0x05); //set PiUp LED to manual
    
    for (int i=0; i<4; i++){
      playChirp(2000);
      setAllPixelsRGB(20,20,20);
      PIC_SetLEDs(0x02);
      delay(500);
      offChirp();
      setAllPixelsRGB(0,0,0);
      PIC_SetLEDs(0x00);
      delay(500);  
    }
    
    servoSpeed(SERVO_SPEED_DEFAULT);
    delay(10);
    
    //twiddle and set GRIP servo, RED LED color
    setAllPixelsRGB(20,0,0);
    delay(1000);
    
    for (int i=0; i<4; i++){
      //delay(400);
      servoGrip(80);
      delay(400);
      servoGrip(110);
      delay(400);
    }
    delay(1000);
    //Note: The Rover assembly process uses servoGrip(110) to set servo in position for assembly
    //of the servo arms.
    
    //twiddle and set PAN servo, GREEN LED color
    setAllPixelsRGB(0,20,0);

    for (int i=0; i<4; i++){
      //delay(400);
      servoPan(50);
      delay(400);
      servoPan(SERVO_PAN_CENTER);
      delay(400);
    }
    delay(1000);
    
    //twiddle and set TILT servo, BLUE LED color
    setAllPixelsRGB(0,0,20);
    
    for (int i=0; i<4; i++){
      //delay(400);
      servoTilt(40);
      delay(400);
      servoTilt(SERVO_TILT_CENTER);
      delay(400);
    }
    delay(1000);
    
    setAllPixelsRGB(7,7,7);   //turn all pixels white at low intensity (can be used to inspect proper pixel operation
  
}

void printBits(byte myByte){
  for(byte mask = 0x80; mask; mask >>= 1){
    if(mask  & myByte)
        Serial.print('1');
    else
        Serial.print('0');
  }
}

void print_BufferIn(void){
  for (int i=0; i<10; i++){
    Serial.print(SPI_BufferIn[i]);
    Serial.print("\t");
  }
  Serial.println("");
}



