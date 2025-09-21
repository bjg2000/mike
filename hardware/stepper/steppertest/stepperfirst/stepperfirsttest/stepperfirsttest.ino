/*
  Control a bi-polar stepper motor using the SparkFun ProDriver TC78H670FTG
  By: Pete Lewis
  SparkFun Electronics
  Date: July 2nd, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example does a default setup (full step resolution) and turns the motor back and forth.
  
  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/16836

  Hardware Connections:

  ARDUINO --> PRODRIVER
  D8 --> STBY
  D7 --> EN
  D6 --> MODE0
  D5 --> MODE1
  D4 --> MODE2
  D3 --> MODE3
  D2 --> ERR
  

*/

#include "SparkFun_ProDriver_TC78H670FTG_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_ProDriver
PRODRIVER myProDriver; //Create instance of this object

void setup() {
  //Serial.begin(115200);
  //Serial.println("SparkFun ProDriver TC78H670FTG Example 1");

  // Let's setup some non-default pins to use!
  // Note, to setup any non-default settings, you much do this before you call .begin()
  myProDriver.settings.standbyPin = 2;
  myProDriver.settings.enablePin = 7;
  myProDriver.settings.mode0Pin = 5;
  myProDriver.settings.mode1Pin = 6;
  myProDriver.settings.mode2Pin = 1;
  myProDriver.settings.mode3Pin = 0;
  myProDriver.settings.errorPin = 4; // (if using hardware int, choose wisely)

  myProDriver.begin(); // default settings
}

void loop() {
  myProDriver.step(2000, 0); // turn 200 steps, CW direction
  delay(1000);
  myProDriver.step(2000, 1); // turn 200 steps, CCW direction
  delay(1000);
}