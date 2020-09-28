/*
  Compass.cpp - Library to control qmcblehbleh
  Created by Jose Miguel Carrillo,  February 27, 2019.
*/

#include "Arduino.h"
#include "Compass.h"


Compass::Compass(){
  
}

void Compass::initialize(){
  compass.init();
  compass.setSamplingRate(50);
}

float Compass::getAngle(){
  float heading = compass.readHeading();
  return heading;
}