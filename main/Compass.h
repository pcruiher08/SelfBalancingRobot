  
/*
  Compass.h - Library to control qmcblehbleh
  Created by Jose Miguel Carrillo,  February 27, 2019.
*/

#include <Wire.h>
#include <QMC5883L.h>

#ifndef Compass_h
#define Compass_h

class Compass{
public:
  Compass(); //Default Constructor
  void initialize();
  float getAngle();

  
private:
  QMC5883L compass;
};

#endif