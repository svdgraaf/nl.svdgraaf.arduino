#include <EEPROM.h>
#include <Wire.h>
#include <GlobalDefined.h>
#include "AeroQuad.h"
#include "PID.h"
#include <AQMath.h>
#include <FourtOrderFilter.h>
#include <Device_I2C.h>
#include <Accelerometer_ADXL345.h>
  
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;

void setup(){ 
  //Create a serial connection to display the data on the terminal.
  Serial.begin(115200);
  
  initializeAccel();
}

void loop(){
  measureAccelSum();
  evaluateMetersPerSec();
  Serial.println("foo");
  Serial.println(accelSampleCount);      
  delay(10); 
}
