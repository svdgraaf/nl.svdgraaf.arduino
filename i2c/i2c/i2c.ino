#include <IO_Expander.h>
#include <Wire.h>

IO_Expander io(32);
  unsigned int buff; 
  
void setup(void) {
    // setup i2c
    Wire.begin();
    io.reset();
    io.pinMode(1, OUTPUT);  
    Serial.begin(9600);
    Serial.println("start");
}

void loop(void) {

  io.digitalWrite(1, HIGH);
  
  Wire.beginTransmission(32);
  Wire.write((byte)0x01);
  buff = 99999;
  Wire.endTransmission();
  if(Wire.available())
    {
	  buff >> Wire.read();  //LSB
	  buff >> Wire.read();  //MSB
    }
  Serial.println(buff);


  Serial.print("high: ");
  Serial.println(io.digitalRead(1));

  delay(1000);
  io.digitalWrite(1, LOW);
  
  Serial.print("low: ");
  Serial.println(io.digitalRead(1));

  delay(1000);
}
