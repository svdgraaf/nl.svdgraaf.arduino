#include <RemoteTransmitter.h>

/*
* Demo for RF remote switch transmitter.
* For details, see RemoteSwitch.h!
*
* This sketch switches some devices on and off in a loop.
*/

//Intantiate a new KaKuSwitch remote, also use pin 11 (same transmitter!)
ElroTransmitter elroTransmitter(12);

void setup() {
    Serial.begin(9600);
}

void loop() {
  Serial.println("sadfasdf");
  elroTransmitter.sendSignal(3,'B',true);
  Serial.println("asdfasdf");
  //wait 2 seconds
  delay(4000);

}

