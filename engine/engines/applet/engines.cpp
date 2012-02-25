#include "Arduino.h"
void setup();
void loop();
//Arduino PWM Speed Control
int E1 = 6;   
int M1 = 7;
int E2 = 5;                         
int M2 = 4;

// used for detecting incoming bytes
int incomingByte = 0;

bool w = false;
bool a = false;
bool s = false;
bool d = false;

bool keyUp = false;

void setup() 
{ 
    pinMode(M1, OUTPUT);   
    pinMode(M2, OUTPUT);
    
    Serial.begin(9600);
    Serial.println("start");
} 

void loop() 
{ 
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    
    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
    Serial.println(incomingByte);
  
    switch(incomingByte)
    {
      case 64:
        keyUp = true;
        break;
      case 119:
        // w
        if(keyUp == true)
        {
          Serial.println("w = false, keyUp = true");
          w = false;
          keyUp = false;
        }
        else
        {
          Serial.println("w = true, keyUp = false");
          w = true;
        }
        break;
      case 97:
        // a
        if(keyUp == true)
        {
          Serial.println("a = false, keyUp = true");
          a = false;
          keyUp = false;
        }
        else
        {
          Serial.println("a = true, keyUp = false");
          a = true;
        }
        break;
      case 115:
        // s
        if(keyUp == true)
        {
          Serial.println("s = false, keyUp = true");
          s = false;
          keyUp = false;
        }
        else
        {
          Serial.println("s = true, keyUp = false");
          s = true;
        }
        break;
      case 100:
        // d
        if(keyUp == true)
        {
          Serial.println("d = false, keyUp = true");
          d = false;
          keyUp = false;
        }
        else
        {
          Serial.println("d = true, keyUp = false");
          d = true;
        }
        break;
      default:
        Serial.println("got nothing, full stop");
        // full stop
        keyUp = false;
        w = false;
        a = false;
        s = false;
        d = false;
        analogWrite(E1, 0);
        analogWrite(E2, 0);
    }
  }

  // if forward
  if(w)
  {
    // Serial.println("w active");
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, 255);
    analogWrite(E2, 255);
  }
  else if(s)
  {
    // Serial.println("s active");
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(E1, 255);
    analogWrite(E2, 255);
  }
  else
  {
    // Serial.println("s/w not active");
    analogWrite(E1, 0);
    analogWrite(E2, 0);
  }

  if(a)
  {
    // Serial.println("a active");
    analogWrite(E1, 255);
    analogWrite(E2, 64);
  }
  else if(d)
  {
    // Serial.println("d active");
    analogWrite(E1, 64);
    analogWrite(E2, 255);
  }

}
