#include "Arduino.h"
void setup();
void loop();
//Arduino PWM Speed Control
int E1 = 6;   
int M1 = 7;
int E2 = 5;                         
int M2 = 4;

// used for detecting incoming bytes
int piece = 0;
String cmd = "";
bool reading = false;

void setup() 
{ 
    pinMode(M1, OUTPUT);   
    pinMode(M2, OUTPUT);
    
    Serial.begin(9600);
    Serial.println("start");
} 

void loop() 
{ 
    while (Serial.available() > 0) {
      char input = Serial.read();
      if (input == '[' && reading == false)
      {
        // start listening
        reading = true;
      }
      else if (input == ']')
      {
        foo(); 
        reading = false;
      }
      else
      {
        cmd += input;
      }
    }
}

void foo()
{
  Serial.println(cmd);
  if(cmd == "0|0|0|0")
  {
    // full stop
    Serial.println("full stop");
    analogWrite(E1, 0);
    analogWrite(E2, 0);
  }
  else
  {
    Serial.println("other");
    if(cmd == "1|0|0|0")
    {
      // forward!
      Serial.println("forward");
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(E1, 255);
      analogWrite(E2, 255);
    }
    if(cmd == '0|0|1|0')
    {
      // forward!
      Serial.println("backwards");
      digitalWrite(M1, LOW);
      digitalWrite(M2, LOW);
      analogWrite(E1, 255);
      analogWrite(E2, 255);
    }

    if(cmd == '1|1|0|0')
    {
      // left forward
      Serial.println("left forward");
      digitalWrite(M1, HIGH);
      digitalWrite(M2, LOW);
      analogWrite(E1, 255);
      analogWrite(E2, 255);
    }
    
    if(cmd == '1|0|0|1')
    {
      // left forward
      Serial.println("right forward");
      digitalWrite(M1, LOW);
      digitalWrite(M2, HIGH);
      analogWrite(E1, 255);
      analogWrite(E2, 255);
    }
  }
  cmd = "";
}

//
//        Serial.println("got nothing, full stop");
//        // full stop
//        keyUp = false;
//        w = false;
//        a = false;
//        s = false;
//        d = false;
//        analogWrite(E1, 0);
//        analogWrite(E2, 0);
//
//  // if forward
//  if(w)
//  {
//    // Serial.println("w active");
//    digitalWrite(M1, HIGH);
//    digitalWrite(M2, HIGH);
//    analogWrite(E1, 255);
//    analogWrite(E2, 255);
//  }
//  else if(s)
//  {
//    // Serial.println("s active");
//    digitalWrite(M1, LOW);
//    digitalWrite(M2, LOW);
//    analogWrite(E1, 255);
//    analogWrite(E2, 255);
//  }
//  else
//  {
//    // Serial.println("s/w not active");
//    analogWrite(E1, 0);
//    analogWrite(E2, 0);
//  }
//
//  if(a)
//  {
//    // Serial.println("a active");
//    analogWrite(E1, 255);
//    analogWrite(E2, 64);
//  }
//  else if(d)
//  {
//    // Serial.println("d active");
//    analogWrite(E1, 64);
//    analogWrite(E2, 255);
//  }
