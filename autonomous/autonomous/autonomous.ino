#include <Servo.h>

Servo head;
int distance = 0;
int headRotation = 0;
int pos = 0;

//Arduino PWM Speed Control
int E1 = 6;   
int M1 = 7;
int E2 = 5;                         
int M2 = 4;

void setup()
{
    Serial.begin(9600);
    Serial.println("start");
    
    // mark forward
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    
    // servo
    head.attach(9);
    
    centerHead();
}

void loop()
{

  distance = analogRead(0);
  Serial.println(distance);
 
  // if close by an object, start rotating
  if(distance > 100)
  {
    Serial.println("uhoh, possible collision detected, sweeping")
    sweep();
  }
  else
  {
    forward();
  }
}

void forward()
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 128);
  analogWrite(E2, 128 );  
}

void standstill()
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}

void sweep()
{
  boolean foundNext = false;
  
  // stop any movement
  standstill();
  
  // move the head, and measure the distance
  for(pos=0;pos<180;pos+=5)
  {
    head.write(pos);           // sweep start position 
    delay(15);
    if(measure())
    {
      foundNext = true;
      break; 
    }
  }
  
  if(foundNext == false)
  {
    for(pos = 180; pos>=1; pos-=5)     // goes from 180 degrees to 0 degrees 
    {                                
      head.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);
      if(measure())
      {
        foundNext = true;
        break; 
      }
    }
  }

  centerHead();
  
  // next step found?
  if(foundNext)
  {
    if(headRotation > 90)
    {
       // turn left
       left();
       delay(500);
    }
    else
    {
      right();
      delay(500);
    }
  }  
}

void turnLeft()
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  analogWrite(E1, 128);
  analogWrite(E2, 128);   
}

void turnRight()
{
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 128);
  analogWrite(E2, 128 );   
}

void centerHead()
{
    head.write(90);
    delay(15);
}

boolean measure()
{
  currentReadout = analogRead(0);
  Serial.print("sees: ");
  Serial.println(currentReadout);
  
  // if more distant, set the direction
  if(currentReadout < distance)
  {
    headRotation = head.read()
  }
}
