#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
int near = 0;

void setup() 
{
  Serial.begin(9600);
  Serial.println("start");
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
} 
 
 
void loop() 
{
  Serial.println("start");
      near = analogRead(0);
    Serial.print("distance: ");
    Serial.println(near);
  sweep();
}

void sweep()
{
  for(pos=0;pos<180;pos+=5)
  {
    myservo.write(pos);           // sweep start position 
    delay(15);
    measure();
  }
  
  for(pos = 180; pos>=1; pos-=5)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);
    measure();
  } 

}

void measure()
{
  near = analogRead(0);
  Serial.print("distance: ");
  Serial.println(near); 
}
