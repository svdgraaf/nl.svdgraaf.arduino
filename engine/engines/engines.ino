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
    
    if(incomingByte == 64)
    {
      keyUp = true;
    }
    
    switch(incomingByte)
    {
      // W
      case(119):
        if(keyUp == true)
        {
          w = false;
          keyUp = false;
        }
        else
        {
          w = true;
        }
      break;
      // A
      case(97):
        if(keyUp == true)
        {
          a = false;
          keyUp = false;
        }
        else
        {
          a = true;
          d = false;
        }
       break;
       // S
       case(115):
        if(keyUp == true)
        {
          s = false;
          keyUp = false;
        }
        else
        {
          s = true;
        }
         break;
       // D
       case(100):
        if(keyUp == true)
        {
          d = false;
          keyUp = false;
        }
        else
        {
          a = false;
          d = true;
        }
       break;
      }      
  }
      if(w)
      {
        Serial.println("w active");
        digitalWrite(M1, HIGH);
        digitalWrite(M2, HIGH);
        analogWrite(E1, 255);
        analogWrite(E2, 255);
      }
      
      if(s)
      {
        Serial.println("s active");
        digitalWrite(M1, LOW);
        digitalWrite(M2, LOW);
        analogWrite(E1, 255);
        analogWrite(E2, 255);
      }
      
      if(a)
      {
        Serial.println("a active");
        analogWrite(E1, 255);
        analogWrite(E2, 64);
      }
      else if(d)
      {
        Serial.println("d active");
        analogWrite(E1, 64);
        analogWrite(E2, 255);
      }
}

void stop()
{
  Serial.println("stop");
  analogWrite(E1, 0);
  analogWrite(E2, 0); 
}

void startForward()
{
  Serial.println("forward");
  //  set the engines forward
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  
  //  full power to the thrusters!
  analogWrite(E1, 255);
  analogWrite(E2, 255);
}

void stopMovement()
{
  // full stop
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}


void startBackward()
{
  Serial.println("backward");
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  
  //  full power to the thrusters!
  analogWrite(E1, 255);
  analogWrite(E2, 255);
}


