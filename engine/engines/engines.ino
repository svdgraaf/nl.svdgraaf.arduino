//Arduino PWM Speed Control
int E1 = 6;   
int M1 = 7;
int E2 = 5;                         
int M2 = 4;

// used for detecting incoming bytes
int piece = 0;
String cmd = "";
bool reading = false;
bool back = false;
unsigned long lastbeep;

void setup() 
{ 
    lastbeep = 0;
    pinMode(M1, OUTPUT);   
    pinMode(M2, OUTPUT);
    pinMode(12, OUTPUT);
    
    Serial.begin(9600);
    Serial.println("start");
    digitalWrite(12, HIGH);
} 

void loop() 
{
  beep();
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

void beep()
{
  if(back == true)
  {
    if(lastbeep < millis() - 1000)
    {
      tone(8, 440, 1000/4);
     lastbeep = millis(); 
    }
  }
}

void foo()
{
  Serial.println(cmd);
  if(cmd == "0|0|0|0" or cmd == "1|0|1|0")
  {
    back = false;
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
      back = false;
      Serial.println("forward");
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(E1, 255);
      analogWrite(E2, 255);
    }
    if(cmd == "0|0|1|0")
    {
      // forward!
      back = true;
      Serial.println("backwards");
      digitalWrite(M1, LOW);
      digitalWrite(M2, LOW);
      analogWrite(E1, 255);
      analogWrite(E2, 255);
    }

    if(cmd == "0|0|0|1")
    {
      back = false;
      Serial.println("left around axis");
      digitalWrite(M1, LOW);
      digitalWrite(M2, HIGH);
      analogWrite(E1, 255);
      analogWrite(E2, 255);      
    }

    if(cmd == "0|1|0|0")
    {
      back = false;
      Serial.println("right around axis");
      digitalWrite(M1, HIGH);
      digitalWrite(M2, LOW);
      analogWrite(E1, 255);
      analogWrite(E2, 255);      
    }

    if(cmd == "0|1|1|0")
    {
      back = true;
      // left backward
      Serial.println("left backward");
      digitalWrite(M1, LOW);
      digitalWrite(M2, LOW);
      analogWrite(E1, 255);
      analogWrite(E2, 128);
    }

    if(cmd == "0|0|1|1")
    {
      back = true;
      // right backward
      Serial.println("right backward");
      digitalWrite(M1, LOW);
      digitalWrite(M2, LOW);
      analogWrite(E1, 128);
      analogWrite(E2, 255);
    }

    if(cmd == "1|1|0|0")
    {
      back = false;
      // left forward
      Serial.println("left forward");
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(E1, 255);
      analogWrite(E2, 128);
    }
    
    if(cmd == "1|0|0|1")
    {
      back = false;
      // left forward
      Serial.println("right forward");
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(E1, 128);
      analogWrite(E2, 255);
    }
  }
  cmd = "";
}
