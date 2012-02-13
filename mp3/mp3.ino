//Arduino PWM Speed ControlÔºö
int E1 = 6;   
int M1 = 7;
int E2 = 5;                         
int M2 = 4;                           

void setup() 
{ 
    pinMode(M1, OUTPUT);   
    pinMode(M2, OUTPUT);
    
    Serial.begin(9600);
    Serial.println("start");
    
} 

void loop() 
{ 
//  forward(5);
//  left(1);
//  right(1);
//  backward(5);
  delay(2000);
}

void forward(int sec)
{
  Serial.println("forward");
  //  set the engines forward
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  
  //  full power to the thrusters!
  analogWrite(E1, 255);
  analogWrite(E2, 255);
  
  delay(1000*sec);
  
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}

void left(int sec)
{
  Serial.println("left");
  digitalWrite(M1, LOW);

  //  full power to the thrusters!
  analogWrite(E1, 255);
  
  delay(1000*sec);
  
  analogWrite(E1, 0);
}

void right(int sec)
{
  Serial.println("right");
  digitalWrite(M2, LOW);

  //  full power to the thrusters!
  analogWrite(E2, 255);
  
  delay(1000*sec);
  
  analogWrite(E2, 0);
}

void backward(int sec)
{
  Serial.println("backward");
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  
  //  full power to the thrusters!
  analogWrite(E1, 255);
  analogWrite(E2, 255);
  
  delay(1000*sec);
  
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}


