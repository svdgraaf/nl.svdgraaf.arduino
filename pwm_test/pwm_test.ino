int pwmPin = 9;    // LED connected to digital pin 9
int current_power = 100;

void setup()  { 
    Serial.begin(9600); 
} 

void loop()  {
  if (Serial.available()) {
    delay(100);
    int power = current_power;
    
    while (Serial.available() > 0) {
      char c = Serial.read();
      if(c == 'w')
      {
        power = power + 1;
      }
      if(c == 's')
      {
        power = power - 1; 
      }
      
      if(c != 's' && c != 'w')
      {
        power = 0; 
      }
    }
    
    if(power != current_power)
    {
      analogWrite( pwmPin, power);
      Serial.println(power);
      current_power = power;
    }
  }
}


