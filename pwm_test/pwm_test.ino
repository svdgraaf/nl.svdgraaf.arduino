int pwmPin = 9;    // LED connected to digital pin 9
int current_power = 120;

void setup()  { 
    Serial.begin(9600);
    analogWrite( 3, current_power);
    analogWrite( 9, current_power);
    analogWrite( 10, current_power);
    analogWrite( 11, current_power);
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
      analogWrite( 3, power);
      analogWrite( 9, power);
      analogWrite( 10, power);
      analogWrite( 11, power);
      Serial.println(power);
      current_power = power;
    }
  }
}



