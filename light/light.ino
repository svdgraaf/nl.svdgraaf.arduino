#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

int photocellPin = 0;     // the cell and 10K pulldown are connected to a0
int photocellReading;     // the analog reading from the sensor divider
int LEDpin = 11;          // connect Red LED to pin 11 (PWM pin)
int LEDbrightness;        // 

void setup(void) {
    // set up the LCD, and write the first word
    lcd.begin(16, 2);
    lcd.print("Lights!");

	// We'll send debugging information via the Serial monitor
	Serial.begin(9600);   
}
 
void loop(void) {
	// read the light amount
	photocellReading = analogRead(photocellPin);  
 
	// log the level to the serial
	Serial.print("Light level = ");
	Serial.println(photocellReading);

	// move to second line, first char, and print the reading
	lcd.setCursor(0, 1);
	lcd.print(photocellReading);
	
	// wait a while...
	delay(100);
}