#include <LiquidCrystal.h>
#include <Wire.h>

//  I2C device address is 0 1 0 0   A2 A1 A0
#define DIP_ADDRESS (0x4 << 3 | 0x0)
#define LED_ADDRESS (0x4 << 3 | 0x7)

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

int testPin = 1;
int photocellPin = 0;     // the cell and 10K pulldown are connected to a0
int photocellReading;     // the analog reading from the sensor divider
int LEDpin = 11;          // connect Red LED to pin 11 (PWM pin)
int LEDbrightness;        // 
float maxLight = 1000;
const char *line = "";
int width = 16;
byte val = 0;
byte port = 0;

int foo=0;

void setup(void) {
    // set up the LCD (16 chars, 2 rows (8x2))
    lcd.begin(width, 2);
    lcd.print("Loading");

    // We'll send debugging information via the Serial monitor
    Serial.begin(9600);
    
    // setup i2c
    Wire.begin();
}

void loop(void) {
	// read the light amount
	photocellReading = analogRead(photocellPin);  
        float percentage = ((photocellReading / maxLight) * 100);
        int blocks = ((width * percentage)) / 100;

	// log the level to the serial
	Serial.print("Light level = ");
	Serial.println(photocellReading);
//	Serial.println(percentage);
//        Serial.println(blocks);

        // print the percentage as dots
        lcd.setCursor(0, 0);
        for (int i=0; i<width; i++) {
          if(i <= blocks)
          {
            lcd.print(".");
          }
          else
          {
            lcd.print(" ");
          }
        }
   
        // move to the first line, and print the percentage
	lcd.setCursor(0, 1);
	lcd.print(photocellReading);
	lcd.print(" ");
        lcd.print(percentage);
        lcd.print("%");
	
        // send data to i2c slave
        Wire.beginTransmission(0x20);  
        Wire.write(0x01); // begin here
        // only 2 switches connected
        Wire.write(port); // send to port 0
        // Note: noise problem on floating pins. Tie to GND through a 2.2k resistor
        Wire.endTransmission();
        if (val == 64) val = 0;
        
//        lcd.setCursor(11, 1);
        Serial.println(digitalRead(8));
        

	// wait a while...
	delay(100);
}
