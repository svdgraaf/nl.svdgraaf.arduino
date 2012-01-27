/*
  Button
 
 Turns on and off a light emitting diode(LED) connected to digital  
 pin 13, when pressing a pushbutton attached to pin 2. 
 
 
 The circuit:
 * LED attached from pin 13 to ground 
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 
 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.
 
 
 created 2005
 by DojoDave <http://www.0j0.org>
 modified 30 Aug 2011
 by Tom Igoe
 
 This example code is in the public domain.
 
 http://www.arduino.cc/en/Tutorial/Button
 */

#include "pitches.h"


// constants won't change. They're used here to 
// set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
bool soundPlaying = false;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status



void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);     
}

void loop(){
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == LOW) {     
    // turn LED on:    
    digitalWrite(ledPin, HIGH);
    playSound();
    digitalWrite(ledPin, LOW); 
  } 
  else {
    // turn LED off:
    digitalWrite(ledPin, LOW); 
  }
}

void playSound(){
  if(soundPlaying == false)
  {
    soundPlaying = true;
    // notes in the melody:
    int melody[] = { NOTE_B3, NOTE_E7};
  
    // note durations: 4 = quarter note, 8 = eighth note, etc.:
    int noteDurations[] = { 8, 2};
  
    // iterate over the notes of the melody:
    for (int thisNote = 0; thisNote < 2; thisNote++) {
  
      // to calculate the note duration, take one second 
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000/noteDurations[thisNote];
      tone(8, melody[thisNote],noteDuration);
  
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
      noTone(8);
      soundPlaying = false;
    }
  }
}
