/*
  Firmware LIDARino

  

  created 26 Aprile 2025
  by M. Romano, P. Renzi, F. Giarrusso
  modified 

 
*/
#include "firmlib.h"

const int TRIGGER = 4;
const int ECHO = 5;

void setup() {
Serial.begin(9600); // initialize the serial communication

pinMode(TRIGGER, OUTPUT);  // Set the Digital Pin 4 as Trigger Input of Ultrasound Sensor
pinMode(ECHO, INPUT);  // Set the Digital Pin 5 as Echo Output of Ultrasound Sensor
}

void loop() {

// Initialization of the variables
long cm;

ultrasound_trigger(TRIGGER); // Trigger the ultrasound sensor

cm = ultrasound_read(ECHO); // Read the echo from the ultrasound sensor


// Some print in the serial monitor to check the code working
Serial.print(cm);
Serial.print(" cm");
Serial.println();

delay(100);

}
