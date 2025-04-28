/*
  Firmware LIDARino

  Components details:
  1) Dual H-Bridge L9110S
  2) 

  created 26 Aprile 2025
  by M. Romano, P. Renzi, F. Giarrusso
  modified 

 
*/

#include "firmlib.h"

// Definition of the PIN
const int TRIGGER = 4;
const int ECHO = 5;
const int ENCODER_L = 2;
const int ENCODER_R = 3; 
const int MOTOR_L_B = 10;
const int MOTOR_L_F = 11; 


// Definition of variables for the encoder
int anglePerPulse = 360 / 20; // The encoder has 20 pulses per revolution
volatile unsigned long totalPulses = 0; // Total number of pulses
volatile long totalAngle = 0; // Total angle in degrees
volatile float lastTime = 0; // Last time the encoder was read
volatile float currentTime = 0; // Current time
volatile float deltaTime = 0; // Time difference between two readings
volatile double angularVelocity=0; // Angular velocity in degrees per second

//volatile unsigned long timeAverageAngularVelocity=0;
//volatile int sumPulses=0;
//volatile int averageSample=50;


// Setup 
void setup() {
Serial.begin(9600); // initialize the serial communication

pinMode(TRIGGER, OUTPUT);  // Set the Digital Pin 4 as Trigger Input of Ultrasound Sensor
pinMode(ECHO, INPUT);  // Set the Digital Pin 5 as Echo Output of Ultrasound Sensor
pinMode(ENCODER_L, INPUT); // Set the Digital Pin 2 as DO of the Encoder Left
attachInterrupt(digitalPinToInterrupt(ENCODER_L), encoderInterrupt, RISING); // Attach the interrupt to the Encoder Left
//pinMode(ENCODER_R, INPUT); // Set the Digital Pin 3 as DO of the Encoder Right
//attachInterrupt(digitalPinToInterrupt(ENCODER_R), encoderRight, RISING); // Attach the interrupt to the Encoder Right
pinMode(MOTOR_L_B, OUTPUT); // Set the Digital Pin 10 as 1A (Forward command) of the Dual H-Bridge 
pinMode(MOTOR_L_F, OUTPUT); // Set the Digital Pin 11 as 1B (Backward command) of the Dual H-Bridge

lastTime = millis(); // Initialize the last time at setup
}


// Loop
void loop() {

// Initialization of the variables
long cm;

ultrasound_trigger(TRIGGER); // Trigger the ultrasound sensor

cm = ultrasound_read(ECHO); // Read the echo from the ultrasound sensor

digitalWrite(MOTOR_L_F, HIGH);
digitalWrite(MOTOR_L_B, LOW); 

// Some print in the serial monitor to check the code working
Serial.print(cm);
Serial.print(" cm");
Serial.println();
Serial.print("Total Pulses: ");
Serial.print(totalPulses);
Serial.println();
Serial.print("Total Angle: ");
Serial.print(totalAngle);
Serial.println();

delay(1000);

}

void encoderInterrupt() {

  totalPulses++; // Increment the total number of pulses
  currentTime = millis(); // Get the current time
  deltaTime = currentTime - lastTime; // Calculate the time difference
  lastTime = currentTime; // Update the last time
  totalAngle = totalPulses * anglePerPulse; // Calculate the total angle
}
