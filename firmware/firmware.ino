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

// PIN
const int TRIGGER = 4;
const int ECHO = 5;
const int ENCODER_L = 2;
const int ENCODER_R = 3; 
const int MOTOR_L_B = 10;
const int MOTOR_L_F = 11; 

// ENCODERS VARIABLES
int steps = 20; // Write your steps based on the encoder wheel (number of white/black spaces)

// Definition of variables for the LEFT wheel encoder
int anglePerPulse_L = 360 / (steps*2); // The encoder has 20 pulses per revolution, we used steps*2 because we are counting +1 each change (both rising or falling)
volatile unsigned long totalPulses_L = 0; // Total number of pulses
volatile float currentTime_L = 0; // Current time
volatile float deltaTime_L = 0; // Time difference between two readings
volatile long totalAngle_L = 0; // Total angle in degrees
volatile float lastTime_L = 0; // Last time the encoder was read
volatile double angularVelocity_L=0; // Angular velocity in degrees per second

// Definition of variables for the RIGTH wheel encoder
int anglePerPulse_R = 360 / (steps*2); // The encoder has 20 pulses per revolution, we used steps*2 because we are counting +1 each change (both rising or falling)
volatile unsigned long totalPulses_R = 0; // Total number of pulses
volatile float currentTime_R = 0; // Current time
volatile float deltaTime_R = 0; // Time difference between two readings
volatile long totalAngle_R = 0; // Total angle in degrees
volatile float lastTime_R = 0; // Last time the encoder was read
volatile double angularVelocity_R=0; // Angular velocity in degrees per second

// ULTRASOUND VARIABLES
// Definition of the centimeters for ultrasound sensor
long cm;


// Setup 
void setup() {
Serial.begin(9600); // initialize the serial communication

pinMode(TRIGGER, OUTPUT);  // Set the Digital Pin 4 as Trigger Input of Ultrasound Sensor
pinMode(ECHO, INPUT);  // Set the Digital Pin 5 as Echo Output of Ultrasound Sensor
pinMode(ENCODER_L, INPUT); // Set the Digital Pin 2 as DO of the Encoder Left
pinMode(ENCODER_R, INPUT); // Set the Digital Pin 3 as DO of the Encoder Left
attachInterrupt(digitalPinToInterrupt(ENCODER_L), encoderInterrupt_L, CHANGE); // Attach the interrupt to the Encoder Left
attachInterrupt(digitalPinToInterrupt(ENCODER_R), encoderInterrupt_R, CHANGE); // Attach the interrupt to the Encoder Right
pinMode(MOTOR_L_B, OUTPUT); // Set the Digital Pin 10 as 1A (Forward command) of the Dual H-Bridge 
pinMode(MOTOR_L_F, OUTPUT); // Set the Digital Pin 11 as 1B (Backward command) of the Dual H-Bridge

lastTime_L = millis(); // Initialize the last time at setup
lastTime_R = millis(); // Initialize the last time at setup

}


// Loop
void loop() {

// Trigger the ultrasound sensor
ultrasound_trigger(TRIGGER); 

// Read the echo from the ultrasound sensor
cm = ultrasound_read(ECHO); 

// Compute the difference of time
deltaTime_L = (currentTime_L - lastTime_L)/1000; // (s)
deltaTime_R = (currentTime_R - lastTime_R)/1000; // (s)

// Calculate the total angle
totalAngle_L = totalPulses_L * anglePerPulse_L; // (grad) 
totalAngle_R = totalPulses_R * anglePerPulse_R; // (grad)



// Motor Part
digitalWrite(MOTOR_L_F, HIGH);
digitalWrite(MOTOR_L_B, LOW); 

// Some print in the serial monitor to check the code working
Serial.print(cm);
Serial.print(" cm");
Serial.println();
Serial.print("L Total Pulses: ");
Serial.print(totalPulses_L);
Serial.println();
Serial.print("R Total Pulses: ");
Serial.print(totalPulses_R);
Serial.println();
Serial.print("L Total Angle: ");
Serial.print(totalAngle_L);
Serial.println();
Serial.print("R Total Angle: ");
Serial.print(totalAngle_R);
Serial.println();

delay(1000);

}

void encoderInterrupt_L() {
  currentTime_L = millis(); // Get the current time
  if(currentTime_L - lastTime_L > 1 ) {
    totalPulses_L++; // Increment the total number of pulses
    lastTime_L = currentTime_L; // Update the last time
  }
}

void encoderInterrupt_R() {
  currentTime_R = millis(); // Get the current time
  if(currentTime_R - lastTime_R > 1 ) {
    totalPulses_R++; // Increment the total number of pulses
    lastTime_R = currentTime_R; // Update the last time
  }
}
