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
int deltaTime = 1000; // Write here the interval of update of the rpm and velocities (ms)
float wheelRadius = 0.063;  // Write here your wheel radius  (our wheel is 6.3 cm => 0.063 m)
volatile float currentTime = 0; // In the loop measure the current time to compare with lastTime 
volatile float lastTime = 0; // The previous currentTime on previous loop

// Definition of variables for the LEFT wheel encoder
//int anglePerPulse_L = 360 / (steps*2); // The encoder has 20 pulses per revolution, we used steps*2 because we are counting +1 each change (both rising or falling)
volatile unsigned long totalPulses_L = 0; // Total number of pulses
volatile float interruptCurrentTime_L = 0; // Current time of the Right Interrupt
volatile float interruptLastTime_L = 0; // Time of the precedent interrupt
volatile float interruptBetweenTime_L = 0; // Time difference between two readings
float rps_L=0; // Round Per Seconds of the left wheel
float v_L=0; // Linear Velocity of the contact point of the left wheel

// Definition of variables for the RIGTH wheel encoder
//int anglePerPulse_R = 360 / (steps*2); // The encoder has 20 pulses per revolution, we used steps*2 because we are counting +1 each change (both rising or falling)
volatile unsigned long totalPulses_R = 0; // Total number of pulses
volatile float interruptCurrentTime_R = 0; // Current time of the Right Interrupt
volatile float interruptLastTime_R = 0; // Time of the precedent interrupt
volatile float interruptBetweenTime_R = 0; // Time difference between two readings
float rps_R=0; // Round Per Seconds of the right wheel
float v_R=0; // Linear Velocity of the contact point of the right wheel

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
attachInterrupt(digitalPinToInterrupt(ENCODER_L), encoderInterrupt_L, RISING); // Attach the interrupt to the Encoder Left
attachInterrupt(digitalPinToInterrupt(ENCODER_R), encoderInterrupt_R, RISING); // Attach the interrupt to the Encoder Right
pinMode(MOTOR_L_B, OUTPUT); // Set the Digital Pin 10 as 1A (Forward command) of the Dual H-Bridge 
pinMode(MOTOR_L_F, OUTPUT); // Set the Digital Pin 11 as 1B (Backward command) of the Dual H-Bridge

interruptLastTime_L = millis(); // Initialize the last time at setup
interruptLastTime_R = millis(); // Initialize the last time at setup

lastTime = millis(); // Initialize the last time at setup

}


// Loop
void loop() {

// Trigger the ultrasound sensor
ultrasound_trigger(TRIGGER); 

// Read the echo from the ultrasound sensor
cm = ultrasound_read(ECHO); 

currentTime = millis(); // measure the actual time


if (currentTime - lastTime >= deltaTime) {
  rps_L = (float)totalPulses_L / ((steps*2) * (deltaTime/1000.0)) ;
  rps_R = (float)totalPulses_R / ((steps*2) * (deltaTime/1000.0));

  v_L = (2*3.14*rps_L) * wheelRadius;
  v_R = (2*3.14*rps_R) * wheelRadius;
  
  totalPulses_L = 0;
  totalPulses_R = 0;
  lastTime = currentTime;
}


// Motor Part
digitalWrite(MOTOR_L_F, HIGH);
digitalWrite(MOTOR_L_B, LOW); 

// Some print in the serial monitor to check the code working
//Serial.print(cm);
//Serial.print(" cm");
//Serial.println();
Serial.print("L Total Pulses: ");
Serial.print(totalPulses_L);
Serial.println();
Serial.print("R Total Pulses: ");
Serial.print(totalPulses_R);
Serial.println();
//Serial.print("L Total Angle: ");
//Serial.print(totalAngle_L);
//Serial.println();
//Serial.print("R Total Angle: ");
//Serial.print(totalAngle_R);
//Serial.println();
Serial.print("RPS L: ");
Serial.print(rps_L);
Serial.println();
Serial.print("RPS R: ");
Serial.print(rps_R);
Serial.println();
Serial.print("v_L: ");
Serial.print(v_L);
Serial.println();
Serial.print("v_R: ");
Serial.print(v_R);
Serial.println();

delay(1000);

}

void encoderInterrupt_L() {
  
  interruptCurrentTime_L = millis(); // Get the current time
  interruptBetweenTime_L = (interruptCurrentTime_L - interruptLastTime_L); // (s)
  
  if(interruptBetweenTime_L > 1) {
    interruptLastTime_L = interruptCurrentTime_L; // Update the last time
    totalPulses_L++; // Increment the total number of pulses
  }
}

void encoderInterrupt_R() {
  
  interruptCurrentTime_R = millis(); // Get the current time
  interruptBetweenTime_R = (interruptCurrentTime_R - interruptLastTime_R); // (s)

  if(interruptBetweenTime_R > 1) {
    interruptLastTime_R = interruptCurrentTime_R; // Update the last time
    totalPulses_R++; // Increment the total number of pulses
  }
}
