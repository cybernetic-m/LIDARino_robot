#include "firmlib.h"


// Definition of variables for the LEFT wheel encoder
volatile unsigned long totalPulses_L = 0; // Total number of pulses
volatile float interruptCurrentTime_L = 0; // Current time of the Right Interrupt
volatile float interruptLastTime_L = 0; // Time of the precedent interrupt
volatile float interruptBetweenTime_L = 0; // Time difference between two readings
float rps_L = 0; // Round Per Second of the left wheel
float v_L = 0; // Linear Velocity of the left wheel

// Definition of variables for the RIGTH wheel encoder
volatile unsigned long totalPulses_R = 0; // Total number of pulses
volatile float interruptCurrentTime_R = 0; // Current time of the Right Interrupt
volatile float interruptLastTime_R = 0; // Time of the precedent interrupt
volatile float interruptBetweenTime_R = 0; // Time difference between two readings
float rps_R = 0; // Round Per Second of the right wheel
float v_R = 0; // Linear Velocity of the right wheel

void encoderStart() {
  interruptLastTime_L = millis(); // Initialize the last time at setup
  interruptLastTime_R = millis(); // Initialize the last time at setup
}

void encoderInterrupt_L() {
  
  interruptCurrentTime_L = millis(); // Get the current time
  interruptBetweenTime_L = (interruptCurrentTime_L - interruptLastTime_L); // (s)
  
  if(interruptBetweenTime_L > 6) {
    interruptLastTime_L = interruptCurrentTime_L; // Update the last time
    totalPulses_L++; // Increment the total number of pulses
  }
}

void encoderInterrupt_R() {
  
  interruptCurrentTime_R = millis(); // Get the current time
  interruptBetweenTime_R = (interruptCurrentTime_R - interruptLastTime_R); // (s)

  if(interruptBetweenTime_R > 6) {
    interruptLastTime_R = interruptCurrentTime_R; // Update the last time
    totalPulses_R++; // Increment the total number of pulses
  }
}
