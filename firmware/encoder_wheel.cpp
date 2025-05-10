#include "firmlib.h"

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

float 
