#ifndef FIRMLIB_H
#define FIRMLIB_H
#include <Arduino.h>

// Ultrasound functions
void ultrasound_trigger(int trigger_pin);
long ultrasound_read(int echo_pin);

// Encoder Wheel functions
void encoderInterrupt_L();
void encoderInterrupt_R();
void encoderStart();
void computeVelocities();


#endif
