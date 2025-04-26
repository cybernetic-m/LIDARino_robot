#ifndef FIRMLIB_H
#define FIRMLIB_H
#include <Arduino.h>

void ultrasound_trigger(int trigger_pin);
long ultrasound_read(int echo_pin);

#endif