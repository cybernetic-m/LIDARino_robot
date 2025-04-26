#include "firmlib.h"

void ultrasound_trigger(int trigger_pin) {

    /* This function triggers the ultrasound sensor to send a pulse.
         Parameters:
         trigger_pin: The pin number connected to the trigger of the ultrasound sensor
    */ 
    
    digitalWrite(trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(trigger_pin, LOW);
}

long ultrasound_read(int echo_pin) {

    /* This function reads the echo from the ultrasound sensor. 
       The speed of sound is 340 m/s or 29 microseconds per centimeter.
       The ping travels out and back, so to find the distance of the object we
       take half of the distance travelled.

         Parameters:
         echo_pin: The pin number connected to the echo of the ultrasound sensor
    */ 

    long duration, cm;

    duration = pulseIn(echo_pin, HIGH);
    cm = duration / 29 / 2;
    
    return cm;
}