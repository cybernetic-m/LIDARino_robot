#include <string.h>
#include <Arduino.h>
#include <math.h>
#include "Motors.h"
#include "firmlib.h"

const int TRIGGER = 4;
const int ECHO = 5;

// Default constructor (does nothing)
Motors::Motors() {}

// Constructor to initialize the motor pins
Motors::Motors(int motor_L_IN1, int motor_L_IN2, int motor_R_IN3, int motor_R_IN4, int motor_L_ENA, int motor_R_ENB, float v_max, float L) {
    // Constructor to initialize the motor pins
     this->motor_L_IN1 = motor_L_IN1;
     this->motor_L_IN2 = motor_L_IN2;
     this->motor_R_IN3 = motor_R_IN3;
     this->motor_R_IN4 = motor_R_IN4;
     this->motor_L_ENA = motor_L_ENA;
     this->motor_R_ENB = motor_R_ENB;
     this->v_max = v_max;
     this->L = L;


     Stop(); // Initialize motors to stop state
}
void Motors::Stop() {

    // Set the motor pins to LOW at start (Stop state)
    digitalWrite( motor_L_IN1, LOW);
    digitalWrite( motor_L_IN2, LOW);
    digitalWrite( motor_R_IN3, LOW);
    digitalWrite( motor_R_IN4, LOW);
    
    // Set PWM to 0 at start (Stop state)
    analogWrite( motor_L_ENA, 0); 
    analogWrite( motor_R_ENB, 0); 

}

void Motors::Move(float v, float omega) {
    
    // Compute v_L and v_R from v and omega from unicycle model formulas
    // Inverse formulas of these:
    // v = (v_L + v_R) / 2
    // omega = (v_R - v_L) / L
    float v_L = v - ((omega * L)/2);
    float v_R = v + ((omega * L)/2);


    // Set the PINs boleans values to move in the correct direction
    if (v_L > 0) {
        // Move forward
        digitalWrite( motor_L_IN1, LOW);
        digitalWrite( motor_L_IN2, HIGH);
    } 
    else if (v_L < 0) {
        digitalWrite( motor_L_IN1, HIGH);
        digitalWrite( motor_L_IN2, LOW);
    }
    else if (v_L == 0) {
        digitalWrite( motor_L_IN1, LOW);
        digitalWrite( motor_L_IN2, LOW);
    }

    if (v_R > 0) {
        // Move forward
        digitalWrite( motor_R_IN3, LOW);
        digitalWrite( motor_R_IN4, HIGH );
    } 
    else if (v_R < 0) {
        digitalWrite( motor_R_IN3, HIGH);
        digitalWrite( motor_R_IN4, LOW);
    }
    else if (v_R == 0) {
        digitalWrite( motor_R_IN3, LOW);
        digitalWrite( motor_R_IN4, LOW);
    }

    // Compute the PWM value for the motors
    int speed_L = round((v_L / v_max)* 255);
    int speed_R = round((v_R / v_max)* 253); // We set 253 because we observe a small drift because of the right wheel, calibrate yours!
    
    // Limit the speed maximum to 255 and invert the sign if negative for both wheels
    if (speed_L > 255 || speed_L < -255) {
        speed_L = 255;
    }
    else if (speed_L < 0) {
        speed_L = -speed_L;
    }
    if (speed_R > 255 || speed_R < -255) {
        speed_R = 255;
    }
    else if (speed_R < 0) {
        speed_R = -speed_R;
    }
    
    // Set PWM to 0 at start (Stop state)
    analogWrite( motor_L_ENA, speed_L); 
    analogWrite( motor_R_ENB, speed_R); 

}
