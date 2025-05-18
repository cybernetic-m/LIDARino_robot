#ifndef MOTORS_H
#define MOTORS_H
#include <Arduino.h>

class Motors {

    private:
    int motor_L_IN1, motor_L_IN2, motor_R_IN3, motor_R_IN4; // PIN for direction of the motors
    int motor_L_ENA, motor_R_ENB; // PIN for PWM speed of the motors
    // Variable needed to compute the PWM value of LEFT and RIGHT motors having v_L and v_R linear velocities at the contact point
    // (v_(L/R) / v_max) * 255; 
    float v_max; 
    float L; 
    int trigger_pin, echo_pin; // needed for the ultrasound sensor check
    public:
    // Constructor to initialize the motor pins
    Motors();
    Motors(int motor_L_IN1, int motor_L_IN2, int motor_R_IN3, int motor_R_IN4, int motor_L_ENA, int motor_R_ENB, float v_max, float L, int trigger_pin, int echo_pin);

    void Stop();

    void Move(float v, float omega, int time);

};

#endif
