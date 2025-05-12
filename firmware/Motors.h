#ifndef MOTORS_H
#define MOTORS_H
#include <Arduino.h>

class Motors {

    public:
        // Constructor to initialize the motor pins
        Motors(int motor_L_IN1, int motor_L_IN2, int motor_R_IN3, int motor_R_IN4, int motor_L_ENA, int motor_R_ENB);

        void Stop();

        void Move(float v, float omega, int time);

}

#endif