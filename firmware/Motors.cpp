#include <string.h>
#include <Arduino.h>
#include <cmath>
#include "Motors.h"

class Motors {

    public:
        int motor_L_IN1, motor_L_IN2, motor_R_IN3, motor_R_IN4; // PIN for direction of the motors
        int motor_L_ENA, motor_R_ENB; // PIN for PWM speed of the motors
        // Variable needed to compute the PWM value of LEFT and RIGHT motors having v_L and v_R linear velocities at the contact point
        // (v_(L/R) / v_max) * 255; 
        float v_max; 
        float L; // Distance between the two wheels (needed to compute v_L and v_R) [in our case 0.135 m]

        // Constructor to initialize the motor pins
        Motors(int motor_L_IN1, int motor_L_IN2, int motor_R_IN3, int motor_R_IN4, int motor_L_ENA, int motor_R_ENB) {
            // Constructor to initialize the motor pins
            this->motor_L_IN1 = motor_L_IN1;
            this->motor_L_IN2 = motor_L_IN2;
            this->motor_R_IN3 = motor_R_IN3;
            this->motor_R_IN4 = motor_R_IN4;
            this->motor_L_ENA = motor_L_ENA;
            this->motor_R_ENB = motor_R_ENB;

            this->Stop(); // Initialize motors to stop state

        void Stop() {

            // Set the motor pins to LOW at start (Stop state)
            digitalWrite(this->motor_L_IN1, LOW);
            digitalWrite(this->motor_L_IN2, LOW);
            digitalWrite(this->motor_R_IN3, LOW);
            digitalWrite(this->motor_R_IN4, LOW);
            
            // Set PWM to 0 at start (Stop state)
            analogWrite(this->motor_L_ENA, 0); 
            analogWrite(this->motor_R_ENB, 0); 

        }

        void Move(float v, float omega, int time) {
            
            // Compute v_L and v_R from v and omega from unicycle model formulas
            // Inverse formulas of these:
            // v = (v_L + v_R) / 2
            // omega = (v_R - v_L) / L
            float v_L = v - ((omega * L)/2);
            float v_R = v + ((omega * L)/2);

            // Set the PINs boleans values to move in the correct direction
            if v_L > 0 {
                // Move forward
                digitalWrite(this->motor_L_IN1, HIGH);
                digitalWrite(this->motor_L_IN2, LOW);
            } 
            else if v_L < 0 {
                digitalWrite(this->motor_L_IN1, LOW);
                digitalWrite(this->motor_L_IN2, HIGH);
            }
            else if v_L == 0 {
                digitalWrite(this->motor_L_IN1, LOW);
                digitalWrite(this->motor_L_IN2, LOW);
            }

            if v_R > 0 {
                // Move forward
                digitalWrite(this->motor_R_IN3, HIGH);
                digitalWrite(this->motor_R_IN4, LOW);
            } 
            else if v_R < 0 {
                digitalWrite(this->motor_R_IN3, LOW);
                digitalWrite(this->motor_R_IN4, HIGH);
            }
            else if v_R == 0 {
                digitalWrite(this->motor_R_IN3, LOW);
                digitalWrite(this->motor_R_IN4, LOW);
            }

            // Compute the PWM value for the motors
            int speed_L = round((v_L / v_max)* 255);
            int speed_R = round((v_R / v_max)* 255);
            
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
            analogWrite(this->motor_L_ENA, speed_L); 
            analogWrite(this->motor_R_ENB, speed_R); 
            delay(time); // Move for the specified time
            // Stop the motors
            this->Stop();
        }

}