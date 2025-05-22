/*
  Firmware LIDARino

  Components details:
  1) Dual H-Bridge L298N
  2) Arduino Uno
  3) Raspberry Pi 
  4) 

  created 26 Aprile 2025
  by M. Romano, P. Renzi, F. Giarrusso
  modified 


 *  Motor Configuration
 *  Motor Left: Forward Direction => MOTOR_L_IN1 = HIGH, MOTOR_L_IN2 = LOW
 *  Motor Left: Backward Direction => MOTOR_L_IN1 = LOW, MOTOR_L_IN2 = HIGH
 *  Motor Left: Stop => MOTOR_L_IN1 = LOW, MOTOR_L_IN2 = LOW (or HIGH, HIGH)
 *  
 *  Motor Right: Forward Direction => MOTOR_R_IN3 = HIGH, MOTOR_R_IN4 = LOW
 *  Motor Right: Backward Direction => MOTOR_R_IN3 = LOW, MOTOR_$_IN4 = HIGH
 *  Motor Right: Stop => MOTOR_R_IN3 = LOW, MOTOR_R_IN4 = LOW (or HIGH, HIGH)
 *  
 *  For the Velocity => analogWrite(MOTOR_(L,R)_(ENA,ENB), value) where value [0;255] in PWM modulation
 
*/

#define nullptr NULL // Needed to use Arduino IDE on Raspberry Pi

#include <ros.h>
#include <geometry_msgs/Twist.h> // This is the custom message type for the cmd_vel
#include <std_msgs/String.h>
#include "firmlib.h"
#include "Motors.h"

// PIN
const int TRIGGER = 4;
const int ECHO = 5;
const int ENCODER_L = 2;
const int ENCODER_R = 3; 
const int MOTOR_L_IN1 = 7;
const int MOTOR_L_IN2 = 8;
const int MOTOR_L_ENA = 9; 
const int MOTOR_R_IN3 = 12;
const int MOTOR_R_IN4 = 13;
const int MOTOR_R_ENB = 10;


// ENCODERS VARIABLES
extern int steps; // Write your steps based on the encoder wheel (number of white/black spaces)
extern int deltaTime; // Write here the interval of update of the rpm and velocities (ms)
extern float wheelRadius;  // Write here your wheel radius  (our wheel diameter is 6.3 cm => 0.063 m)
extern volatile float currentTime; // In the loop measure the current time to compare with lastTime 
extern volatile float lastTime; // The previous currentTime on previous loop

// Recall of the variables initialized in "encoderWheel.cpp" for the LEFT wheel encoder
extern volatile unsigned long totalPulses_L; // Total number of pulses
extern float rps_L; // Round Per Seconds of the left wheel
extern float v_L; // Linear Velocity of the contact point of the left wheel

// Recall of the variables initialized in "encoderWheel.cpp" for the RIGTH wheel encoder
extern volatile unsigned long totalPulses_R; // Total number of pulses
extern float rps_R; // Round Per Seconds of the right wheel
extern float v_R; // Linear Velocity of the contact point of the right wheel

// Motor definitions
Motors motors;
float v_max = 0.4; // our is 0.4 m/s (max velocity reached by your robot)
float L = 0.215; // Distance between the two wheels (needed to compute v_L and v_R) [in our case 0.135 m]

// ROS-RASPBERRY VARIABLES
// v and omega that will be received by Raspberry Pi that publishes the cmd_vel topic
// vectors of chars to save as a string the v and omega to print in "serial_monitor", and serial_data that is the entire message
// serial_msg will be publish in a topic called "serial_monitor" to print
extern float v;
extern float omega;
extern std_msgs::String serial_msg; 

// Create a subscriber to the cmd_vel topic to read v and omega
// Create a publisher to "serial_monitor" topic to print v and omega
// Create a NodeHandle: the interface that links the Arduino to ROS nodes (you can see its functions in the setup())
extern ros::Publisher serial_publisher;
ros::NodeHandle nh; 
ros::Subscriber<geometry_msgs::Twist> arduino_sub("cmd_vel", &velocitiesCallback); 


// Setup 
void setup() {

Serial.begin(57600); // initialize the serial communication

pinMode(TRIGGER, OUTPUT);  // Set the Digital Pin 4 as Trigger Input of Ultrasound Sensor
pinMode(ECHO, INPUT);  // Set the Digital Pin 5 as Echo Output of Ultrasound Sensor
pinMode(ENCODER_L, INPUT); // Set the Digital Pin 2 as DO of the Encoder Left
pinMode(ENCODER_R, INPUT); // Set the Digital Pin 3 as DO of the Encoder Left
attachInterrupt(digitalPinToInterrupt(ENCODER_L), encoderInterrupt_L, RISING); // Attach the interrupt to the Encoder Left
attachInterrupt(digitalPinToInterrupt(ENCODER_R), encoderInterrupt_R, RISING); // Attach the interrupt to the Encoder Right
pinMode(MOTOR_L_IN1, OUTPUT); // Set the Digital Pin as IN1 of the L298N Driver
pinMode(MOTOR_L_IN2, OUTPUT); // Set the Digital Pin as IN2 of the L298N Driver
pinMode(MOTOR_L_ENA, OUTPUT); // Set the Digital Pin as ENA of the L298N Driver
pinMode(MOTOR_R_IN3, OUTPUT); // Set the Digital Pin as IN3 of the L298N Driver
pinMode(MOTOR_R_IN4, OUTPUT); // Set the Digital Pin as IN4 of the L298N Driver
pinMode(MOTOR_R_ENB, OUTPUT); // Set the Digital Pin as ENA of the L298N Driver

encoderStart(); // Initialize the start time of both encoders

// Constructor of the Motors class
motors = Motors(MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_R_IN3, MOTOR_R_IN4, MOTOR_L_ENA, MOTOR_R_ENB, v_max, L);

// Initialize NodeHandle, the Subscriber Node and the Publisher Node
nh.initNode(); 
nh.subscribe(arduino_sub); 
nh.advertise(serial_publisher);

lastTime = millis(); // Initialize the last time at setup

}


// Loop
void loop() {


// Measure the actual time
currentTime = millis(); 

// Trigger the ultrasound sensor to check for the distance cm
// Eventually stop the motors if the distance is less or equal to 10 centimeters
ultrasound_trigger(TRIGGER); // Trigger the ultrasound sensor
long cm = ultrasound_read(ECHO); // Read the echo from the ultrasound sensor (centimeters)
if (cm <= 10) {
  motors.Stop(); // Stop the motors if the distance is less than 10 cm
}

if (currentTime - lastTime >= deltaTime) {
  computeVelocities();
}

// This function allows the Arduino node to poll and check for incoming messages
nh.spinOnce(); // Poll v, omega and publish the serial_msg topic
delay(50); // Small delay to avoid overloading the CPU

motors.Move(v, omega);

/*
Serial.print("L Total Pulses: ");
Serial.print(totalPulses_L);
Serial.println();
Serial.print("R Total Pulses: ");
Serial.print(totalPulses_R);
Serial.println();
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
Serial.print("Front distance [cm]\n");
Serial.print(cm);
Serial.println();
*/
}
