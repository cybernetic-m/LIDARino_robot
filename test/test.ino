#include <ros.h>
#include <geometry_msgs/Twist.h> // This is the custom message type for the cmd_vel

ros::NodeHandle nh; // this is the interface that links the Arduino to ROS nodes (you can see its functions in the setup())

// Definition of the variables v and omega that will be received by Raspberry Pi that publishes the cmd_vel topic
float v = 0.0 ;
float omega = 0.0;

void velocitiesCallback(const geometry_msgs::Twist &velocities) {
    v = velocities.linear.x; // Read the linear velocity
    omega = velocities.angular.z; // Read the angular velocity
    Serial.print("Received v: ");
    Serial.print(v);
    Serial.print(", omega: ");
    Serial.println(omega);
}

// Create a subscriber to the cmd_vel topic
ros::Subscriber<geometry_msgs::Twist> arduino_sub("cmd_vel", &velocitiesCallback); 

void setup() {

    Serial.begin(115200); // initialize the serial communication

    // Use the NodeHandle interface to initialize the node and subscribe to the cmd_vel topic
    nh.initNode(); 
    nh.subscribe(arduino_sub); 
    Serial.print("Starting communication between Arduino and Raspberry Pi...");
}

void loop() {
    nh.spinOnce(); // This function allows the Arduino node to poll and check for incoming messages
    delay(10); // Small delay to avoid overloading the CPU
}
