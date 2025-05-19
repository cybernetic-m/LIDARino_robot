#include "firmlib.h"

void velocitiesCallback(const geometry_msgs::Twist &velocities) {
    // Read the linear and angular velocity
    v = velocities.linear.x; 
    omega = velocities.angular.z; 
    
    // Convert float (double) into strings
    dtostrf(v, 6, 2, v_str); // 6: minimum width, 2: decimal places
    dtostrf(omega, 6, 2, omega_str); 

    // Now publish the velocities to the "serial_monitor" topic
    // First we create the string using concatenation
    strcpy(serial_data, "v: ");
    strcat(serial_data, v_str);
    strcat(serial_data, " omega: ");
    strcat(serial_data, omega_str);
    
    // Save the string to the data field of serial_msg object 
    serial_msg.data = serial_data;  

    // Publish the data to the ROS topic
    serial_publisher.publish(&serial_msg);
  }