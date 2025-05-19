#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>  

/*
This is a simple keyboard interface for controlling a robot using ROS.
The robot can be controlled using the following keys:

'w' for forward, 's' for backward, 'a' for left, 'd' for right,
'p' for increase linear speed, 'l' for decrease linear speed
'o' for increase angular speed, 'k' for decrease angular speed
*/



int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_interface"); // Initialize the ROS node

    ros::NodeHandle nh; // Create a NodeHandle object as an interface to the ROS nodes

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); // Create a publisher for the cmd_vel topic
    geometry_msgs::Twist cmd_vel; //Create a Twist message struct 

    cmd_vel.linear.x = 0.0; // Initialize the linear velocity to 0
    cmd_vel.angular.z = 0.0; // Initialize the angular velocity to 0
    char key; // Variable to store the key pressed 
    float linear_speed = 0.5; // Set the initial linear speed (variable used to increase/decrease the speed)
    float angular_speed = 0.5; // Set the initial angular speed (variable used to increase/decrease the speed)

    // Set the loop rate (to control the frequency of checking for key presses)
    ros::Rate loop_rate(10); // 10 Hz

    // Print instructions to the console
    std::cout << "Instructions to move the robot with keyboard:" << std::endl;
    std::cout << "w: forward, s: backward, a: left, d: right" << std::endl;
    std::cout << "p: increase linear speed, l: decrease linear speed" << std::endl;
    std::cout << "o: increase angular speed, k: decrease angular speed" << std::endl;
    std::cout << "Press 'q' to stop the robot" << std::endl;

    // Loop until ros is not closed, we read the key pressed on the keyboard and publish the cmd_vel topic
    while (ros::ok()) {
       
        key = getchar(); // Read the key pressed on the keyboard
        switch (key) {
            case 'w':
                cmd_vel.linear.x = linear_speed; // Increase linear velocity
                break;
            case 's':
                cmd_vel.linear.x = -linear_speed; // Decrease linear velocity
                break;
            case 'a':
                cmd_vel.angular.z = angular_speed; // Increase angular velocity
                break;
            case 'd':
                cmd_vel.angular.z = -angular_speed; // Decrease angular velocity
                break;
            case 'p':
                linear_speed += 0.1; // Increase linear speed
                break;
            case 'l':
                linear_speed -= 0.1; // Decrease linear speed
                break;
            case 'o':
                angular_speed += 0.1; // Increase angular speed
                break;
            case 'k':
                angular_speed -= 0.1; // Decrease angular speed
                break;
            case 'q':
                cmd_vel.linear.x = 0.0; // Stop the robot
                cmd_vel.angular.z = 0.0; // Stop the robot
                break;
            default:
                break;
        }
        cmd_vel_pub.publish(cmd_vel); // Publish the cmd_vel topic
        ros::spinOnce(); // Process any incoming messages
        loop_rate.sleep(); // Sleep to maintain the loop rate
    }

}