#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>  
#include <termios.h> // Termios is "Terminal I/O Settings" and is used to configure terminal I/O characteristics
#include <unistd.h> 

/*
This is a simple keyboard interface for controlling a robot using ROS.
The robot can be controlled using the following keys:

'w' for forward, 's' for backward, 'a' for left, 'd' for right,
'p' for increase linear speed, 'l' for decrease linear speed
'o' for increase angular speed, 'k' for decrease angular speed
*/

// Function to get a single key press from the keyboard
// In particular we use the file descriptor STDIN_FILENO 
// that is in general an int 0, it is the index of the struct array in the kernel that have 
// all the settings for the standard input.
// We take with "tcgetattr" the current settings (the current termios struct) and save into "oldt"
// Then we set to zero the ICANON and ECHO flags in the new termios struct called "newt"
// Finally we set the new settings "newt" into the standard input stream and restore the old settings

char getKey() {
     
    // Create termios struct to store terminal settings (the old and the new)
    // ch is the variable to store the character read
    struct termios oldt, newt; 
    char ch;

    // Get the current standard input stream settings (STDIN_FILENO is an int 0)
    tcgetattr(STDIN_FILENO, &oldt); 
    newt = oldt; // newt is now a pointer to the old settings 

    // Disable canonical mode and echo
    newt.c_lflag &= ~(ICANON | ECHO); // now the input is not buffered and the characters are not dispayed in the terminal

    // Set now (TCSANOW flag) the new settings to the standard input stream
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // Set the new terminal settings

    // Read a single character from the keyboard
    ch = getchar(); 

    // Restore the old terminal settings (now)
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore the old terminal settings

    // Return the character read
    return ch; 

}

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
    std::cout << "Press spacebar to stop the robot" << std::endl;

    // Loop until ros is not closed, we read the key pressed on the keyboard and publish the cmd_vel topic
    while (ros::ok()) {
       
        key = getKey(); // Read the key pressed on the keyboard
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
            case ' ':
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