#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>

using namespace std;


void laserCallback(const sensor_msgs::LaserScan& scan) {
    ROS_INFO("Received string ");
    //vector<Vector2f> endpoints = scanToPoints(scan);  
    //if (first_scan)    buildDistanceMap(endpoints);
    //else               localizer.localize(endpoints, 10);
}



int main(int argc, char** argv) {
    
    ros::init(argc, argv, "LIDARINO_LOCALIZINO");
    ros::NodeHandle n;
    //string topic_name=argv[1];
    string topic_name="sensor_msgs/LaserScan";

    ros::Subscriber sub_main = n.subscribe<const sensor_msgs::LaserScan&>(topic_name, 10, laserCallback);


    while (ros::ok()) {
      ros::spinOnce();
    }
    
   return 0;
}
