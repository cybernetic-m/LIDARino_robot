#include <ros/ros.h>
#include <std_msgs/String.h>



 void positionCallback(const std_msgs::StringConstPtr& msg) {
 
    ROS_INFO("Received position: %s", msg->data.c_str());

 }
 
 int main(int argc, char** argv) {
 
   ros::init(argc, argv, "LISTERINO_POSIZIONINO_NODINO");
   ros::NodeHandle n;
  
   ros::Subscriber sub_pos = n.subscribe<std_msgs::String>("/POSITION", 10, positionCallback); // remember 10 queue size
 
   ros::spin();   // the code just do this, it's not necessary to do   while (ros::ok()) {ros::spinOnce();}
   return 0;
 }