#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_publisher");

  ros::NodeHandle nh;

  ros::Publisher pub_handle =
      nh.advertise<std_msgs::String>("/publisher/magic_numbers", 10);

  unsigned long counter = 0;

  ros::Rate rate(10.f);
  while (ros::ok()) {
    std_msgs::String message_to_send;
    message_to_send.data = std::to_string(counter);
    counter++;

    ROS_INFO("Publishing a message");
    pub_handle.publish(message_to_send);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}