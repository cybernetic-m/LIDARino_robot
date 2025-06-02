#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "map_config.h"

using namespace std; 


double vl = 0.0;
double vr = 0.0;



bool new_map=true;
string base_path = "/home/francesco/Documenti/LIDARINO_ROBOT/LIDARino_robot/LIDARINO_WORKSPACE/src/lidarino_pkg/";
string map_yaml_path = base_path + "maps/map.yml";
//string map_yaml_path = base_path + "maps/sim_map.yaml";


void VelocitiesCallback(geometry_msgs::Twist velocities){
    vl = velocities.linear.x;
    vr = velocities.angular.z;
}

int main(int argc, char** argv){

  MapConfig map_config;
  if (!map_config.loadMapParameters(map_yaml_path)) {
      cerr << "Using default odometry parameters" << endl;
  }

  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber cmd_vel = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, VelocitiesCallback);
  ros::Subscriber cmd_sim_vel = n.subscribe<geometry_msgs::Twist>("cmd_sim_vel", 10, VelocitiesCallback);
  tf::TransformBroadcaster odom_broadcaster;

  double x = map_config.origin.x();
  double y = map_config.origin.y();
  double th = 0.0;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = vl * cos(th) * dt;
    double delta_y = vl * sin(th) * dt;
    double delta_th = vr * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vl * cos(th);
    odom.twist.twist.linear.y = vl * sin(th);
    odom.twist.twist.angular.z = vr;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();

    cerr << "x: " << odom_trans.transform.translation.x << " y: " << odom_trans.transform.translation.y << endl;
  }
}