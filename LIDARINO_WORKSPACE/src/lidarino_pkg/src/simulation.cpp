#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include "grid_map.h"
#include "laser_scan.h"
#include "laser_scanner.h"
#include "world_item.h"
#include <geometry_msgs/Twist.h>          
#include <geometry_msgs/TwistStamped.h>

using namespace std;




Isometry2f fromCoefficients(float tx, float ty, float alpha) {
    Isometry2f iso;
    iso.setIdentity();
    iso.translation()<< tx, ty;
    iso.linear()=Eigen::Rotation2Df(alpha).matrix();
    return iso;
}



int SCAN_FREQ_HZ = 100;
float DT = 0.1f;
UnicyclePlatform* robot_pointer;

void cmdVelCallback(const geometry_msgs::Twist& cmd){
    robot_pointer->tv =  cmd.linear.x;   
     robot_pointer->rv =  cmd.angular.z;   
}


int main(int argc, char** argv) {


    ros::init(argc, argv, "SCANNERINO_SIMULINO");
    ros::NodeHandle n;
    ros::Publisher pub_scan =n.advertise<sensor_msgs::LaserScan>("LiDAR/LD06", 1);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("simulated_vel", 1);
    ros::Subscriber sub_cmd  = n.subscribe("cmd_vel", 20, cmdVelCallback);
    
    const char* filename = "/home/francesco/Documenti/LIDARINO_ROBOT/LIDARino_robot/LIDARINO_WORKSPACE/src/lidarino_pkg/src/cappero_laser_odom_diag_2020-05-06-16-26-03.png";
    const float resolution = 0.1f;


    GridMap grid_map(resolution,0, 0 );
    grid_map.loadFromImage(filename, resolution);
  
  
    World world_object(grid_map);
    Vector2f grid_middle(grid_map.cols/2, grid_map.rows/2); //106.9 -49.3 for cappero

    Vector2f world_middle = grid_map.grid2world(grid_middle);
    //cerr << "grid_middle is:" << grid_middle << "world middle is:"<< world_middle << endl ; 
    UnicyclePlatform robot(world_object, fromCoefficients(world_middle.x(), world_middle.y(), -0.5));
    robot.radius=0.20;

    robot_pointer=&robot;

    LaserScan scan;
    LaserScanner scanner(scan, robot, fromCoefficients(-0.07, 0, -0));
    scanner.radius = 0.05;
  


    /*

    GridMap grid_map(resolution, 0, 0);
    grid_map.loadFromImage(filename, resolution);
    Eigen::Vector2f center = grid_map.grid2world(grid_map.origin());
    
    
    cerr << "center: " << center.transpose() << endl;
    cerr << "origin: " << grid_map.origin().transpose() << endl;


    //grid_map.draw(canvas);


    // world object definition

    WorldItem* items[3];
    memset(items, 0, sizeof(WorldItem*) * 3);

    World world_object(grid_map);
    items[0] = &world_object;



    Eigen::Isometry2f robot_in_world = Eigen::Isometry2f::Identity();
    robot_in_world.translation() << 5, 0;
    UnicyclePlatform robot(world_object, robot_in_world);
    robot.radius = 1;
    robot.tv = 0;
    robot.rv = 0;
    items[1] = &robot;


    LaserScan scan(range_min, range_max, angle_min,angle_max, ranges_num);
    Isometry2f scanner_in_robot = Eigen::Isometry2f::Identity();
    scanner_in_robot.translation().x() = 0.f;       
    LaserScanner scanner(scan, robot, scanner_in_robot, SCAN_FREQ_HZ);
    scanner.radius = 0.5f;
    items[2] = &scanner;

    
    //...................................................................
    


    */
    
    float range_min=0.1,range_max=10, angle_min=-M_PI/2,angle_max=M_PI/2;
    int ranges_num=180;
    sensor_msgs::LaserScan msg;

    msg.header.frame_id   = "laser";
    
    msg.angle_min        = angle_min;
    msg.angle_max        = angle_max;
    msg.angle_increment  = (angle_max - angle_min) / ranges_num;
    msg.range_min        = range_min;
    msg.range_max        = range_max;
    msg.time_increment   = 1.0f / (SCAN_FREQ_HZ * ranges_num);

    msg.ranges.resize(ranges_num);


   
   
   Canvas canvas;
   ros::Rate rate(SCAN_FREQ_HZ);
   //while (true) { // if intrested only in the simulation 
   while(ros::ok()){
    world_object.tick(DT);          
    scanner.getScan();  
    //grid_map.draw(canvas);

    world_object.draw(canvas); 

    int ret = showCanvas(canvas, DT*100);   // 1 ms waitKey

    if (ret>0)
        cerr << "Key pressed: " << ret << endl;
    switch (ret) {
        case 81:  // left;
            robot.rv += 0.1;
            break;
        case 82:  // up;
            robot.tv += 0.1;
            break;
        case 83:  // right;
            robot.rv -= 0.1;
            break;
        case 84:  // down;
            robot.tv -= 0.1;
            break;
        case 32:  // space;
            robot.tv = 0;
            robot.rv = 0;
        default:;
    }

    

    if (ret == 'q') {
        break;
    }

    msg.header.stamp = ros::Time::now();
    std::copy(scan.ranges.begin(), scan.ranges.end(), msg.ranges.begin());
    
    
    ros::spinOnce();
    pub_scan.publish(msg);

    geometry_msgs::Twist twist_msg;
    //geometry_msgs::TwistStamped twist_msg;
    //twist_msg.header.stamp = ros::Time::now();
    twist_msg.linear.x  = robot.tv;  
    twist_msg.angular.z = robot.rv;   
    pub_vel.publish(twist_msg);
    
   }
    
    return 0;
}


