#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include "grid_map.h"
#include "laser_scan.h"
#include "laser_scanner.h"
#include "world_item.h"
#include <geometry_msgs/Twist.h>          
#include <geometry_msgs/TwistStamped.h>

#include <ros/package.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  
#include <tf2/LinearMath/Quaternion.h>
#include "map_config.h" 




using namespace std;

//map parameters
//const float resolution = 0.1f;
const float resolution = 0.05f;
string pkg_name = "lidarino_pkg";
string base_path = ros::package::getPath(pkg_name);
string map_yaml_path = base_path + "/maps/map.yml";
//string map_yaml_path = base_path + "maps/sim_map.yaml";
string map_file_path = base_path + "/maps/map.pgm";
//string map_file_path= base_path+ "maps/cappero_laser_odom_diag_2020-05-06-16-26-03.png";
//float default_or_x=-51.200024f, default_or_y=-51.200024f; map
//106.9 -49.3 for cappero


//World parameters 
float theta_initial=0;
float robot_radius=0.15;
float scanner_radius= 0.05;
float x_offset=-0.05,y_offset=0;
UnicyclePlatform* robot_pointer;

//SIMULATION PARAMETERS
int SCAN_FREQ_HZ = 10;
float DT = 0.1f;


//SCAN PARAMETERS
float range_min=0.1,range_max=10, angle_min=-M_PI/2,angle_max=M_PI/2;
int ranges_num=180;


//New Canvas Parameters
int canvas_mode = 3;  // 1=original, 2=scaled, 3=cropped+scaled
float crop_width = 200, crop_height=200, scale=3;


Isometry2f fromCoefficients(float tx, float ty, float alpha) {
    Isometry2f iso;
    iso.setIdentity();
    iso.translation()<< tx, ty;
    iso.linear()=Eigen::Rotation2Df(alpha).matrix();
    return iso;
}


void cmdVelCallback(const geometry_msgs::Twist& cmd){
    robot_pointer->tv =  cmd.linear.x;   
    robot_pointer->rv =  cmd.angular.z;   
}


int main(int argc, char** argv) {


    ros::init(argc, argv, "SCANNERINO_SIMULINO");
    ros::NodeHandle n;

    ros::Publisher pub_initial_pose =n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1,true);
    ros::Publisher pub_scan =n.advertise<sensor_msgs::LaserScan>("LiDAR/LD06", 1);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("cmd_sim_vel", 1);

    ros::Subscriber sub_cmd  = n.subscribe("cmd_vel", 20, cmdVelCallback);




    //MapConfig map_config;

    //if (!map_config.loadMapParameters(map_yaml_path)) {
    //    map_config.image_file = base_path+'map/'+map_file_path;
    //    map_config.resolution = resolution;
    //    map_config.origin = Eigen::Vector2f(default_or_x, default_or_y);
    //}
    //GridMap grid_map(map_config.resolution, 0, 0);
    //grid_map.loadFromImage(full_map_path.c_str(), map_config.resolution);
    //grid_map.reset(map_config.origin, map_config.resolution);
    



    GridMap grid_map(resolution, 0, 0 );
    grid_map.loadFromImage(map_file_path.c_str(), resolution);

    
    World world_object(grid_map);
    Vector2f grid_middle(grid_map.cols/2, grid_map.rows/2); //106.9 -49.3 for cappero
    Vector2f world_middle = grid_map.grid2world(grid_middle);
    cerr << "grid_middle is:" << grid_middle << "world middle is:"<< world_middle << endl ; 

    UnicyclePlatform robot(world_object, fromCoefficients(world_middle.x(), world_middle.y(), theta_initial));
    robot.radius=robot_radius;
    robot_pointer=&robot;

    LaserScan scan;
    LaserScanner scanner(scan, robot, fromCoefficients(x_offset, y_offset, -0));
    scanner.radius = scanner_radius;
  

    /*

    GridMap grid_map(resolution, 0, 0);
    grid_map.loadFromImage(map_file_path..c_str(), resolution);
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

    geometry_msgs::PoseWithCovarianceStamped init_position;
    init_position.header.frame_id = "/map";
    init_position.pose.pose.position.x = world_middle.x();
    init_position.pose.pose.position.y = world_middle.y();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_initial); 
    init_position.pose.pose.orientation = tf2::toMsg(q);
    init_position.pose.covariance[0]  = 0.25;    
    init_position.pose.covariance[7]  = 0.25;    
    init_position.pose.covariance[35] = 0.17; 
    pub_initial_pose.publish(init_position);


    sensor_msgs::LaserScan msg;

    msg.header.frame_id   = "lidar_frame";

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

    //int ret = showCanvas(canvas, DT*10);   // 1 ms waitKey
    //int ret = showScaledCanvas(canvas, 0.6f, DT*10);   
    int ret = showCanvasMode(canvas, canvas_mode, crop_width, crop_height, scale, DT*10);

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


