#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <dmap.h>
#include <grid_map.h>
#include <dmap_localizer.h>
#include <draw_helpers.h>

#include <std_msgs/String.h>

#include <tf2/LinearMath/Quaternion.h>      
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/OccupancyGrid.h>
#include "map_config.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>


using namespace std;

DMap dmap(0,0); 
GridMapping grid_mapping;

DMapLocalizer localizer;
bool localizer_initialized=false;


float resolution; 
//float default_resolution = 0.10f;
float default_resolution= 0.05f;

                    

string pkg_name = "lidarino_pkg";
string base_path = ros::package::getPath(pkg_name);
string map_yaml_path = base_path + "/maps/map.yml";
//string map_yaml_path = base_path + "maps/sim_map.yaml";
//string map_file_path = base_path + "/maps/map.pgm";
string map_file_path = base_path + "/maps/map.pgm";

//string map_file_path= base_path+ "maps/cappero_laser_odom_diag_2020-05-06-16-26-03.png";


GridMap grid_map(default_resolution, 0, 0 );
int counter=0;


Canvas canvas; 
int canvas_mode = 3; // 1=original, 2=scaled, 3=cropped+scaled 
float crop_width = 200, crop_height=200, scale=3;


ros::Publisher rviz_position_pub;
ros::Publisher string_position_pub;
ros::Publisher pose_pub;
ros::Subscriber laser_scan_sub;
ros::Publisher amcl_pose_pub;
unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
Isometry2f lmap_pose;



void computeScanEndpoints(std::vector<Vector2f>& dest, const sensor_msgs::LaserScan& scan) {
  dest.clear();
  for (size_t i=0; i<scan.ranges.size(); ++i) {
    float alpha=scan.angle_min+i*scan.angle_increment;
    float r=scan.ranges[i];
    if (r< scan.range_min || r> scan.range_max)
      continue;
    dest.push_back(Vector2f(r*cos(alpha), r*sin(alpha)));
  }
}



void initLocalizer() {

    float influence_range = 2.0f;
    uint8_t occupancy_threshold = 127;  
    
    localizer.setMap(grid_map, influence_range, occupancy_threshold);

    Vector2f map_center = grid_map.center();
    localizer.X.setIdentity();
    localizer.X.translation() = map_center;
    
    localizer_initialized = true;
    cerr << "Localizer initialized. Map is center: " << map_center << endl;
}



void laserCallback(const sensor_msgs::LaserScan& scan) {
    grid_map.draw(canvas); 
    std::vector<Vector2f> scan_endpoints;
    computeScanEndpoints(scan_endpoints, scan);

    if (!localizer_initialized) {
        initLocalizer();
    } 


    bool localization_success=localizer.localize(scan_endpoints,10);
    if (!localization_success) {
        cerr << "Localization failed" << endl;
        return;
    }

    for (const auto& ep: scan_endpoints) {
        drawCircle(canvas, grid_mapping.world2grid(localizer.X*ep), 2, 100);
    }


    //ROBOT IN WORD COORDINATES
    Eigen::Vector2f rob_in_wd  = localizer.X.translation();              
    Eigen::Vector2f rob_in_gd = grid_mapping.world2grid(rob_in_wd);
    double x_world = rob_in_wd.x(); 
    double y_world= rob_in_wd.y(); 
    float theta = Eigen::Rotation2Df( localizer.X.linear() ).angle();

    drawCircle(canvas, rob_in_gd, 2.5, 0);

    Eigen::Vector2f front_dir = rob_in_wd + localizer.X.linear() * Eigen::Vector2f(0.25f, 0.f);


    drawLine(canvas, rob_in_gd, grid_mapping.world2grid(front_dir), 128);  // 128 grey


    //showCanvas(canvas,1);
    //showScaledCanvas(canvas, 0.4f, 1);
    showCanvasMode(canvas, canvas_mode, crop_width, crop_height, scale, 1);


    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    Eigen::Rotation2Df R(theta);
    const std::string map_frame = "map";

    

    //msg.data = std::to_string(x_world);
    char position_string[256];
    std::snprintf(position_string, sizeof(position_string),
                    "x=%.6f  y=%.6f  theta=%.6f",
                    x_world, y_world, theta);
    
    std_msgs::String msg;
    msg.data = position_string;

    string_position_pub.publish(msg);


    geometry_msgs::PolygonStamped robot_foot;
    robot_foot.header.stamp    = ros::Time::now();
    robot_foot.header.frame_id = map_frame;     
                     

    array<Eigen::Vector2f,4> corners_square = {{{-0.1f, -0.1f},{ 0.1f, -0.1f},{ 0.1f,  0.1f},  {-0.1f,  0.1f} }};

    for (int i=0;i<4;i++) {
        Eigen::Vector2f pp = rob_in_wd + R * corners_square[i];  
        geometry_msgs::Point32 p;
        p.x = pp.x();
        p.y = pp.y();
        p.z = 0.0f;
        robot_foot.polygon.points.push_back(p);
    
    }

    rviz_position_pub.publish(robot_foot);


    geometry_msgs::TransformStamped t;
    t.header.stamp = ros::Time::now();
    t.header.frame_id = map_frame;  
    t.child_frame_id = "odom";
    t.transform.translation.x = x_world;
    t.transform.translation.y = y_world;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x();  
    t.transform.rotation.y = q.y();  
    t.transform.rotation.z = q.z();  
    t.transform.rotation.w = q.w();


    tf_broadcaster->sendTransform(t);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = map_frame;
    pose.pose.position.x = x_world;
    pose.pose.position.y = y_world;
    pose.pose.position.z = 0.0;  

    pose.pose.orientation.x = q.x(); 
    pose.pose.orientation.y = q.y(); 
    pose.pose.orientation.z = q.z();  
    pose.pose.orientation.w = q.w();  
    pose_pub.publish(pose);
    
    geometry_msgs::PoseWithCovarianceStamped amcl_pose;
    amcl_pose.header.stamp = ros::Time::now();
    amcl_pose.header.frame_id = map_frame;
    amcl_pose.pose.pose.position.x = x_world;
    amcl_pose.pose.pose.position.y = y_world;
    amcl_pose.pose.pose.position.z = 0.0;

    amcl_pose.pose.pose.orientation.x = q.x(); 
    amcl_pose.pose.pose.orientation.y = q.y();  
    amcl_pose.pose.pose.orientation.z = q.z();  
    amcl_pose.pose.pose.orientation.w = q.w(); 
    for(int i = 0; i < 36; i++) {
        amcl_pose.pose.covariance[i] = 0.0;
    }

    amcl_pose.pose.covariance[0] = 0.25;  
    amcl_pose.pose.covariance[7] = 0.25;    
    amcl_pose.pose.covariance[35] = 0.17;  
    
    amcl_pose_pub.publish(amcl_pose);



}

  

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "LIDARINO_LOCALIZINO");
    ros::NodeHandle n;

    string_position_pub = n.advertise<std_msgs::String>("/POSITION", 10); 
    rviz_position_pub= n.advertise<geometry_msgs::PolygonStamped>("local_costmap/robot_footprint",10);
    pose_pub  = n.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
    amcl_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10);

    //laser_scan_sub = n.subscribe<const sensor_msgs::LaserScan&>("scan", 10, laserCallback);
    laser_scan_sub = n.subscribe("LiDAR/LD06",10,laserCallback);

    auto map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(5.0));
    

    resolution = map_msg ? map_msg->info.resolution : default_resolution;


    grid_map.loadFromImage(map_file_path.c_str(), resolution);
    Vector2f origin(-grid_map.cols*resolution*0.5f, grid_map.rows*resolution*0.5f);
    grid_map.reset(origin,resolution);
    
    grid_mapping = grid_map;


    cerr << "Map loaded successfully:" << endl;
    cerr << " Dimensions: " << grid_map.rows << "x" << grid_map.cols << endl;
    cerr << " Resolution: " << resolution << endl;
    cerr << " Center: " << grid_map.center() << endl;


    grid_map.draw(canvas); 

    //showCanvas(canvas,1);
    //showScaledCanvas(canvas, 0.4f, 1);
    showCanvasMode(canvas, canvas_mode, crop_width, crop_height, scale, 1);



    tf_broadcaster = make_unique<tf2_ros::TransformBroadcaster>();
    
    //lmap_pose.setIdentity();

    cerr << "LIDARINO_LOCALIZINO node initialized and ready" << endl;

    ros::Rate r(10.0);
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }
    
   return 0;
}