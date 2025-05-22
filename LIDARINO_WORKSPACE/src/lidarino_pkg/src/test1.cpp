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


#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/OccupancyGrid.h>


using namespace std;


DMap dmap(0,0); 
GridMapping grid_mapping;

DMapLocalizer localizer;
bool localizer_initialized=false;


float resolution, default_resolution = 0.10f;
GridMap grid_map(default_resolution, 0, 0 );
int counter=0;

const char* map_yaml_file = "/home/francesco/Documenti/LIDARINO_ROBOT/LIDARino_robot/LIDARINO_WORKSPACE/src/lidarino_pkg/maps/sim_map.yaml";
const char* map_file= "/home/francesco/Documenti/LIDARINO_ROBOT/LIDARino_robot/""LIDARINO_WORKSPACE/src/lidarino_pkg/src/""cappero_laser_odom_diag_2020-05-06-16-26-03.png";

                              
Canvas canvas; 
ros::Publisher rviz_position_pub;
ros::Publisher string_position_pub;
ros::Publisher pose_pub;
ros::Subscriber laser_scan_sub;
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

    drawCircle(canvas, rob_in_gd, 10, 0);

    Eigen::Vector2f front_dir = rob_in_wd + localizer.X.linear() * Eigen::Vector2f(0.5f, 0.f);


    drawLine(canvas, rob_in_gd, grid_mapping.world2grid(front_dir), 128);  // 128 grey

    showCanvas(canvas,1);
    
    

    //msg.data = std::to_string(x_world);
    char position_string[256];
    std::snprintf(position_string, sizeof(position_string),
                    "x=%.6f  y=%.6f  theta=%.6f",
                    x_world, y_world, theta);
    
    std_msgs::String msg;
    msg.data = position_string;

    string_position_pub.publish(msg);

    //geometry_msgs::PolygonStamped  msg;
    //msg.header.stamp = ros::Time::now();
    //geometry_msgs::Polygon polygon;
    //geometry_msgs::Point32* square {geometry_msgs::Point32(x_world+0.5,y_world+0.5,0),geometry_msgs::Point32(x_world+0.5,y_world-0.5,0),geometry_msgs::Point32(x_world-0.5,y_world-0.5,0),geometry_msgs::Point32(x_world-0.5,y_world+0.5,0),geometry_msgs::Point32(x_world+0.5,y_world+0.5,0)};
    //polygon.points = square;
    //msg.Polygon = polygon;

    geometry_msgs::PolygonStamped foot;
    foot.header.stamp    = ros::Time::now();
    foot.header.frame_id = "map";                 

                        
    Eigen::Rotation2Df R(theta);                  

    array<Eigen::Vector2f,4> corners_square = {{{-0.5f, -0.5f},{ 0.5f, -0.5f},{ 0.5f,  0.5f},  {-0.5f,  0.5f} }};


    foot.polygon.points.reserve(4);

    for (int i=0;i<4;i++) {
        Eigen::Vector2f pp = rob_in_wd + R * corners_square[i];  
        geometry_msgs::Point32 p;
        p.x = pp.x();
        p.y = pp.y();
        p.z = 0.0f;
        foot.polygon.points.push_back(p);
    
    }

    rviz_position_pub.publish(foot);


    geometry_msgs::TransformStamped t;
    t.header.stamp        = ros::Time::now(); 
    t.header.frame_id     = "map";
    t.child_frame_id      = "odom";
    t.transform.translation.x = x_world;
    t.transform.translation.y = y_world;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;   q.setRPY(0, 0, theta);
    t.transform.rotation.x = q.x();   t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();   t.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(t);



    geometry_msgs::PoseStamped pose;
    pose.header   = t.header;
    pose.pose.position.x  = x_world;
    pose.pose.position.y  = y_world;
    pose.pose.orientation = t.transform.rotation;
    pose_pub.publish(pose);
  
}

  

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "LIDARINO_LOCALIZINO");
    ros::NodeHandle n;

    string_position_pub = n.advertise<std_msgs::String>("/POSITION", 10); //added
    rviz_position_pub= n.advertise<geometry_msgs::PolygonStamped>("local_costmap/robot_footprint",10);
    pose_pub  = n.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);

    //laser_scan_sub = n.subscribe<const sensor_msgs::LaserScan&>("scan", 10, laserCallback);
    laser_scan_sub      = n.subscribe("scan",10,laserCallback);

    auto map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(5.0));
    

    resolution = map_msg ? map_msg->info.resolution : default_resolution;


    grid_map.loadFromImage(map_file, resolution);
    Vector2f origin(-grid_map.cols*resolution*0.5f, grid_map.rows*resolution*0.5f);
    grid_map.reset(origin,resolution);
    
    grid_mapping = grid_map;


    cerr << "Map loaded successfully:" << endl;
    cerr << "  Dimensions: " << grid_map.rows << "x" << grid_map.cols << endl;
    cerr << "  Resolution: " << resolution << endl;
    cerr << "  Origin: " << origin << endl;
    cerr << "  Center: " << grid_map.center() << endl;


    grid_map.draw(canvas); 
    showCanvas(canvas,1);

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

