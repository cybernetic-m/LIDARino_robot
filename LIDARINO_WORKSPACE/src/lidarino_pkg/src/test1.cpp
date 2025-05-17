#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>

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

using namespace std;


DMap dmap(0,0); 
GridMapping grid_mapping;
DMapLocalizer localizer;
bool first_scan=true;

int counter=0;

// parameters of the MAP to be changed for the moment copied pasted
float resolution=0.05;
float max_range=10;
float expansion_range=1;
Canvas canvas;
Isometry2f lmap_pose;
///

ros::Publisher abs_position_pub; 
ros::Publisher pose_pub;      

unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;  




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

void computeGridEndpoints(std::vector<Vector2i>& dest, const std::vector<Vector2f>& src) {
  dest.clear();
  for (const auto &ep: src) {
    dest.push_back(grid_mapping.world2grid(ep).cast<int>());
  }
}

void initLocalizer(std::vector<Vector2f>& scan_endpoints) {
    std::vector<Vector2i> grid_endpoints;
    computeGridEndpoints(grid_endpoints, scan_endpoints);
    dmap.clear();
    int dmax2=pow(expansion_range/resolution, 2);
    int ops=dmap.compute(grid_endpoints, dmax2);
    cerr << "refresh n."  << counter++ << endl;
    Grid_<float> distances;
    dmap.copyTo(distances);
    for (auto& d: distances.cells) {
      d*=resolution;
    }
    localizer.setMap(grid_mapping, distances);
}

void laserCallback(const sensor_msgs::LaserScan& scan) {
  
  std::vector<Vector2f> scan_endpoints;
  computeScanEndpoints(scan_endpoints, scan);

  if (first_scan) {
    initLocalizer(scan_endpoints);
    first_scan=false;
    localizer.X.setIdentity();
  } 

  else{
    float angle=scan.angle_min;
    localizer.localize(scan_endpoints,10);
    float translation_norm=localizer.X.translation().norm();
    float orientation_norm=fabs(Eigen::Rotation2Df(localizer.X.linear()).angle());
    //printf("trans %f\n",translation_norm);
    //printf("orient %f\n", orientation_norm);
    if (translation_norm>0.1 || orientation_norm>0.5) {

      lmap_pose = lmap_pose*localizer.X;
      initLocalizer(scan_endpoints);
    }
  }


  localizer.distances.draw(canvas, true); 
  
  for (const auto& ep: scan_endpoints) {
    drawCircle(canvas, grid_mapping.world2grid(localizer.X*ep), 3, 0);
  }


  //added with respect to original code
  Eigen::Vector2f rob_in_wd  = localizer.X.translation();              
  Eigen::Vector2f rob_in_gd = grid_mapping.world2grid(rob_in_wd);
  double x_world = rob_in_wd.x(); 
  double y_world= rob_in_wd.y(); 
  float theta = Eigen::Rotation2Df( localizer.X.linear() ).angle();

  drawCircle(canvas, rob_in_gd, 5, 0);

  Eigen::Vector2f front = rob_in_wd + localizer.X.linear() * Eigen::Vector2f(0.3f, 0.f);


  drawLine(canvas, rob_in_gd, grid_mapping.world2grid(front), 128);  // 128 grey

  showCanvas(canvas,1);
 
  /*
  char m[256];
  std::snprintf(m, sizeof(m),
                "x=%.6f  y=%.6f  theta=%.6f",
                x_world, y_world, theta);
  
  std_msgs::String msg;
  msg.data = m;

  //msg.data = std::to_string(x_world)
  
  geometry_msgs::PolygonStamped  msg;

  msg.header.stamp = ros::Time::now();
  
  geometry_msgs::Polygon polygon;


  geometry_msgs::Point32* square {geometry_msgs::Point32(x_world+0.25,y_world+0.25,0),geometry_msgs::Point32(x_world+0.25,y_world-0.25,0),geometry_msgs::Point32(x_world-0.25,y_world-0.25,0),geometry_msgs::Point32(x_world-0.25,y_world+0.25,0),geometry_msgs::Point32(x_world+0.25,y_world+0.25,0)};

  polygon.points = square;

  msg.Polygon = polygon;
  */







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

  abs_position_pub.publish(foot);


  geometry_msgs::TransformStamped t;
  t.header.stamp        = scan.header.stamp; 
  t.header.frame_id     = "odom";
  t.child_frame_id      = "base_link";
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


    //abs_position_pub = n.advertise<std_msgs::String>("/POSITION", 10); //added

    abs_position_pub= n.advertise<geometry_msgs::PolygonStamped>("local_costmap/robot_footprint",10);
    
    //string topic_name=argv[1];
    string topic_name="LiDAR/LD06";

    //
    int grid_size = 2*(max_range+expansion_range)/resolution;
    dmap.resize(grid_size, grid_size);
    grid_mapping.reset(Vector2f(-grid_size*resolution/2, grid_size*resolution/2), resolution);
    cerr << "grid_size" << grid_size << endl;
    cerr << "world center"  << grid_mapping.world2grid(Vector2f(0,0)).transpose() << endl;
    

    lmap_pose.setIdentity();
    ros::Subscriber sub_main = n.subscribe<const sensor_msgs::LaserScan&>(topic_name, 10, laserCallback);

    tf_broadcaster =  make_unique<tf2_ros::TransformBroadcaster>();
    pose_pub  = n.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);

    while (ros::ok()) {
      ros::spinOnce();
    }
    
   return 0;
}