#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <dmap.h>
#include <grid_map.h>
#include <dmap_localizer.h>
#include <draw_helpers.h>

#include <std_msgs/String.h>


using namespace std;


DMap dmap(0,0); 
GridMapping grid_mapping;
DMapLocalizer localizer;
bool first_scan=true;

 

// parameters of the CANVAS to be changed for the moment copied pasted
float resolution=0.05;
float max_range=10;
float expansion_range=1;
Canvas canvas;

////////////

ros::Publisher pos_pub; 

void laserCallback(const sensor_msgs::LaserScan& scan) {
  ROS_INFO("Received string ");

  std::vector<Vector2f> scan_endpoints;
  for (size_t i=0; i<scan.ranges.size(); ++i) {
    float alpha=scan.angle_min+i*scan.angle_increment;
    float r=scan.ranges[i];
    if (r< scan.range_min || r> scan.range_max)
      continue;
    scan_endpoints.push_back(Vector2f(r*cos(alpha), r*sin(alpha)));
  }

  if (first_scan) {
    std::vector<Vector2i> grid_endpoints;
    for (const auto &ep: scan_endpoints) {
      grid_endpoints.push_back(grid_mapping.world2grid(ep).cast<int>());
    }
    dmap.clear();
    int dmax2=pow(expansion_range/resolution, 2);
    int ops=dmap.compute(grid_endpoints, dmax2);
    cerr  << ops;
    Grid_<float> distances;
    dmap.copyTo(distances);
    for (auto& d: distances.cells) {
      d*=resolution;
    }
    localizer.setMap(grid_mapping, distances);
    first_scan=false;
    localizer.X.setIdentity();
  } else {
    float angle=scan.angle_min;
    localizer.localize(scan_endpoints,10);
  }

  localizer.distances.draw(canvas, true); // the end of the rays, the endpoints obviously! 
  for (const auto& ep: scan_endpoints) {
    drawCircle(canvas, grid_mapping.world2grid(localizer.X*ep), 3, 255);
  }

  //added with respect to original code
  Eigen::Vector2f rob_in_wd  = localizer.X.translation();      //position of the robot in the world!          
  Eigen::Vector2f rob_in_gd = grid_mapping.world2grid(rob_in_wd);
  drawCircle(canvas, rob_in_gd, 5, 200);

  Eigen::Vector2f front = rob_in_wd + localizer.X.linear() * Eigen::Vector2f(0.3f, 0.f);


  drawLine(canvas, rob_in_gd, grid_mapping.world2grid(front), 150);  // 150 little bit clearer than black

  showCanvas(canvas,1);


  double x_world = rob_in_wd.x(); 
  double y_world= rob_in_wd.y(); 
  float theta = Eigen::Rotation2Df( localizer.X.linear() ).angle();
  
  /*
  char m[256];
  std::snprintf(m, sizeof(m),
                "x=%.6f  y=%.6f  theta=%.6f",
                x_world, y_world, theta);
  
  std_msgs::String msg;
  msg.data = m;

  //msg.data = std::to_string(x_world)
 
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

  pos_pub.publish(foot);
}



int main(int argc, char** argv) {
    
    ros::init(argc, argv, "LIDARINO_LOCALIZINO");
    ros::NodeHandle n;


    //pos_pub = n.advertise<std_msgs::String>("/POSITION", 10); //added

    pos_pub= n.advertise<geometry_msgs::PolygonStamped>("local_costmap/robot_footprint",10);

    //string topic_name=argv[1];
    string topic_name="LiDAR/LD06";

    int grid_size = 2*(max_range+expansion_range)/resolution;
    dmap.resize(grid_size, grid_size);
    grid_mapping.reset(Vector2f(-grid_size*resolution/2, grid_size*resolution/2), resolution);
    cerr << "grid_size" << grid_size << endl;
    cerr << "world center"  << grid_mapping.world2grid(Vector2f(0,0)).transpose() << endl;
  

    ros::Subscriber sub_main = n.subscribe<const sensor_msgs::LaserScan&>(topic_name, 10, laserCallback);


    while (ros::ok()) {
      ros::spinOnce();
    }
    
   return 0;
}
