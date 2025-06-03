
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>
#include <cmath>

#include <dmap.h>
#include <grid_map.h>
#include <dmap_localizer.h>

using namespace std;
using Eigen::Vector2f;
using Eigen::Vector2i;

/* ---------- variabili globali (comode per il callback) --------------- */
DMap           dmap(0,0);
GridMapping    grid_mapping;
DMapLocalizer  localizer;
bool           first_scan = true;

float  resolution       = 0.05f;   // m/cell
float  max_range        = 10.0f;   // m
float  expansion_range  = 1.0f;    // m

Canvas canvas;

/*  Publisher della X: deve essere visibile dal callback */
ros::Publisher pos_pub;

/* -------------------- Callback LIDAR --------------------------------- */
void laserCallback(const sensor_msgs::LaserScan& scan) {

  /* 1. Estrae end-points in coordinate world ------------------------- */
  std::vector<Vector2f> scan_endpoints;
  scan_endpoints.reserve(scan.ranges.size());

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float r = scan.ranges[i];
    if (r < scan.range_min || r > scan.range_max)
      continue;
    float alpha = scan.angle_min + i * scan.angle_increment;
    scan_endpoints.emplace_back(r * cos(alpha), r * sin(alpha));
  }

  /* 2. Prima scansione → costruisci DMap ----------------------------- */
  if (first_scan) {
    std::vector<Vector2i> grid_endpoints;
    grid_endpoints.reserve(scan_endpoints.size());
    for (const auto& ep : scan_endpoints)
      grid_endpoints.push_back(grid_mapping.world2grid(ep).cast<int>());

    dmap.clear();
    int dmax2 = std::pow(expansion_range / resolution, 2);
    dmap.compute(grid_endpoints, dmax2);

    Grid_<float> distances;
    dmap.copyTo(distances);
    for (auto& d : distances.cells)
      d *= resolution;                       // metri

    localizer.setMap(grid_mapping, distances);
    localizer.X.setIdentity();               // (0,0,0)
    first_scan = false;
  }
  else {
    localizer.localize(scan_endpoints, 10);
  }

  /* 3. Posa stimata → X in metri ------------------------------------- */
  float x_world = localizer.X.translation().x();

  /* 4. Pubblica X come stringa su /POSITION -------------------------- */
  std_msgs::String x_msg;
  x_msg.data = std::to_string(x_world);
  pos_pub.publish(x_msg);

  /* 5. Disegno su canvas -------------------------------------------- */
  localizer.distances.draw(canvas, true);

  /* raggi */
  for (const auto& ep : scan_endpoints)
    drawCircle(canvas, grid_mapping.world2grid(localizer.X * ep), 3, 255);

  /* robot (cerchio più grosso, colore 200/255) */
  drawCircle(canvas,
             grid_mapping.world2grid(localizer.X.translation()),
             5, 200);

  showCanvas(canvas, 1);  // waitKey(1 ms)
}

/* ------------------------------ main --------------------------------- */
int main(int argc, char** argv) {

  ros::init(argc, argv, "lidar_localizer_sub");
  ros::NodeHandle nh;

  /* 1. Griglia interna per DMap ------------------------------------- */
  int grid_size = static_cast<int>( 2 * (max_range + expansion_range) /
                                    resolution );
  dmap.resize(grid_size, grid_size);
  grid_mapping.reset(Vector2f(-grid_size * resolution / 2,
                               grid_size * resolution / 2),
                     resolution);

  cerr << "grid_size:   " << grid_size << endl
       << "world center:" << grid_mapping.world2grid(Vector2f(0,0)).transpose()
       << endl;

  /* 2. Subscriber LIDAR --------------------------------------------- */
  std::string topic_scan = "/scan";           // o quello che usi nel publisher
  ros::Subscriber sub_scan =
      nh.subscribe(topic_scan, 10, laserCallback);

  /* 3. Publisher posizione X ---------------------------------------- */
  pos_pub = nh.advertise<std_msgs::String>("/POSITION", 10);

  /* 4. Loop ROS ------------------------------------------------------ */
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}
