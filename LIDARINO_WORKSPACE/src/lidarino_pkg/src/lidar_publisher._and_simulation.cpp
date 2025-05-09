
#include <iostream>

#include "grid_map.h"
#include "laser_scan.h"
#include "laser_scanner.h"
#include "world_item.h"


using namespace std;

int main(int argc, char** argv) {

    const char* filename = "/home/francesco/Documenti/LIDARINO_ROBOT/LIDARino_robot/LIDARINO_WORKSPACE/src/lidarino_pkg/src/cappero_laser_odom_diag_2020-05-06-16-26-03.png";
    const int resolution= 0.01;

    GridMap grid_map(0, 0, 0.1);
    grid_map.loadFromImage(filename, resolution);
    Canvas canvas;
    Eigen::Vector2f center = grid_map.grid2world(grid_map.origin());
    cerr << "center: " << center.transpose() << endl;
    cerr << "origin: " << grid_map.origin().transpose() << endl;

    grid_map.draw(canvas);

    WorldItem* items[100];
    memset(items, 0, sizeof(WorldItem*) * 100);

    World world_object(grid_map);
    items[0] = &world_object;

    Eigen::Isometry2f robot_in_world = Eigen::Isometry2f::Identity();
    robot_in_world.translation() << 5,-1;
    UnicyclePlatform robot(world_object, robot_in_world);
    robot.radius = 1;
    robot.tv = 0;
    robot.rv = 0;
    items[1] = &robot;


    while (true) {
        grid_map.draw(canvas);
        world_object.tick(0.01f);          
        world_object.draw(canvas, false);

        int ret = showCanvas(canvas, 0);   // 1 ms waitKey
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
        
        // Add a check to break out of the loop on 'q' key press
        if (ret == 'q' || ret == 'Q') {
            break;
        }
    }
    
    return 0;
}
















/*****************************************************************
 * robot_simulator_node.cpp
 * ---------------------------------------------------------------
 * Simulatore semplice di robot differenziale con LIDAR.
 * - Frecce ← ↑ → ↓   modificano rv / tv
 * - Spazio           ferma il robot
 * - 'q'              chiude il programma
 *
 * Publishes:
 *   /scan     (sensor_msgs/LaserScan)
 *****************************************************************/

 #include <ros/ros.h>
 #include <sensor_msgs/LaserScan.h>
 
 #include <iostream>
 #include <thread>
 #include <chrono>
 
 #include "grid_map.h"
 #include "laser_scan.h"
 #include "laser_scanner.h"
 #include "world_item.h"
 
 using namespace std;
 using Eigen::Vector2f;
 using Eigen::Vector2i;
 using Eigen::Isometry2f;
 using Eigen::Rotation2Df;
 
 /* ---------------------------------------------------------------
  *  Parametri (modifica a piacere o leggi da argv / param server)
  * --------------------------------------------------------------- */
 static const char*  MAP_FILE      = "/home/francesco/Documenti/LIDARINO_ROBOT/"
                                     "LIDARino_robot/LIDARINO_WORKSPACE/"
                                     "src/lidarino_pkg/src/"
                                     "cappero_laser_odom_diag_2020-05-06-16-26-03.png";
 static const float  MAP_RES       = 0.01f;      // m/pixel
 static const int    BEAMS         = 180;        // numero raggi
 static const float  RANGE_MIN     = 0.10f;      // m
 static const float  RANGE_MAX     = 10.0f;      // m
 static const float  FOV           = M_PI;       // 180°
 static const float  SCAN_FREQ_HZ  = 10.0f;      // Hz
 static const float  DT            = 1.0f/SCAN_FREQ_HZ;
 
 /* =============================================================== */
 int main(int argc, char** argv)
 {
   /* ----------------- ROS set-up -------------------------------- */
   ros::init(argc, argv, "robot_simulator");
   ros::NodeHandle nh;
   ros::Publisher pub_scan =
       nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
 
   /* ----------------- Mappa e mondo ----------------------------- */
   GridMap grid_map(MAP_RES, 0, 0);
   grid_map.loadFromImage(MAP_FILE, MAP_RES);
   Canvas canvas;
 
   World world(grid_map);
 
   /* ----------------- Robot (unicycle) -------------------------- */
   Isometry2f robot_pose = Isometry2f::Identity();
   robot_pose.translation() << 5.f, -1.f;
 
   UnicyclePlatform robot(world, robot_pose);
   robot.radius = 1.f;
   robot.tv     = 0.f;
   robot.rv     = 0.f;
 
   /* ----------------- Scanner ----------------------------------- */
   LaserScan scan(RANGE_MIN, RANGE_MAX,
                  -FOV/2,  FOV/2,
                  BEAMS);
 
   Isometry2f scanner_in_robot = Isometry2f::Identity();
   scanner_in_robot.translation().x() = 0.f;       // LIDAR al centro
   LaserScanner scanner(scan, robot, scanner_in_robot, SCAN_FREQ_HZ);
 
   /* ----------------- Messaggio ROS pre-compilato --------------- */
   sensor_msgs::LaserScan msg;
   msg.header.frame_id   = "laser";
   msg.range_min         = RANGE_MIN;
   msg.range_max         = RANGE_MAX;
   msg.angle_min         = -FOV/2;
   msg.angle_max         =  FOV/2;
   msg.angle_increment   = FOV / BEAMS;
   msg.ranges.resize(BEAMS);
 
   /* ----------------- Main loop --------------------------------- */
   ros::Rate rate(SCAN_FREQ_HZ);
   while (ros::ok()) {
 
     /* 1. Fisica/kinematica ------------------------------------ */
     world.tick(DT);     // integra robot, scanner, ecc.
 
     /* 2. Aggiorna messaggio scan ------------------------------ */
     msg.header.stamp = ros::Time::now();
     std::copy(scan.ranges.begin(), scan.ranges.end(), msg.ranges.begin());
     pub_scan.publish(msg);
 
     /* 3. Disegno --------------------------------------------- */
     grid_map.draw(canvas);
     world.draw(canvas, false);           // include robot e raggi
 
     /* cerchio per evidenziare la posa del robot */
     drawCircle(canvas,
                grid_map.world2grid(robot.globalPose().translation()),
                5, 200);
 
     /* raggi (opz. – LaserScan::draw è già in world.draw se previsto) */
     // scan.draw(canvas, grid_map, robot.globalPose());
 
     int key = showCanvas(canvas, 1);     // 1 ms waitKey
     switch (key) {
       case 81: robot.rv += 0.2f; break;   // ←
       case 83: robot.rv -= 0.2f; break;   // →
       case 82: robot.tv += 0.2f; break;   // ↑
       case 84: robot.tv -= 0.2f; break;   // ↓
       case 32: robot.tv  = robot.rv = 0; break;   // spazio
       case 'q':
       case 'Q': return 0;
       default:;
     }
 
     ros::spinOnce();
     rate.sleep();   // mantiene SCAN_FREQ_HZ
   }
   return 0;
 }









  const Isometry2 forward(0.1, 0, 0);
  const Isometry2 backward(-0.1, 0, 0);
  const Isometry2 left(0, 0, 0.1);
  const Isometry2 right(0, 0, -0.1);

  while (true) {
    scanner.getScan();
    grid_map.draw(canvas);
    drawItems(canvas, items);
    // scan.draw(canvas, grid_map, object_1.globalPose());
    int ret = showCanvas(canvas, 0);
    std::cerr << "Key pressed: " << ret << std::endl;

    Isometry2 motion_iso = Isometry2::Identity();
    switch (ret) {
      case 81:  // left;
        motion_iso = left;
        break;
      case 82:  // up;
        motion_iso = forward;
        break;
      case 83:  // right;
        motion_iso = right;
        break;
      case 84:  // down;
        motion_iso = backward;
        break;
      default:;
    }
    object_1.move(motion_iso);
  }




 #include <ros/ros.h>
 #include <sensor_msgs/LaserScan.h>
 
 #include <iostream>
 #include <thread>
 #include <chrono>
 
 #include "grid_map.h"
 #include "laser_scan.h"
 #include "laser_scanner.h"
 #include "world_item.h"
 
 using namespace std;
 using Eigen::Vector2f;
 using Eigen::Vector2i;
 using Eigen::Isometry2f;
 using Eigen::Rotation2Df;
 
 /* ---------------------------------------------------------------
  *  Parametri (modifica a piacere o leggi da argv / param server)
  * --------------------------------------------------------------- */
 static const char*  MAP_FILE      = "/home/francesco/Documenti/LIDARINO_ROBOT/"
                                     "LIDARino_robot/LIDARINO_WORKSPACE/"
                                     "src/lidarino_pkg/src/"
                                     "cappero_laser_odom_diag_2020-05-06-16-26-03.png";
 static const float  MAP_RES       = 0.01f;      // m/pixel
 static const int    BEAMS         = 180;        // numero raggi
 static const float  RANGE_MIN     = 0.10f;      // m
 static const float  RANGE_MAX     = 10.0f;      // m
 static const float  FOV           = M_PI;       // 180°
 static const float  SCAN_FREQ_HZ  = 10.0f;      // Hz
 static const float  DT            = 1.0f/SCAN_FREQ_HZ;
 
 /* =============================================================== */
 int main(int argc, char** argv)
 {
   /* ----------------- ROS set-up -------------------------------- */
   ros::init(argc, argv, "robot_simulator");
   ros::NodeHandle nh;
   ros::Publisher pub_scan =
       nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
 
   /* ----------------- Mappa e mondo ----------------------------- */
   GridMap grid_map(MAP_RES, 0, 0);
   grid_map.loadFromImage(MAP_FILE, MAP_RES);
   Canvas canvas;
 
   World world(grid_map);
 
   /* ----------------- Robot (unicycle) -------------------------- */
   Isometry2f robot_pose = Isometry2f::Identity();
   robot_pose.translation() << 5.f, -1.f;
 
   UnicyclePlatform robot(world, robot_pose);
   robot.radius = 1.f;
   robot.tv     = 0.f;
   robot.rv     = 0.f;
 
   /* ----------------- Scanner ----------------------------------- */
   LaserScan scan(RANGE_MIN, RANGE_MAX,
                  -FOV/2,  FOV/2,
                  BEAMS);
 
   Isometry2f scanner_in_robot = Isometry2f::Identity();
   scanner_in_robot.translation().x() = 0.f;       // LIDAR al centro
   LaserScanner scanner(scan, robot, scanner_in_robot, SCAN_FREQ_HZ);
 
   /* ----------------- Messaggio ROS pre-compilato --------------- */
   sensor_msgs::LaserScan msg;
   msg.header.frame_id   = "laser";
   msg.range_min         = RANGE_MIN;
   msg.range_max         = RANGE_MAX;
   msg.angle_min         = -FOV/2;
   msg.angle_max         =  FOV/2;
   msg.angle_increment   = FOV / BEAMS;
   msg.ranges.resize(BEAMS);
 
   /* ----------------- Main loop --------------------------------- */
   ros::Rate rate(SCAN_FREQ_HZ);
   while (ros::ok()) {
 
     /* 1. Fisica/kinematica ------------------------------------ */
     world.tick(DT);     // integra robot, scanner, ecc.
 
     /* 2. Aggiorna messaggio scan ------------------------------ */
     msg.header.stamp = ros::Time::now();
     std::copy(scan.ranges.begin(), scan.ranges.end(), msg.ranges.begin());
     pub_scan.publish(msg);
 
     /* 3. Disegno --------------------------------------------- */
     grid_map.draw(canvas);
     world.draw(canvas, false);           // include robot e raggi
 
     /* cerchio per evidenziare la posa del robot */
     drawCircle(canvas,
                grid_map.world2grid(robot.globalPose().translation()),
                5, 200);
 
     /* raggi (opz. – LaserScan::draw è già in world.draw se previsto) */
     // scan.draw(canvas, grid_map, robot.globalPose());
 
     int key = showCanvas(canvas, 1);     // 1 ms waitKey
     switch (key) {
       case 81: robot.rv += 0.2f; break;   // ←
       case 83: robot.rv -= 0.2f; break;   // →
       case 82: robot.tv += 0.2f; break;   // ↑
       case 84: robot.tv -= 0.2f; break;   // ↓
       case 32: robot.tv  = robot.rv = 0; break;   // spazio
       case 'q':
       case 'Q': return 0;
       default:;
     }
 
     ros::spinOnce();
     rate.sleep();   // mantiene SCAN_FREQ_HZ
   }
   return 0;
 }
 