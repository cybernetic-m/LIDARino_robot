#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include "grid_map.h"
#include "laser_scan.h"
#include "laser_scanner.h"
#include "world_item.h"


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




int main(int argc, char** argv) {


    ros::init(argc, argv, "SCANNERINO_SIMULINO");
    ros::NodeHandle n;
    ros::Publisher pub_scan =n.advertise<sensor_msgs::LaserScan>("LiDAR/LD06", 1);

    const char* filename = "/home/francesco/Documenti/LIDARINO_ROBOT/LIDARino_robot/LIDARINO_WORKSPACE/src/lidarino_pkg/src/cappero_laser_odom_diag_2020-05-06-16-26-03.png";
    const float resolution= 0.1f;


    GridMap grid_map(0, 0, 0.1);
    grid_map.loadFromImage(filename, resolution);
  
  
    World world_object(grid_map);
    WorldItem object_0(world_object, fromCoefficients(5, 0, 0.5));
    Vector2f grid_middle(grid_map.cols/2, grid_map.rows/2);
    Vector2f world_middle = grid_map.grid2world(grid_middle);
    UnicyclePlatform robot(world_object, fromCoefficients(world_middle.x(), world_middle.y(), -0.5));
    robot.radius=1;
  
    LaserScan scan;
    LaserScanner scanner(scan, robot, fromCoefficients(3, 0, -0));
    scanner.radius = 0.5;
  
    float dt=0.1;

    /*

    GridMap grid_map(resolution, 0, 0);
    grid_map.loadFromImage(filename, resolution);
    Eigen::Vector2f center = grid_map.grid2world(grid_map.origin());
    
    
    cerr << "center: " << center.transpose() << endl;
    cerr << "origin: " << grid_map.origin().transpose() << endl;


    //grid_map.draw(canvas);


    // definizione di tutti gli oggetti del mondo

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

    


    
    //...................................................................
    

    LaserScan scan(range_min, range_max, angle_min,angle_max, ranges_num);
    Isometry2f scanner_in_robot = Eigen::Isometry2f::Identity();
    scanner_in_robot.translation().x() = 0.f;       
    LaserScanner scanner(scan, robot, scanner_in_robot, SCAN_FREQ_HZ);
    scanner.radius = 0.5f;
    items[2] = &scanner;
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

    

    
   //const Iso2f forward  = Iso2f( Eigen::Translation2f( 0.1f,  0.f) );
   //const Iso2f backward = Iso2f( Eigen::Translation2f(-0.1f,  0.f) );
   
   //Iso2f left  = Iso2f::Identity();
   //Iso2f right = Iso2f::Identity();
   //left.rotate ( Rot2f( 0.1f) );
   //right.rotate( Rot2f(-0.1f) );
 

   
   //const Isometry2 forward(0.1, 0, 0);
   //const Isometry2 backward(-0.1, 0, 0);
   //const Isometry2 left(0, 0, 0.1);
   //const Isometry2 right(0, 0, -0.1);
   
   
   Canvas canvas;
   ros::Rate rate(SCAN_FREQ_HZ);
   //while (ros::ok()) { // true if intrested only in the simulation 
   while(ros::ok()){
    world_object.tick(DT);          
    //scanner.getScan();  
    //grid_map.draw(canvas);
    world_object.draw(canvas);       
    int ret = showCanvas(canvas, DT*100);   // 1 ms waitKey

    if (ret>0)
        cerr << "Key pressed: " << ret << endl;
    switch (ret) {
        case 81:  // left;
            robot.rv += 0.1;
            //motion_iso = left;
            break;
        case 82:  // up;
            robot.tv += 0.1;
            //motion_iso = forward;
            break;
        case 83:  // right;
            robot.rv -= 0.1;
            //motion_iso = right;
            break;
        case 84:  // down;
            robot.tv -= 0.1;
            //motion_iso = backward;
            break;
        case 32:  // space;
            robot.tv = 0;
            robot.rv = 0;
        default:;
    }


    
    //robot.move(motion_iso);
    

    if (ret == 'q') {
        break;
    }

    //msg.header.stamp = ros::Time::now();
    //std::copy(scan.ranges.begin(), scan.ranges.end(), msg.ranges.begin());
    //pub_scan.publish(msg);
    //rate.sleep();
    
    }
    
    return 0;
}






/*

#include <iostream>

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include "grid_map.h"

#include "laser_scan.h"
#include "laser_scanner.h"
#include "world_item.h"

#include "isometry_2.h"

// TODO 1: refactor this code, separating headers and cpp for
//        LaserScanner
//        LaserScan

using namespace std;

void printItems(WorldItem** items) {
  while (*items) {
    WorldItem* v = *items;
    cout << "**************************************" << endl
         << "Item: " << v << endl
         << "pose: " << endl
         << v->pose_in_parent << endl
         << "parent: " << v->parent << endl;
    ++items;
  }
}

void drawItems(Canvas& canvas, WorldItem** items) {
  while (*items) {
    WorldItem* v = *items;
    v->draw(canvas, true);
    ++items;
  }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "SCANNERINO_SIMULINO");
    ros::NodeHandle n;
    ros::Publisher pub_scan =n.advertise<sensor_msgs::LaserScan>("LiDAR/LD06", 1);

    const char* filename = "/home/francesco/Documenti/LIDARINO_ROBOT/LIDARino_robot/LIDARINO_WORKSPACE/src/lidarino_pkg/src/cappero_laser_odom_diag_2020-05-06-16-26-03.png";
    const float resolution= 0.1f;

    GridMap grid_map(0, 0, 0.1);
    grid_map.loadFromImage(filename, resolution);
    Canvas canvas;
    Vec2 center = grid_map.grid2world(grid_map.origin);
    cerr << "center: " << center[0] << "  " << center[1] << endl;
    cerr << "origin: " << grid_map.origin[0] << "  " << grid_map.origin[1]
        << endl;

    grid_map.draw(canvas);

    WorldItem* items[100];
    memset(items, 0, sizeof(WorldItem*) * 100);

    World world_object(grid_map);
    items[0] = &world_object;

    UnicyclePlatform robot(world_object);
    robot.radius = 1;
    robot.tvel = 1;
    robot.rvel = -0.1;
    items[1] = &robot;


    LaserScan scan;
    LaserScanner scanner(scan, robot, Isometry2(0, -1, -0.5), 1);
    scanner.radius = 0.5;
    items[2] = &scanner;

    const Isometry2 forward(0.1, 0, 0);
    const Isometry2 backward(-0.1, 0, 0);
    const Isometry2 left(0, 0, 0.1);
    const Isometry2 right(0, 0, -0.1);

    while (true) {
        // scanner.getScan();
        grid_map.draw(canvas);
        drawItems(canvas, items);
        robot.tick(0.01);
        scanner.tick(0.01);

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
        // object_1.move(motion_iso);
    }
}

*/