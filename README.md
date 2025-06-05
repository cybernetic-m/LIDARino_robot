# LIDARino v0.1
This is the official repository of the Robot Programming subject in Artificial Intelligence and Robotics course at Sapienza University of Rome



<img src="./images/image_1.png" alt="Description" width="600" height = "400" />


<img src="./gifs/RP3v3.gif" alt="Description"  width="600" height = "1000" />


## STORY

da v0.1 a v0.2




<img src="./images/lidarinov01.jpeg" alt="Description"  />
<img src="./gifs/rplidarino1.gif" alt="Description"   />
<img src="./gifs/rplidarino2.gif" alt="Description"  />
<img src="./gifs/rplidarino3.gif" alt="Description"  />



## INSTALLATION 

1. Clone the repository:  
 ```sh 
 git clone "https://github.com/cybernetic-m/LIDARino_robot"
 ```
2. Install rosserial on the arduino IDE and on on ROS

3. open the firmware folder with arduino IDE and upload it to your arduino

4. Install the lidar ros package from the repo and follow the instructions in the doc folder of the repo
 ```sh 
 git clone "https://github.com/LetsOKdo/sdk_ld06_raspberry_ros"
 ```

5. (optional) follow point 7 of this tutorial https://yoraish.wordpress.com/2021/09/08/a-full-autonomous-stack-a-tutorial-ros-raspberry-pi-arduino-slam/ to install hector mapper to create a custom map

## LAUNCH

1. start the lidar 
 ```sh 
 cd sdk_ld06_raspberry_ros
 roslaunch ldlidar ld06.launch 
 ```

2. start our package in another terminal

 ```sh 
 cd LIDARino_robot/LIDARINO_WORKSPACE
 . ./devel/setup.bash
 roslaunch lidarino_pkg pi.launch
 ```

3. start rosserial on the raspberry side

 ```sh 
 rosrun rosserial_python serial_node.py /dev/ttyACM0
 ```

4. start the keyboard interface 

 ```sh 
 cd LIDARino_robot/LIDARINO_WORKSPACE
 . ./devel/setup.bash
 rosrun lidarino_pkg keyboard_interface
 ```


 ## Authors
Massimo Romano (2043836) (https://github.com/cybernetic-m) 

Paolo Renzi (1887793) (https://github.com/RenziPaolo)

Francesco Giarrusso (1807094) (https://github.com/fragiarrusso)
