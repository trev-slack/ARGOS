# ARGOS Gazebo Simulator
This package is used to simulate the sensor stack on board the ARGOS rover. The Clearpath Jackal is a stand in for the ARGOS rover. 
## Sensors
The simulator uses a 360 2D plannar LiDAR, an FPV camera, IMU, wheel odometry, and GPS. All sensor data is integrated using an Extended Kalman Filter provided by the robot_localization package. 
* LiDAR: Velodyne 360 Puck
* FPV Camera: FLIR Flea3
* IMU: Gazebo default IMU
* GPS: Gazebo default GPS
* Odometer: Clearpath default Odometer
## Install
To install the simulator, ROS Melodic and Gazebo 9 are required. Clone this repository into your catkin workspace along with the required dependencies.

Dependencies:
1. multi_jackal: https://github.com/NicksSimulationsROS/multi_jackal
2. pointgrey_camera_driver: https://github.com/ros-drivers/pointgrey_camera_driver
Once all of the repos are in your workspace, build it with catkin_make. 
## Usage
First move the "lidar_flea" file from the "argos_gazebo/resources" folder to the "multi_jackal/multi_jackal_description/urdf/configs" folder. This file tells the launcher what sensors to add. 
After sourcing your workspace, a test simulation can be run with "roslaunch argos_gazebo argos_test.launch"
Publish Waypoints to "/argos/move_base/goal" in the form of MoveBaseActionGoal messages. The RViz display can be modified to show the costmap and planners. 
## Navigation
This simulator uses the ROS navigation stack as a motion planner. This planner publishes geometry/Twist messages to the Jackal. The tolerances of the planner can be adjusted in the launch file. 
