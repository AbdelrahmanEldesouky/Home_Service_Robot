# Home Service Robot
Project 5 of Udacity Robotics Software Engineer Nanodegree

## Summary 

In this project I use every thing I had learn in this Nanodegree. 

### Mapping  
I was created a test_slam.sh script file and launch it to manually test SLAM. A functional map of the environment will be created which would be used for localization and navigation tasks.
### Localization and Navigation  
I was created a `test_navigation.sh` script file to launch it for manual navigation test. my robot will be able to navigate in the environment after a 2D Nav Goal command is issued, also I was created a `pick_objects.sh` file that will send multiple goals for the robot to reach. 

> 1. The robot travels to the desired pickup zone.
> 2. Displays a message that it reached its destination.
> 3. Waits 5 seconds.
> 4. Travels to the desired drop off zone.
> 5. Displays a message that it reached the drop off zone.

### Virtual Objects

 I was created a `add_marker.sh` file that will publish a marker to rviz.

> 1. The marker will initially be published at the pickup zone.
> 2. After 5 seconds it will be hidden. 
> 3. Then after another 5 seconds it will appear at the drop off zone.

### Home Service Functions  

1. Initially show the marker at the pickup zone. 
2. Hide the marker once your robot reaches the pickup zone. 
3. Wait 5 seconds to simulate a pickup. 
4. Show the marker at the drop off zone once your robot reaches it.


## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS navigation package  
```
sudo apt-get install ros-kinetic-navigation
```
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```

## Setup Instructions (abbreviated)  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line and execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. On the command line and execute  
```
cd RoboND-Term1-P5-Home-Service-Robot/catkin_ws/src  
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git  
```
5. Build and run the code.  

## Project Description  
Directory Structure  
```
.Home-Sevice-Robot                                        # Home Service Robot Project
├── catkin_ws                                             # Catkin workspace
│   ├── src
│   │   ├── add_markers                                   # add_markers package        
│   │   │   ├── launch
│   │   │   │   ├── view_home_service_navigation.launch   # launch file for home service robot demo
│   │   │   ├── src
│   │   │   │   ├── add_markers.cpp                       # source code for add_markers node
│   │   │   │   ├── add_markers_demo.cpp                  # source code for add_markers_demo
│   │   ├── pick_objects                                  # pick_objects package     
│   │   │   ├── src
│   │   │   │   ├── pick_objects.cpp                      # source code for pick_objects node
│   │   │   │   ├── pick_objects_demo.cpp                 # source code for pick_objects_demo
│   │   ├── my_robot	                                  # my_robot package 
│   │   │   ├── worlds              					# House.world file
│   │   ├── map	                                  		 # map files 
│   │   ├── rvizConfig                                    # rvizConfig package        
│   │   │   ├── home_service_rvizConfig.rviz              # rvizConfig file for home service robot demo  
│   │   ├── scripts                                       # shell scripts files
│   │   │   ├── add_marker.sh                             # shell script to model virtual objects  
│   │   │   ├── home_service.sh                           # shell script to launch home service robot demo  
│   │   │   ├── pick_objects.sh                           # shell script to send multiple goals  
│   │   │   ├── test_navigation.sh                        # shell script to test localization and navigation
│   │   │   ├── test_slam.sh                              # shell script to test SLAM
│   │   ├── slam_gmapping                                 # gmapping_demo.launch file
│   │   ├── turtlebot                                     # keyboard_teleop.launch file
│   │   ├── turtlebot_interactions                        # view_navigation.launch file
│   │   ├── turtlebot_simulator                           # turtlebot_world.launch file package        
│   │   ├── CMakeLists.txt                                # compiler instructions
```
- [view_home_service_navigation.launch](/catkin_ws/src/add_markers/launch/view_home_service_navigation.launch): Launch rviz with specify rviz configuration file  
- [add_markers.cpp](/catkin_ws/src/pick_objects/src/add_markers.cpp): C++ script, communicate with `pick_objects` node and control the marker appearance to simulate object pick up and drop off   
- [pick_objects.cpp](/catkin_ws/src/pick_objects/src/pick_objects.cpp): C++ script, communicate with `add_markers` node and command the robot to pick up the object  
- [home_service_rvizConfig.rviz](/catkin_ws/src/rvizConfig/home_service_rvizConfig.rviz): rvizConfig file for home service robot demo which contained `markers` option  
- [add_marker.sh](/catkin_ws/src/scripts/add_marker.sh): Shell script file to deploy a turtlebot inside your environment, model a virtual object with markers in `rviz`.  
- [home_service.sh](/catkin_ws/src/scripts/home_service.sh): Shell script file to deploy a turtlebot inside your environment, simulate a full home service robot capable of navigating to pick up and deliver virtual objects.  
- [pick_objects.sh](/catkin_ws/src/scripts/pick_objects.sh): Shell script file to deploy a turtlebot inside your environment, communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach.  
- [test_navigation.sh](/catkin_ws/src/scripts/test_navigation.sh): Shell script file to deploy a turtlebot inside your environment, pick two different goals and test your robot's ability to reach them and orient itself with respect to them.  
- [test_slam.sh](/catkin_ws/src/scripts/test_slam.sh): Shell script file to deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in `rviz`  
- [CMakeLists.txt](/catkin_ws/src/CMakeLists.txt): File to link the C++ code to libraries.  

## Run the project  
* Clone this repository

```
```

* Navigate to the `src` folder and clone the necessary repositories  

```
cd RoboND-Term1-P5-Home-Service-Robot/catkin_ws/src  
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git  
```
* Open the repository, make and source  
```
cd /home/workspace/catkin_ws/
catkin_make
source devel/setup.bash
```
* Launch the home service robot
```
cd /src/scripts
chmod +x ./home_service.sh
./home_service.sh
```
* Done. 
