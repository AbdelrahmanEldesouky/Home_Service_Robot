#!/bin/sh

# launches these files
# turtlebot_world.launch
# amcl_demo.launch
# view_navigation.launch

# going back to folder /catkin_ws
cd ../.. 

# source and launch turtlebot_world.launch
xterm -e "source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/my_robot/worlds/House.world" &

sleep 10

# source and launch amcl_demo.launch
xterm -e "source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch  map_file:=$(pwd)/src/map/map.yaml" &

sleep 5

# source and launch view_navigation.launch
xterm -e "source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" 
