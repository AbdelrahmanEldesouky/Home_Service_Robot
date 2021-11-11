#!/bin/sh

# launches these files
# turtlebot_world.launch
# amcl_demo.launch
# view_home_service_navigation.launch
# add_markers node

# going back to folder /catkin_ws
cd ../.. 

# source and launch turtlebot_world.launch
xterm -e "source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/my_robot/worlds/House.world" &

sleep 10

# source and launch amcl_demo.launch
xterm -e "source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch  map_file:=$(pwd)/src/map/map.yaml" &

sleep 5

# source and launch view_home_service_navigation.launch
xterm -e "source devel/setup.bash && roslaunch add_markers view_home_service_navigation.launch rviz_path:=$(pwd)/src/rvizConfig/home_service_rvizConfig.rviz" &

sleep 5

# source and launch add_markers add_markers
xterm -e "source devel/setup.bash && rosrun add_markers add_markers" &

# source and launch pick_objects pick_objects
xterm -e "source devel/setup.bash && rosrun pick_objects pick_objects" 
