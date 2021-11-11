#!/bin/sh

# launches these files
# turtlebot_world.launch
# gmapping_demo.launch 
# view_navigation.launch
# keyboard_teleop.launch

# going back to folder /catkin_ws
cd ../.. 

# source and launch turtlebot_world.launch
xterm -e "source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/my_robot/worlds/House.world" &

sleep 10

# source and launch gmapping_demo.launch
xterm -e "source devel/setup.bash && roslaunch turtlebot_gazebo gmapping_demo.launch" &

sleep 5

# source and launch view_navigation.launch
xterm -e "source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

# source and launch keyboard_teleop.launch
xterm -e "source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch"
