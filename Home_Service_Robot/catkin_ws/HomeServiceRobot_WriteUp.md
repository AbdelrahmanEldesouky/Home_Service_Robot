# Home Service Robot

My final project in UDACITY Robotics Software Engineer Nanodegree, I have use the following topic to simulate my robot successfully

1.  Gazebo Simulator to create my environment and simulate it as in real environment.
2. Using the gmapping SLAM to create my environment map to use it for localization and navigation and manually navigate it, then export map files (map.pgm, map.ymal) by using map_server and attach them in project file to use them in navigation.
3. Using navigation to send the robot in dummy position by using 2D Nav Goal in rviz and use the exported map from SLAM package to navigate the environment properly.
4. Write pick_objects node that automatically move the robot from initial position to the pick up position, then drive it to the drop off goal using navigation package and map exported by SLAM package.
5.    Write add_markers node that add markers in rviz, then follow my robot odometry, track my robot position, disappear when robot reach it and publish it to the rviz.

We just put robot in dummy environment, create map for this environment, localize and navigate throw this environment, create markers, then picked up and drop it off in another place.