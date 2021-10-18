#!/bin/bash

if command -v xterm > /dev/null 2>&1;
then
  term_name="xterm -e"
else
  echo "xterm is not installed, using gnome-terminal instead"
  term_name="gnome-terminal -- bash -c"
fi

map_path="$HOME/catkin_ws/src/map"

$term_name "killall gzserver && killall gazebo " &
sleep 1

$term_name " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$map_path/myworld.world extra_gazebo_args:="--verbose"" &

sleep 5

$term_name " roslaunch turtlebot_gazebo amcl_demo.launch " &

sleep 5

$term_name " rosrun pick_objects pick_objects_node " &

sleep 5

$term_name " roslaunch turtlebot_rviz_launchers view_navigation.launch"

