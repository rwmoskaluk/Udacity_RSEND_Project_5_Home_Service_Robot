#!/bin/bash

#tests slam setup for project
if command -v xterm > /dev/null 2>&1;
then
  term_name="xterm -e"
else
  echo "xterm is not installed, using gnome-terminal instead"
  term_name="gnome-terminal -- bash -c"
fi

map_path="$HOME/catkin_ws/src/map"
rviz_path="$HOME/catkin_ws/src/rvizConfig"

$term_name "killall gzserver && killall gazebo " &
sleep 1

$term_name " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$map_path/myworld.world extra_gazebo_args:="--verbose"" &

sleep 5

$term_name " roslaunch turtlebot_gazebo gmapping_demo.launch " &

sleep 5

$term_name " roslaunch turtlebot_interactions turtlebot_rviz_launches rviz_config_file=$rviz_path/rviz.rviz " &

sleep 5

$term_name " roslaunch turtlebot_teleop keyboard_teleop.launch"
