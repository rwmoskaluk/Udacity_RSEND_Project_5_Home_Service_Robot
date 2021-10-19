#!/bin/bash

#tests slam setup for project
if command -v xterm > /dev/null 2>&1;
then
  term_name="xterm -e"
else
  echo "xterm is not installed, using gnome-terminal instead"
  term_name="gnome-terminal -- bash -c"
fi

world_path="$HOME/catkin_ws/src/worlds/smallworld.world"

(cd $HOME/catkin_ws/ && source devel/setup.bash)

$term_name "killall gzserver && killall gazebo " &
sleep 1

$term_name " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$world_path extra_gazebo_args:="--verbose"" &

sleep 5

$term_name " roslaunch turtlebot_gazebo gmapping_demo.launch " &

sleep 5

$term_name " roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

$term_name " roslaunch turtlebot_teleop keyboard_teleop.launch"
