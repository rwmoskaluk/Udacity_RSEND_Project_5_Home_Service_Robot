#!/bin/bash

#tests slam setup for project
if command -v xterm > /dev/null 2>&1;
then
  term_name="xterm"
else
  echo "xterm is not installed, using gnome-terminal instead"
  term_name="gnome-terminal"
fi

cd ../map/
map_path="$(pwd)"

$term_name -- bash -c "killall gzserver && killall gazebo &"
sleep 1
cd ../../
$term_name -- bash -c " source devel/setup.bash & roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$map_path/myworld.world extra_gazebo_args:="--verbose""

sleep 5

$term_name -- bash -c " source devel/setup.bash & roslaunch turtlebot_gazebo gmapping_demo.launch &"

sleep 30
