#!/bin/bash

if command -v xterm > /dev/null 2>&1;
then
  term_name="xterm -e"
else
  echo "xterm is not installed, using gnome-terminal instead"
  term_name="gnome-terminal -- bash -c"
fi

world_path="$HOME/catkin_ws/src/worlds/smallworld.world"
map_path="$HOME/catkin_ws/src/map/map.yaml"
rviz_path="$HOME/catkin_ws/src/rvizConfig/rviz.rviz"


(cd $HOME/catkin_ws/ && source devel/setup.bash)


$term_name "killall gzserver && killall gazebo " &
sleep 1

$term_name " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$world_path extra_gazebo_args:="--verbose"" &

sleep 5

$term_name " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$map_path " &

sleep 5

$term_name " rosrun pick_objects pick_objects_node " &

sleep 1

$term_name " rosrun add_markers add_markers_node " &

sleep 1

$term_name " rosrun rviz rviz -d $rviz_path "