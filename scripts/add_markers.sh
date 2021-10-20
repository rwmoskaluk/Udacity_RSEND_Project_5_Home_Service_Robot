#!/bin/bash

if command -v xterm > /dev/null 2>&1;
then
  term_name="xterm -e"
else
  echo "xterm is not installed, using gnome-terminal instead"
  term_name="gnome-terminal -- bash -c"
fi

known_path=$(rospack find pick_objects)
root_path=$(cd $known_path && cd .. && pwd)
world_path="$root_path/worlds/smallworld.world"
map_path="$root_path/map/map.yaml"
rviz_path="$root_path/rvizConfig/rviz.rviz"


(cd $root_path && cd .. && source devel/setup.bash)


$term_name "killall gzserver && killall gazebo " &
sleep 1

$term_name " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$world_path extra_gazebo_args:="--verbose"" &

sleep 5

$term_name " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$map_path " &

sleep 5

$term_name " rosrun add_markers add_markers_node_tester " &

sleep 5

$term_name " rosrun rviz rviz -d $rviz_path "