#!/bin/bash
run_script=false
if command -v xterm > /dev/null 2>&1;
then
  term_name="xterm -e"
else
  echo "xterm is not installed, using gnome-terminal instead"
  term_name="gnome-terminal -- bash -c"
fi

if command -v rosversion -d > /dev/null 2>&1;
then
  ros=$(rosversion -d)
  run_script=true
else
  echo "ros not found!"
fi
if $run_script
then
  $term_name "killall gzserver"
  $term_name  " gazebo " &
  echo "launching gazebo"
  sleep 5
  ros_dir="/opt/ros/$ros/setup.bash"
  source_ros=$(source $ros_dir)
  $term_name "killall roscore"
  $term_name  $source_ros
  $term_name  roscore &
  echo "launching ros"
  sleep 5
  $term_name  " rosrun rviz rviz"
  echo "launching rviz"
else
  echo "missing dependencies"
fi