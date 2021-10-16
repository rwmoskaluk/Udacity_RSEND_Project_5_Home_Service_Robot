#!/bin/sh
if command -v xterm > /dev/null 2>&1;
then
  term_name="xterm"
else
  echo "xterm is not installed"
  term_name="gnome-terminal"
fi

if command -v rosversion -d > /dev/null 2>&1;
then
  ros=$(rosversion -d)
else
  echo "ros not found!"
fi
$term_name -- bash -c "killall gzserver"
sleep 1
$term_name  -- bash -c  " gazebo " &
sleep 5
$term_name  -- bash -c  " source /opt/ros/melodic/setup.bash; roscore" &
sleep 5
$term_name  -- bash -c  " rosrun rviz rviz"