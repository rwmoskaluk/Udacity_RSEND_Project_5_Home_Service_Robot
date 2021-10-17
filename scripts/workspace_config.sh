#!/bin/bash

#mkdir -p ~/catkin_ws/src
#cd ~/catkin_ws/src
#catkin_init_workspace
#cd ..
#catkin_make
#sudo apt-get update
#cd ~/catkin_ws/src
#git clone https://github.com/ros-perception/slam_gmapping
#git clone https://github.com/turtlebot/turtlebot
#git clone https://github.com/turtlebot/turtlebot_interactions
#git clone https://github.com/turtlebot/turtlebot_simulator
#cd ~/catkin_ws/
#source devel/setup.bash
#rosdep -i install gmapping
#rosdep -i install turtlebot_teleop
#rosdep -i install turtlebot_rviz_launchers
#rosdep -i install turtlebot_gazebo
#catkin_make
#source devel/setup.bash

if command -v xterm > /dev/null 2>&1;
then
  term_name="xterm -e"
else
  echo "xterm is not installed, using gnome-terminal instead"
  term_name="gnome-terminal -- bash -c"
fi

$term_name " mkdir -p ~/catkin_ws/src &&
cd ~/catkin_ws/ &&
catkin_make &&
sudo apt-get update &&
cd ~/catkin_ws/src &&
git clone https://github.com/ros-perception/slam_gmapping &&
git clone https://github.com/turtlebot/turtlebot &&
git clone https://github.com/turtlebot/turtlebot_interactions &&
git clone https://github.com/turtlebot/turtlebot_simulator &&
cd ~/catkin_ws/ &&
source devel/setup.bash &&
rosdep -i install gmapping &&
rosdep -i install turtlebot_teleop &&
rosdep -i install turtlebot_rviz_launchers &&
rosdep -i install turtlebot_gazebo &&
catkin_make &&
source devel/setup.bash "
echo "completed workspace configuration"