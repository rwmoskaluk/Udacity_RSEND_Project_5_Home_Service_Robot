# Udacity_RSEND_Project_5_Home_Service_Robot

[image1]: ./pictures/3dmap_perspective.png "Gazebo 3d Map Perspective view"
[image2]: ./pictures/3dmap_top.png "Gazebo 3d Map Top view"
[image3]: ./pictures/gslam_map_result.png "Gslam map result"
[image4]: ./pictures/Dropoff_location.png "Drop off"
[image5]: ./pictures/Example_running_dropoff.png "Pickup"
[image6]: ./pictures/Example_running_dropoff.png "AMCL example"



### Project Overview
This project had the task of using an off the shelf robot, Turtlebot, with various sensors to navigate an environment to reach different goals.  The goals represented a pick up goal and a drop off goal in the environment.  For this project these are modelled as cubes (red=pickup, green=dropoff)

# How to Run
This project requires usage of a ros version less than Melodic, the original turtlebot and supporting packages are not supported.  Turtlebot3 is though.

Clone this repo to the catkin workspace, and move the folders to the src directory
```
cd ~/catkin_ws/src
git clone https://github.com/rwmoskaluk/Udacity_RSEND_Project_5_Home_Service_Robot.git
mv Udacity_RSEND_Project_5_Home_Service_Robot/* .
```

Run catkin_make to build everything
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

For Udacity Workspace (This affects Gazebo from launching)
```
pip install rospkg
```

Navigate to the scripts directory
```
cd ~/catkin_ws
source devel/setup.bash
cd ~/catkin_ws/src/scripts
```

Make all the scripts executable in the scripts directory
```
sudo chmod +x *.sh
```

Launch the home_service script with debug enabled

```
bash -x ./home_service.sh
```

## Environment and Conops
The map used for this project consists of various rooms with some unique objects (bookshelf, stop sign, fire hydrant) along with unique colors for each room.

![alt text][image1]
![alt text][image2]

The objective of the robot is as follows:
1. Navigate to the pickup location where a red cube is waiting
2. Pickup the red cube
3. Wait 5 seconds
4. Navigate to the drop off location
5. Place a green cube at the location

The pickup location is shown below next to the bookshelf by the orange room.
![alt text][image6]


The dropoff location is shown below next to the red room corridor.
![alt text][image4]




## Example Running 
![alt text][image1]

Example running where delays are disabled
<img src="pictures/Example_running_no_delays.gif?raw=true" width="720px">

Example running with delay at pickup
<img src="pictures/Example_running_delays.gif?raw=true" width="720px">



## Writeup

Throughout the Udacity course various fundamentals in robotics were taught.  In this project these fundamentals were all utilized together to create a robot that can navigate throughout it's environment to accomplish simple tasks.

The packages that were utilized included the following: AMCL, GSLAM, Turtlebot, and Rviz

The ROS package `gmapping` was used for mapping the environment. The results can of this map can be seen below.  Care was taken to have many unique markers in the environment along with navigating the environment slow enough that an accurate map could be made.  `gmapping` utilizes a particle filter with some extra filtering in order to generate a map and keep track of the robot's location.  This specific package works well with long range lidar scanners as indicated on the package page.  Default values for this package proved to work well for the Turtlebot and the environment being mapped.

`gmapping` package information
```
https://openslam-org.github.io/gmapping.html
```

![alt text][image3]

The next package that was utilized was the `AMCL` package.  This package was used to take the map generated above and localize the robot within the environment.  The default configuration for the `AMCL` package is to utilize laser scans to determine localization in the environment.  For this project the default parameters were utilized in the `AMCL` package for the Turtlebot.  Further tuning could be done to improve speed and accuracy.  This could have been done for the larger maps I initially started with, but as noted below in the `Notes and Learning` section `AMCL` has memory limitations for map size being loaded in at a given time.


`AMCL` package information
```
http://wiki.ros.org/amcl
```

It can be seen that the map generated from `gmapping` is padded for path planning.  This is managed by the `AMCL` package when planning paths throughout an environment.  
![alt text][image6]


For this project a state machine was utilized for the previously mentioned goals that the robot had to accomplish.  A publisher for the current state based on the feedback from the `actionClientLib` when commanding positions in the environment was utilized. This was then coupled with a subscriber for generating the environment markers in `Rviz` given the current state of the robot.


# Notes and Learnings

To save time creating a Udacity workspace script `workspace_config.sh` was generated to configure the Udacity workspace each time.

ROS Melodic does not support officially the Turtlebot setup and packages specific to this project.  It can be run with a specific install of the following packages (not recommended as a main choice for running)
```
https://github.com/gaunthan/Turtlebot2-On-Melodic/
```
This may further require fiddling with your ROS install.

Make sure your map is small, problems can arise with too large of map to try and load into AMCL node.  If you wish to use a larger map consider loading sections of it at a time.  

With any map be sure to have various unique features (models, colors, room shapes) so that localization and mapping can easily be accomplished.
