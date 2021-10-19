#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Bool.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool pick_goal = false;
bool place_goal = false;

int main(int argc, char** argv){
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");

    ros::NodeHandle n;

    ros::Publisher pick_marker_goal = n.advertise<std_msgs::bool>("pick_goal", 10);
    ros::Publisher place_marker_goal = n.advertise<std_msgs::bool>("place_goal", 10);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    pick_marker_goal.publish(pick_goal);
    place_marker_goal.publish(place_goal);


    if (!pick_goal && !place_goal) {
        move_base_msgs::MoveBaseGoal pick_goal;

        // set up the frame parameters
        pick_goal.target_pose.header.frame_id = "/map";
        pick_goal.target_pose.header.stamp = ros::Time::now();

        // Define a position and orientation for the robot to reach
        pick_goal.target_pose.pose.position.x = 5.0;
        pick_goal.target_pose.pose.position.y = 5.0;
        pick_goal.target_pose.pose.position.z = 0.1;
        pick_goal.target_pose.pose.orientation.w = 1.0;


        // Send the goal position and orientation for the robot to reach
        ROS_INFO("Sending pick goal");
        ac.sendGoal(pick_goal);

        // Wait an infinite time for the results
        ac.waitForResult();

        // Check if the robot reached the pick_goal
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Robot arrived at pick_goal");
            pick_goal = true;
            // Wait 5 sec for move_base action server to come up
            while (!ac.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for 5 seconds at pick_goal");
                pick_marker_goal.publish(pick_goal);
            }
        } else {
            ROS_INFO("The robot failed to move to the pick_goal for some reason");
        }


    }
    if (pick_goal && !place_goal) {

        move_base_msgs::MoveBaseGoal place_goal;

        // set up the frame parameters
        place_goal.target_pose.header.frame_id = "/map";
        place_goal.target_pose.header.stamp = ros::Time::now();

        // Define a position and orientation for the robot to reach
        place_goal.target_pose.pose.position.x = -3.0;
        place_goal.target_pose.pose.position.y = -2.0;
        place_goal.target_pose.pose.position.z = 0.1;
        place_goal.target_pose.pose.orientation.w = 1.0;

        // Send the goal position and orientation for the robot to reach
        ROS_INFO("Sending place goal");
        ac.sendGoal(place_goal);

        // Wait an infinite time for the results
        ac.waitForResult();

        // Check if the robot reached the place_goal
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Robot arrived at the place_goal");
            place_goal = true;
            while (!ac.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for 5 seconds at pick_goal");
                place_marker_goal.publish(place_goal);
            }
        }
        else {
            ROS_INFO("The base failed to move to the place_goal for some reason");
        }
    }

    return 0;
}