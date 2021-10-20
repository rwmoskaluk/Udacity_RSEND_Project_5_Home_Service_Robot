#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int8.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std_msgs::Int8 state_value;
int current_state = 0;

int main(int argc, char** argv){
    ros::init(argc, argv, "pick_objects");

    ros::NodeHandle n;
    ros::Duration duration(5.0);

    ros::Publisher state_pub = n.advertise<std_msgs::Int8>("robot_goal_state", 1);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal pick_goal;

    // set up the frame parameters
    pick_goal.target_pose.header.frame_id = "/map";
    pick_goal.target_pose.header.stamp = ros::Time::now();

    move_base_msgs::MoveBaseGoal place_goal;

    // set up the frame parameters
    place_goal.target_pose.header.frame_id = "/map";
    place_goal.target_pose.header.stamp = ros::Time::now();

    state_pub.publish(state_value);

    while (ros::ok()) {
        switch (current_state) {
            case 0:
                //pick display state, setting pick goal
                pick_goal.target_pose.pose.position.x = 2.0;
                pick_goal.target_pose.pose.position.y = -1.0;
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
                    current_state = 1;
                    state_value.data = 1;
                    state_pub.publish(state_value);
                } else {
                    ROS_INFO("The robot failed to move to the pick_goal for some reason");
                }
                break;
            case 1:
                //hide marker after pickup
                ROS_INFO("Hiding pick marker");
                current_state = 2;
                state_value.data = 2;
                state_pub.publish(state_value);
                break;
            case 2:
                //wait 5 seconds after marker is hidden
                ROS_INFO("waiting 5 seconds");
                duration.sleep();
                current_state = 3;
                state_value.data = 3;
                state_pub.publish(state_value);
                break;
            case 3:
                //place display state
                place_goal.target_pose.pose.position.x = -4.0;
                place_goal.target_pose.pose.position.y = 1.0;
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
                    current_state = 4;
                    state_value.data = 4;
                    state_pub.publish(state_value);
                } else {
                    ROS_INFO("The base failed to move to the place_goal for some reason");
                }
                break;
            case 4:
                //mission complete
                ROS_WARN_ONCE("Completed states");
                state_pub.publish(state_value);
                break;
            default:
                break;
        }
    }
}