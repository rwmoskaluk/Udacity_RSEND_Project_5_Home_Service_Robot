#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Int8.h"

int current_state = 0;

void robot_stateCallback(const std_msgs::Int8::ConstPtr &robot_state) {
    //std_msgs::Int8 state_value;

    //state_value = robot_state->data;
    current_state = robot_state->data;
    if (current_state < 4)
        ROS_INFO("The current state goal is [%d]", robot_state->data);
    else
        ROS_WARN_ONCE("completed mission");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber state_sub = n.subscribe("/robot_goal_state", 100, robot_stateCallback);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETEALL; //clear old markers for rerunning
    marker_pub.publish(marker);


    while (ros::ok()) {

        switch (current_state) {
            case 0:
                //pick up zone marker state
                // Set the namespace and id for this marker.  This serves to create a unique ID
                // Any marker sent with the same namespace and id will overwrite the old one
                ROS_WARN_ONCE("current state = %i", current_state);
                ROS_WARN_ONCE("Pick marker being displayed");

                marker.ns = "pick_marker";
                marker.id = 0;

                // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                marker.type = shape;

                // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                marker.action = visualization_msgs::Marker::ADD;

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                marker.pose.position.x = 2.0;
                marker.pose.position.y = -1.0;
                marker.pose.position.z = 0.1;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;

                // Set the color -- be sure to set alpha to something non-zero!

                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;

                marker.lifetime = ros::Duration();
                break;

            case 1:
                // hide marker state
                ROS_WARN_ONCE("current state = %i", current_state);
                ROS_WARN_ONCE("Pick marker being hidden");
                //marker.action = visualization_msgs::Marker::DELETE; //doesn't seem to work as intended
                marker.ns = "pick_marker";
                marker.id = 0;
                marker.action = visualization_msgs::Marker::ADD;
                marker.color.a = 0.0;

                marker_pub.publish(marker);
                break;
            case 2:
                // wait 5 second state
                ROS_WARN_ONCE("current state = %i", current_state);
                ROS_WARN_ONCE("Simulating pickup for 5 seconds");
                //marker.action = visualization_msgs::Marker::DELETE; //doesn't seem to work as intended
                marker.ns = "pick_marker";
                marker.id = 0;
                marker.action = visualization_msgs::Marker::ADD;
                marker.color.a = 0.0;


                marker_pub.publish(marker);
                break;

            case 3:
                //robot navigating to place zone
                //make sure marker is deleted in this state too
                ROS_INFO("current state = %i", current_state);
                ROS_INFO("robot is navigating to place zone");
                break;
            case 4:
                // place marker state
                // Set the namespace and id for this marker.  This serves to create a unique ID
                // Any marker sent with the same namespace and id will overwrite the old one
                ROS_WARN_ONCE("current state = %i", current_state);
                ROS_WARN_ONCE("Place marker being displayed");
                marker.ns = "place_marker";
                marker.id = 1;

                // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                marker.type = shape;

                // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                marker.action = visualization_msgs::Marker::ADD;

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                marker.pose.position.x = -4.0;
                marker.pose.position.y = 1.0;
                marker.pose.position.z = 0.1;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;

                // Set the color -- be sure to set alpha to something non-zero!

                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;

                marker.lifetime = ros::Duration();
                break;


            default:
                ROS_INFO("In bad state...");
                break;
        }

        marker_pub.publish(marker);
        ros::spinOnce();
    }
}