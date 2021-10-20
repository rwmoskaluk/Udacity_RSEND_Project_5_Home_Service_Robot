#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Int8.h"

int current_state = 0;

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers_tester");
    ros::NodeHandle n;
    ros::Duration duration(5.0);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    ROS_WARN_ONCE("current state = %i", current_state);
    ROS_WARN_ONCE("Pick marker being displayed");

    marker.ns = "pick_marker";
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 2.0;
    marker.pose.position.y = -1.0;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
    duration.sleep();
    current_state = 1;

    while (ros::ok()) {
        marker_pub.publish(marker);

        switch (current_state) {

            case 1:
                // pausing for 5 seconds
                ROS_WARN_ONCE("current state = %i", current_state);
                ROS_WARN_ONCE("Pausing 5 seconds");
                marker.action = visualization_msgs::Marker::ADD;
                marker_pub.publish(marker);
                duration.sleep();
                current_state = 2;
                break;
            case 2:
                // hiding the marker
                ROS_WARN_ONCE("current state = %i", current_state);
                ROS_WARN_ONCE("Hiding the pick marker");
                marker.action = visualization_msgs::Marker::DELETE;
                marker_pub.publish(marker);
                current_state = 3;
                break;

            case 3:
                //Pausing for 5 seconds
                //make sure marker is deleted in this state too
                ROS_WARN_ONCE("current state = %i", current_state);
                ROS_WARN_ONCE("Pausing for 5 seconds");
                marker.action = visualization_msgs::Marker::DELETE;
                marker_pub.publish(marker);
                duration.sleep();
                current_state = 4;
                break;
            case 4:
                // place marker state
                ROS_WARN_ONCE("current state = %i", current_state);
                ROS_WARN_ONCE("Place marker being displayed");
                marker.ns = "place_marker";
                marker.id = 1;

                marker.type = shape;

                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position.x = -4.0;
                marker.pose.position.y = 1.0;
                marker.pose.position.z = 0.1;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;

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
    }
}

