#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Bool.h"

bool pick_goal = false;
bool place_goal = false;

void pickgoalCallback(const std_msgs::Bool::ConstPtr& pick_msg)
{
    pick_goal = pick_msg->data;
    ROS_INFO("pick goal state [%i]", pick_goal);
}

void placegoalCallback(const std_msgs::Bool::ConstPtr& place_msg)
{
    place_goal = place_msg->data;
    ROS_INFO("place goal state [%i]", place_goal);

}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber pick_sub = n.subscribe("/pick_marker_goal", 10, pickgoalCallback);
  ros::Subscriber place_sub = n.subscribe("/place_marker_goal", 10, placegoalCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  bool display_pick = true;
  bool display_place = false;
  bool pickup_reached = false;
  bool place_reached = false;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  while (ros::ok())
  {
      if (display_pick && !pick_goal) {
          // Set the namespace and id for this marker.  This serves to create a unique ID
          // Any marker sent with the same namespace and id will overwrite the old one
          marker.ns = "pick_marker";
          marker.id = 0;

          // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
          marker.type = shape;

          // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
          marker.action = visualization_msgs::Marker::ADD;

          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
          marker.pose.position.x = 5;
          marker.pose.position.y = 5;
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
      }

      if (display_place && !place_goal) {
          // Set the namespace and id for this marker.  This serves to create a unique ID
          // Any marker sent with the same namespace and id will overwrite the old one
          marker.ns = "place_marker";
          marker.id = 1;

          // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
          marker.type = shape;

          // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
          marker.action = visualization_msgs::Marker::ADD;

          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
          marker.pose.position.x = -3;
          marker.pose.position.y = -2;
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

      }

      marker_pub.publish(marker);

    // Publish the marker
    /*
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    */






    // Cycle between different shapes
    /*
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }
    */

    r.sleep();
  }
}