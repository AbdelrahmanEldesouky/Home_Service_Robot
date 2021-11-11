#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  bool pickup = true;

  // Define target goals
  float pickup_x = 6.5;
  float pickup_y = -0.5;
  float dropoff_x = 7.0;
  float dropoff_y = 4.0;
  
  // Set shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "markers";
    marker.id = 0;

    // Set the marker type
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
  
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      //ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    if (pickup)
    {
      marker.pose.position.x = pickup_x;
      marker.pose.position.y = pickup_y;
      marker.pose.orientation.w = 1.0;
      marker.lifetime = ros::Duration(5.0);
    }
    else
    {
      marker.pose.position.x = dropoff_x;
      marker.pose.position.y = dropoff_y;
      marker.pose.orientation.w = 1.0;
      marker.lifetime = ros::Duration();
    } 
   
    marker_pub.publish(marker);    

    if (pickup) 
    {
      ros::Duration(5.0).sleep();
      pickup=false;
    }
    r.sleep();
  }
}
