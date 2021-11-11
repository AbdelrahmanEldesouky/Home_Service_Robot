#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// Define target goals
float pickup_x = 6.5;
float pickup_y = -0.5;
float dropoff_x = 7.0;
float dropoff_y = 4.0;

bool pick_up = false;
bool drop_off = false;

float tolerance = 0.25;

// Define odometry function
void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg) 
{
	float robot_pose_x = msg->pose.pose.position.x;
	float robot_pose_y = msg->pose.pose.position.y;
	
	float pickup_dis;
	float dropoff_dis;
	
	if(!pick_up && !drop_off)
	{
		pickup_dis = sqrt(pow((pickup_x - robot_pose_x), 2) + pow((pickup_y - robot_pose_y), 2));
		ROS_INFO("Distance to pick up zone = %f", pickup_dis);
		
		if(pickup_dis <= tolerance)
		{
			ROS_INFO("Arrived at the pick up zone");
			pick_up = true;
			drop_off = false;
			ROS_INFO("Object was pickerd up");
			ros::Duration(5.0).sleep();
		}
	}

	if(pick_up && !drop_off)
	{
		dropoff_dis = sqrt(pow((dropoff_x - robot_pose_x), 2) + pow((dropoff_y - robot_pose_y), 2));
		ROS_INFO("Distance to drop off zone = %f", dropoff_dis);
		
		if(dropoff_dis <= tolerance)
		{
			ROS_INFO("Arrived at the drop off zone");
			drop_off = true;
			pick_up = false;
			ROS_INFO("Object was dropped off");
		}
	}
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");
  	ros::NodeHandle n;
  	ros::Rate r(1);
  	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  	// odometry values
  	ros::Subscriber odometry_sub = n.subscribe("/odom", 1000, odometry_cb);

  	// Set shape type to cube
  	uint32_t shape = visualization_msgs::Marker::CUBE;

  	// Initialize marker
  	visualization_msgs::Marker marker;
  	marker.header.frame_id = "/map";
  	marker.header.stamp = ros::Time::now();
  	marker.ns = "basic_shapes";
  	marker.id = 0;
  	marker.type = shape;

  	// Add the marker
  	marker.action = visualization_msgs::Marker::ADD;

  	// Set the scale of the marker
  	marker.scale.x = 0.4;
  	marker.scale.y = 0.4;
  	marker.scale.z = 0.4;
	
  	// Set the color
  	marker.color.r = 0.0f;
  	marker.color.g = 1.0f;
  	marker.color.b = 0.0f;
  	marker.color.a = 1.0;
	
  	// Set marker pose
	marker.pose.position.x = pickup_x;
  	marker.pose.position.y = pickup_y;
  	marker.pose.position.z = 0;
  	marker.pose.orientation.x = 0.0;
  	marker.pose.orientation.y = 0.0;
  	marker.pose.orientation.z = 0.0;
  	marker.pose.orientation.w = 1.0;
	
	marker.lifetime = ros::Duration();

  	while (ros::ok())
  	{
		// Publish the marker
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		
		if(pick_up)
		{
			marker.action = visualization_msgs::Marker::DELETE;
		}
		
		if(drop_off)
		{
			marker.pose.position.x = dropoff_x;
			marker.pose.position.y = dropoff_y;
			marker.action = visualization_msgs::Marker::ADD;
			ros::Duration(1.0).sleep();
		}
		
		marker_pub.publish(marker);
		
		ros::spinOnce();
	}
	
	return 0;
}
