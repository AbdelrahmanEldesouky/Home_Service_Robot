#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 6.5;
  goal.target_pose.pose.position.y = -0.5;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Hooray, the base moved 1 meter forward");

		ros::Duration(5).sleep();
		
    		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		// Define another position and orientation for the robot to reach
		
		goal.target_pose.pose.position.x = 7.0;
		goal.target_pose.pose.position.y = 4.0;
		goal.target_pose.pose.orientation.w = 1.0;
		
		// Send the goal position and orientation for the robot to reach
		ROS_INFO("Sending second goal");
		ac.sendGoal(goal);
		
		// Wait an infinite time for the results
		ac.waitForResult();
		
		// Check if the robot reached its goal
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Hooray, the base moved to the second goal");
		}
    	else
		{
			ROS_INFO("The base failed to move to the second goal");
		}
	}
    
  else
	{
		ROS_INFO("The base failed to move forward 1 meter for some reason");
	}
    

  return 0;
}
