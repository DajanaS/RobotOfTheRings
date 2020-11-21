#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include "get_coordinate.hpp"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#ifndef PI
#define PI 3.14159265359
#endif

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Point* approach(const tf::TransformListener& listener, Point face)
{
	geometry_msgs::PoseStamped pBase, pMap;
	pBase.header.frame_id = "base_link";
	pBase.pose.position.x = 0.0;
	pBase.pose.position.y = 0.0;
	pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	ros::Time current_transform = ros::Time::now();
	ROS_INFO("Waiting for transform");
	listener.waitForTransform(pBase.header.frame_id, "map",current_transform, ros::Duration(3.0));
	listener.getLatestCommonTime(pBase.header.frame_id, "map", current_transform, NULL);
	pBase.header.stamp = current_transform;
	ROS_INFO("Doing the transform");
	listener.transformPose("map", pBase, pMap);
	
	Point robot(pMap.pose.position.x, pMap.pose.position.y);
	ROS_INFO("I am now x: %f y: %f", robot.getX(),robot.getY());
	ROS_INFO("The face is now x: %f y: %f", face.getX(),face.getY());
	Point* goal = wantedPosition(robot, face);
	return goal;
}


/*int main(int argc, char** argv)
{
	ros::init(argc, argv, "approach");
	ros::NodeHandle n;
   
    tf::TransformListener listener;
	Point face(3.74397182465,-3.192299366);
	ROS_INFO("Start approach");
	Point* new_goal = approach(listener,face);
	
	MoveBaseClient ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(1.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
	
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();
 	goal.target_pose.pose.position.x = new_goal->getX();
 	goal.target_pose.pose.position.y = new_goal->getY();
  	goal.target_pose.pose.orientation.z = sin(-PI/2);
  	goal.target_pose.pose.orientation.w = cos(-PI/2);
  	
  	ROS_INFO("Trying to reach x: %f y: %f", new_goal->getX(), new_goal->getY());
	ac.sendGoal(goal);
	ac.waitForResult();
	
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the base reached the goal");
	else
		ROS_INFO("The base failed to move to the destination for some reason");
	
   delete new_goal;
}*/
