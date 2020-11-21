#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <string>
#include "read_goals.hpp"
#include <cmath>
#include "std_msgs/String.h"

#ifndef PI
#define PI 3.14159265359
#endif

using namespace std;
using namespace cv;

bool go_on = true;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void stop_route(const std_msgs::String::ConstPtr& msg)
{
   ROS_INFO("Stopping route now!");
   go_on = false;
}

move_base_msgs::MoveBaseGoal* get_goal(double* points)
{   
	move_base_msgs::MoveBaseGoal* goal = new move_base_msgs::MoveBaseGoal;
	goal->target_pose.header.frame_id = "map";
  	goal->target_pose.header.stamp = ros::Time::now();
 	goal->target_pose.pose.position.x = points[0];
 	goal->target_pose.pose.position.y = points[1];
  	goal->target_pose.pose.orientation.z = points[2];
  	goal->target_pose.pose.orientation.w = points[3];
	return goal;
}

move_base_msgs::MoveBaseGoal* do_a_turn(double* points)
{   
	move_base_msgs::MoveBaseGoal* goal = new move_base_msgs::MoveBaseGoal;
	goal->target_pose.header.frame_id = "map";
  	goal->target_pose.header.stamp = ros::Time::now();
 	goal->target_pose.pose.position.x = points[0];
 	goal->target_pose.pose.position.y = points[1];
 	
	static int i = 1;
	double theta = -PI+i*2*PI/4;
	goal->target_pose.pose.orientation.z = sin(theta/2);
  	goal->target_pose.pose.orientation.w = cos(theta/2);
	ROS_INFO("Theta %f", theta);
	if(i == 3) i = 1;
	else ++i;
	return goal;
}
 
 int main(int argc, char** argv) {

    ros::init(argc, argv, "send_goals");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("goal_reached", 1000, &stop_route);

	MoveBaseClient ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(1.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

	ROS_INFO("Start reading goals");
	double goals[100][4];
	string path = "/home/marie/catkin_ws/Route_4.txt";
	int numb_goals = createGoals_short(goals, path);
	ROS_INFO("I got %d new goals", numb_goals);

	for(int i = 0; i < numb_goals; ++i)
	{
		if(!go_on) break;
		ROS_INFO("Sending goal %d",i);
		move_base_msgs::MoveBaseGoal* goal = get_goal(goals[i]);
		ac.sendGoal(*goal);
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the base reached the goal");
		else
			ROS_INFO("The base failed to move to the destination for some reason");
		delete goal;
		ros::Rate r(1);
		for(int j = 0; j < 3; ++j)
		{
			if(!go_on) break;
			ROS_INFO("Turning %d",j);
			goal = do_a_turn(goals[i]);
			ac.sendGoal(*goal);
			ac.waitForResult();
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("Hooray, the base reached the goal");
			else
				ROS_INFO("The base failed to move to the destination for some reason");
			delete goal;
			//r.sleep();
		}
	}
    return 0;
}
