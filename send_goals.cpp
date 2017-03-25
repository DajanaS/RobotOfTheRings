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

using namespace std;
using namespace cv;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void send_goal(double* points)
{
	
	MoveBaseClient ac("move_base", true);
		while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();
 	goal.target_pose.pose.position.x = poins[0];
 	goal.target_pose.pose.position.y = points[1];
  	goal.target_pose.pose.orientation.w = points[2];
  	goal.target_pose.pose.orientation.z = points[3];
		
	ROS_INFO("Sending goal");
   	ac.sendGoal(goal);
  	 
    ac.waitForResult();
   
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base reached the goal");
	else
    ROS_INFO("The base failed to move to the destination for some reason");
 }	
 
 void make_turn(double* points)
 {
	MoveBaseClient ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();
 	goal.target_pose.pose.position.x = points[0];
 	goal.target_pose.pose.position.y = points[1];
 	
  	goal.target_pose.pose.orientation.w = 0.732877889024;
  	goal.target_pose.pose.orientation.z = 0.680360198556;
	ROS_INFO("Turn move 1");
   	ac.sendGoal(goal);
  	ac.waitForResult();
    if(!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    ROS_INFO("Something went wrong, could not turn");
    
    goal.target_pose.pose.orientation.w = 0.928538683024;
  	goal.target_pose.pose.orientation.z = 0.371235658481;
	ROS_INFO("Turn move 2");
   	ac.sendGoal(goal);
  	ac.waitForResult();
    if(!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    ROS_INFO("Something went wrong, could not turn");
    
    goal.target_pose.pose.orientation.w = 0.999170917595;
  	goal.target_pose.pose.orientation.z = -0.0407121288048;
	ROS_INFO("Turn move 3");
   	ac.sendGoal(goal);
  	ac.waitForResult();
    if(!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    ROS_INFO("Something went wrong, could not turn");
    
	goal.target_pose.pose.orientation.w = 0.907728715199;
  	goal.target_pose.pose.orientation.z = -0.419557599865;
	ROS_INFO("Turn move 4");
   	ac.sendGoal(goal);
  	ac.waitForResult();
    if(!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    ROS_INFO("Something went wrong, could not turn");
    
    goal.target_pose.pose.orientation.w = 0.725986319584;
  	goal.target_pose.pose.orientation.z = -0.687709141845;
	ROS_INFO("Turn move 5");
   	ac.sendGoal(goal);
  	ac.waitForResult();
    if(!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    ROS_INFO("Something went wrong, could not turn");
   
	goal.target_pose.pose.orientation.w = -0.375329237111;
  	goal.target_pose.pose.orientation.z = 0.926891559876;
	ROS_INFO("Turn move 6");
   	ac.sendGoal(goal);
  	ac.waitForResult();
    if(!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    ROS_INFO("Something went wrong, could not turn");
    
    goal.target_pose.pose.orientation.w = 0.991441736315;
  	goal.target_pose.pose.orientation.z = 0.130549927205;
	ROS_INFO("Turn move 7");
   	ac.sendGoal(goal);
  	ac.waitForResult();
    if(!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    ROS_INFO("Something went wrong, could not turn");
    
    goal.target_pose.pose.orientation.w = 0.423606038301;
  	goal.target_pose.pose.orientation.z = 0.905846523598;
	ROS_INFO("Turn move 8");
   	ac.sendGoal(goal);
  	ac.waitForResult();
    if(!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    ROS_INFO("Something went wrong, could not turn");
    
    ROS_INFO("SPIN FINISEHD");
}
	 
 
 
 int main(int argc, char** argv) {

    ros::init(argc, argv, "map_goals");

	double goal[2];
	goal[0] = 0.0;
	goal[1] = 0.0;
	
	make_turn(goal);
   /* while(ros::ok()) {

        waitKey(30);

        ros::spinOnce();
    }*/
    return 0;

}
