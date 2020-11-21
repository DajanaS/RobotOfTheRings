#include <ros/ros.h>
#include "std_msgs/String.h"


int main(int argc, char** argv){
	
	
	ROS_INFO("Entered main.");
	ros::init(argc, argv, "find_goals");
	ROS_INFO("Found goals.");
	ros::NodeHandle n;
	ROS_INFO("Node handeled.");
	return 0;
}
