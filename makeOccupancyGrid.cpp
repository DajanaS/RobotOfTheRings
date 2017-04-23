#include "ros/ros.h"
#include "std_msgs/String.h"

void createGrid(const std_msgs::String::ConstPtr& msg){
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	int * occup = msg->data.c_str(); //The map data, in row-major order, starting with (0,0). 
	//http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html

}


int main(int argc, char **argv){

	ros::init(argc, argv, "createGrid");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("nav_msgs/OccupancyGrid", 1000, chatterCallback);
	ros::spin();
	return 0;

}