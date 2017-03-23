//The program subscribes to turtle1/pose and prints the messages
#include <ros/ros.h>
//#include "exercise2/Input.h" //The message type
 #include "std_msgs/String.h"
#include <sstream>
//This is a callback function. It executes every time a new pose message arrives
void messageReceived(const std_msgs::String::ConstPtr& msg){	//The parameter is the message type

	ROS_INFO("I recieved the string: %s", msg->data.c_str());
	//ROS_INFO("The ID is: %d", msg->number);
	//system("rosnode kill -a");

	if(strcmp(msg->data.c_str(), "flowerWalk") == 0){
		system("roslaunch exercise2 flowerWalk.launch");
	} else if(strcmp(msg->data.c_str(), "circleWalk") == 0){
		system("roslaunch exercise2 circlewalk.launch");
	} else if(strcmp(msg->data.c_str(), "moonWalk") == 0){
		system("roslaunch exercise2 backForthWalk.launch");
	} else if(strcmp(msg->data.c_str(), "rectangleWalk") == 0){
		system("roslaunch exercise2 rectangle.launch");
	} else if(strcmp(msg->data.c_str(), "randomWalk") == 0){
		system("roslaunch exercise2 randomwalk.launch");
	} 

}

int main(int argc, char **argv){

	//Initialize ROS system, become a registered node
	ros::init(argc, argv,"our_custom_subscriber");
	ros::NodeHandle nh;

	//Create a subscriber objet
	ros::Subscriber sub=nh.subscribe("/move_type",1000,&messageReceived);	//First parameter is the topic
																				//Second parameter is the queue size
																				//Third parameter is a ponter to a
																					//callback function that will execute
																					//each time a new message is recieved

	//Ros will take over after this
	ros::spin();
	//the same thing can be done with a loop like:
		//while(ros::ok()){  ros::spinOnce(); }
}
