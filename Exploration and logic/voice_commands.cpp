#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

ros::Publisher chatter_pub;
int ready = 0;
std_msgs::String msg1;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  

  ros::NodeHandle n;
  //std::stringstream ss;
  //ss << "hello world ";
  //msg1.data = ss.str();
  msg1.data = msg->data.c_str();
  ROS_INFO("%s", msg1.data.c_str());
  ready = 1;

 
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "filter");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("Tablet/voice", 1000, chatterCallback);
  chatter_pub = n.advertise<std_msgs::String>("speachFiltered", 1000);
  

  ros::Rate loop_rate(10);
	ROS_INFO("Start spinning");
	
	while(ros::ok()){
		while (ros::ok())
		  {
			if(ready == 1) break;
			ros::spinOnce();
		  }
		  ready = 0;
		  chatter_pub.publish(msg1);
		  		
	}
  

  return 0;
}
