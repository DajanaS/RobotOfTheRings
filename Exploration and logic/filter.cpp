#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "jsk_gui_msgs/VoiceMessage.h"

// needed package: https://github.com/jsk-ros-pkg/jsk_common_msgs/tree/master/

ros::Publisher chatter_pub;
int ready = 0;
std_msgs::String msg1;

void chatterCallback(const jsk_gui_msgs::VoiceMessage::ConstPtr& msg)
{
  for(int i = 0; i < msg->texts.size(); i++)
    ROS_INFO("I heard: [%s]", msg->texts[i].c_str());
  
  ros::NodeHandle n;
  std::stringstream ss;

  for(int i = 0; i < msg->texts.size(); i++) {
  if(strcmp(msg->texts[i].c_str(),"yes")==0) {
    ss << "yes";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"no")==0) {
    ss << "no";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"blue")==0) {
    ss << "blue";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"red")==0) {
    ss << "red";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"green")==0) {
    ss << "green";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"yellow")==0) {
    ss << "yellow";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"black")==0) {
    ss << "black";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Peter")==0) {
    ss << "Peter";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Tina")==0) {
    ss << "Tina";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Dina")==0) {
    ss << "Tina";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Forrest")==0) {
    ss << "Forrest";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Scarlett")==0) {
    ss << "Scarlett";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Tesla")==0) {
    ss << "Tesla";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"This Love")==0) {
    ss << "Tesla";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Adele")==0) {
    ss << "Adele";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Albert")==0) {
    ss << "Albert";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Ellen")==0) {
    ss << "Ellen";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Lindsey")==0) {
    ss << "Lindsey";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Harry")==0) {
    ss << "Harry";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Ilka")==0) {
    ss << "Ilka";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Google")==0) {
    ss << "Ilka";
    msg1.data = ss.str();
    break;
  } else if(strcmp(msg->texts[i].c_str(),"Elvis")==0) {
    ss << "Elvis";
    msg1.data = ss.str();
    break;
  } else {
    if(i==msg->texts.size()-1) {
      ss << "repeat";
      msg1.data = ss.str();
      break;
    }
  }
  }
  ready = 1; 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("Tablet/voice", 1000, chatterCallback);
  chatter_pub = n.advertise<std_msgs::String>("speachFiltered", 1000);
  
  ros::Rate loop_rate(10);
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
