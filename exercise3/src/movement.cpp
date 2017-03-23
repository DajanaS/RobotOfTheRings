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

using namespace std;
using namespace cv;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Mat cv_map;
float map_resolution = 0;
tf::Transform map_transform;

ros::Publisher goal_pub;
ros::Subscriber map_sub;

void reachable_goals();

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3) ) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
	tf::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y-1;

    for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y -y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }
    reachable_goals();
}

void reachable_goals()
{
	vector<tf::Point> goals;
	for(int i = 0; i < cv_map.rows; ++i)
	{
		for(int j = 0; j < cv_map.cols; ++j)
		{
			if(cv_map.at<int>(i,j) == 0) goals.push_back(tf::Point((float)i*map_resolution,(float)j*map_resolution,0.0));
		}
	}
	srand(time(NULL));
	int size = goals.size();
	MoveBaseClient ac("move_base", true);
	for(int i = 0; i < 10; ++i)
	{
		int random = rand()%size;
		tf::Point transformed = map_transform * goals.at(random);

		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "base_link";
  	  	goal.target_pose.header.stamp = ros::Time::now();
 	   	goal.target_pose.pose.position.x = transformed.x();
  	   	goal.target_pose.pose.orientation.w = -transformed.y();
		
		ROS_INFO("Sending goal %d", i);
   		ac.sendGoal(goal);
  	 
    	ac.waitForResult();
   
    	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       	ROS_INFO("Hooray, the base moved 1 meter forward");
    	else
    	ROS_INFO("The base failed to move forward 1 meter for some reason");
	}
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "map_goals");
    ros::NodeHandle n;

    map_sub = n.subscribe("map", 10, &mapCallback);
  	goal_pub = n.advertise<move_base_msgs::MoveBaseGoal>("goal", 10);
  	//goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 10);
  	
    while(ros::ok()) 
    {
       ros::spinOnce();
   	}
    return 0;

}
