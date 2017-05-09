#include "ros/ros.h"

#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/console.h>
#include <vector>
#include <vector>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>

#include <nav_msgs/GetMap.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/flann.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

#define X 2.3423
#define Y 0.683258
#define X_OFF_PXL 0.05//0.03
#define Y_OFF_PXL -0.45//0.05

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
int range = 10;


#define X_OFF -0.05
#define Y_OFF 0.45
#define Y_PXL 1

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Subscriber map_sub;
//ros::Subscriber ring_sub;
Mat cv_map;
float map_resolution = 0;
tf::Transform map_transform;
bool map_avaiable = false;

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
	ROS_INFO("Creating map");
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
	map_avaiable = true;
}

class Approach
{
private:	
	bool new_ring;
	MoveBaseClient ac;
	ros::NodeHandle n;
	ros::Publisher cmd_vel_pub;
	ros::Publisher goal_pub;
	ros::Subscriber ring_sub;
	tf::TransformListener listener;
	geometry_msgs::PoseStamped ring_pos;
	
public: 

	void set_ring_position(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		ring_pos.header.frame_id = msg->header.frame_id;
		ring_pos.pose = msg->pose;
		ROS_INFO("Got new ring!");
		set_init(ring_pos.pose.position.x, ring_pos.pose.position.y);
	}
	
	Approach(ros::NodeHandle &nh): ac("move_base", true), n(nh)
	{		
		ROS_INFO("Start initializing approach");
		
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
		goal_pub = n.advertise<move_base_msgs::MoveBaseGoal>("init_orientation", 1000);
		
		ring_sub = n.subscribe<geometry_msgs::PoseStamped>("new_ring", 100, &Approach::set_ring_position, this);
		
		new_ring = false;
		while(!ac.waitForServer(ros::Duration(1.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		ROS_INFO("Approach is set");
	}
	~Approach()
	{
	}
	
	void set_init(double x, double y)
	{
		Mat src, src_gray, detected_edges;
		
		ROS_INFO("Clicked point: x: %f, y: %f", x,y);//msg.pose.position.x, msg.pose.position.y);
		tf::Point pt( x+X_OFF, -(y+Y_OFF), 0.0);
		tf::Point transformed = map_transform.inverse()*pt;
		int map_x = round(transformed.x()/map_resolution);
		int map_y = round(transformed.y()/map_resolution);
		ROS_INFO("Pixels: x: %i, y: %i", map_x, map_y);
		
		ROS_INFO("Reduce noise");
		/// Reduce noise with a kernel 3x3
		blur(cv_map, detected_edges, Size(3,3) );
		/// Canny detector
		ROS_INFO("Starting detection");
		Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
		cout << "Did the canny detection " << detected_edges.size() << endl;
		vector<Point2f> pointsForSearch;
		for(int i = 0; i < detected_edges.rows; ++i)
		{
			for(int j = 0; j < detected_edges.cols; ++j)
			{
				if(detected_edges.at<char>(i,j) != 0)
				pointsForSearch.push_back(Point((float)j,(float)i));
			}
		}
		cout << "edges: " << pointsForSearch.size() << endl;
		//flann::LinearIndexParams indexParams;
		flann::KDTreeIndexParams indexParams;
		Mat search = Mat(pointsForSearch).reshape(1,pointsForSearch.size());
		flann::Index kdtree(search, indexParams);
		vector<float> query;
		query.push_back(map_x);
		query.push_back(map_y);
		vector<int> indices;
		vector<float> dists;
		cout << "Doing the search" << endl;
		int nn = 4;
		kdtree.knnSearch(query, indices, dists, nn);
		cout << query.at(0) << " " << query.at(1) << endl;
		for(unsigned int i = 0; i < indices.size(); ++i)
		{
			cout << indices.at(i)  << " " << dists.at(i) << " " << pointsForSearch.at(indices.at(i)) <<endl;
		}
		
		vector<Point2f> contours;
		for(unsigned int i = 0; i < indices.size(); ++i)
		{
			contours.push_back(pointsForSearch.at(indices.at(i)));
		}
		/*int dx = pointsForSearch.at(indices.at(0)).x - pointsForSearch.at(indices.at(1)).x;
		int dy = pointsForSearch.at(indices.at(0)).y - pointsForSearch.at(indices.at(1)).y;
		ROS_INFO("dx: %d, dy: %d", dx, dy);
		tf::Vector3 direction_(dx,dy,0);*/
		
		Vec4f line;
		cv::fitLine(contours,line,CV_DIST_L2,0,0.01,0.01);
		tf::Vector3 direction(line[0],line[1],0);
		
		tf::Matrix3x3 rotation;
		rotation.setEulerYPR(-M_PI/2,0,0);
		tf::Vector3 normal = direction*rotation;
		
		int check_x = pointsForSearch.at(indices.at(0)).x+2*normal.x();
		int check_y = pointsForSearch.at(indices.at(0)).y+2*normal.y();
		int v = (int)cv_map.at<unsigned char>(check_y , check_x );
		check_x = pointsForSearch.at(indices.at(0)).x-2*normal.x();
		check_y = pointsForSearch.at(indices.at(0)).y-2*normal.y();
		int w = (int)cv_map.at<unsigned char>( check_y, check_x);
		
		ROS_INFO("v: %d, w: %d", v,w);
		if(v == 0 && w == 255)
		{
			normal *= -1.0;
		}
		else if(v == 255 && w == 0)
		{
			direction *= -1.0;
		}
		else ROS_INFO("Something went with the directions wrong!");
		{
			exit(EXIT_FAILURE);
		}
		ROS_INFO("norm dx: %f, dy: %f", normal.x(), normal.y());
		ROS_INFO("direction dx: %f, dy: %f", direction.x(), direction.y());
		
		int dist_n = 4;
		int dist_d = 15;
		int init_x = round(map_x + dist_n*normal.x() - dist_d*direction.x());
		int init_y = round(map_y + dist_n*normal.y() - dist_d*direction.y());
		
		double yaw = atan2(-direction.y(),direction.x());
		pxl_goal(init_x, init_y, yaw);
		
		for(int i = 0; i < 2; ++i)
		{
			ROS_INFO("approach %i",i);
			approach(x,y);
			move_straight(8);
		}
		move_straight(30);
		ROS_INFO("Approach done");
	}
	void pxl_goal(int init_x, int init_y, double yaw)
	{
		ROS_INFO("Got pxl: x: %d, y: %d", init_x, init_y);
		tf::Point pt_goal((float)init_x * map_resolution, (float)init_y * map_resolution, 0.0);
		tf::Point init_goal = map_transform * pt_goal;
		
		move_base_msgs::MoveBaseGoal orientation;
		orientation.target_pose.header.frame_id = "map";
		orientation.target_pose.header.stamp = ros::Time::now();
		orientation.target_pose.pose.position.x = init_goal.x()-X_OFF_PXL;
		orientation.target_pose.pose.position.y = -(init_goal.y()-Y_OFF_PXL);
		orientation.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);//atan2(-direction.y(),direction.x()));
		goal_pub.publish(orientation);
		//send_goal(orientation);
		exit(EXIT_SUCCESS);
	}
	
	void send_goal(const move_base_msgs::MoveBaseGoal& goal)
	{
		ROS_INFO("Got goal and sending it now");
		ROS_INFO("x: %f, y: %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
		ac.sendGoal(goal);
		ROS_INFO("Waiting for result");
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Reached initial pose of approache");
		else
			ROS_INFO("The base failed to reach the initial pose for the approach");
	}
	
	void move_straight(int time)
	{
		geometry_msgs::Twist base_cmd;
		base_cmd.linear.x = 0.25;
		ros::Rate r(20);
		ROS_INFO("Moving straight");
		for(int i = 0; i < time; ++i)
		{
			cmd_vel_pub.publish(base_cmd);
			r.sleep(); 
		}
	}
	
	void approach(double x, double y)
	{
		move_base_msgs::MoveBaseGoal pBase, pMap;
		pBase.target_pose.header.frame_id = "base_link";
		pBase.target_pose.pose.position.x = 0.1;
		pBase.target_pose.pose.position.y = -0.1;
		pBase.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		ros::Time current_transform = ros::Time::now();
		ROS_INFO("Waiting for transform");
		listener.waitForTransform(pBase.target_pose.header.frame_id, "map",current_transform, ros::Duration(3.0));
		listener.getLatestCommonTime(pBase.target_pose.header.frame_id, "map", current_transform, NULL);
		pBase.target_pose.header.stamp = current_transform;
		ROS_INFO("Doing the transform");
		listener.transformPose("map", pBase.target_pose, pMap.target_pose);
		
		
		double dx = x-pMap.target_pose.pose.position.x;//ring_pose->target_pose.pose.position.x-pMap.target_pose.pose.position.x;
		double dy = y-pMap.target_pose.pose.position.y;//ring_pose->target_pose.pose.position.y-pMap.target_pose.pose.position.y;
		pMap.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(dy,dx));
		ac.sendGoal(pMap);
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Adjusted pose for approache");
		else
			ROS_INFO("The base failed to adjust the pose for the approach");
	}
	

};

int main(int argc, char** argv) {

    ros::init(argc, argv, "find_orientation");
    ros::NodeHandle n;
    Approach a(n);

    map_sub = n.subscribe("map", 10, &mapCallback);

    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;

}
