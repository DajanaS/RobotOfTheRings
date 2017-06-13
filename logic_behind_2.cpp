#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sound_play/sound_play.h>
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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#define X_OFF_PXL 0.05//0.03
#define Y_OFF_PXL -0.45//0.05

using namespace std;
using namespace cv;



int MALE = 0;
int FEMALE = 1;
int LYING = 1;
int NOTLYING = 0;
int dist = 2;
int width = 4;
int black = 0;
long white = 2139062143;
int sqrdist = 1.42;
int mapFlag = 1;
int goToFaceFlag = 0;
int xFace = 0;
int yFace = 0;
int recognisedFace = 0;
int personFlag = 0;
string name = "";

class Person{

public:

    
    int age;
    int height;
    int gender;
	std::string name;
    int id;
    int isLying;
    float x;
    float y;
    int numQuestions;
    int yesNoAnswers;

	Person(){}
	
    Person(std::string name, int age, int height, int gender, float x, float y, int id){
        this->name = name;
        this->age = age;
        this->height = height;
        this->gender = gender;
        this->x = x;
        this->y = y;
        this->id = id;
        if(gender == MALE){
            isLying = NOTLYING;
            numQuestions = 2;
            yesNoAnswers = 1;
        } else {
            isLying = LYING;
            numQuestions = 1;
            yesNoAnswers = 0;
        }
    }

    void setPosition(float x, float y){
        this->x = x;
        this->y = y;
    }

    void setLying(bool ly){
        isLying = ly;
    }

};


std::string reply;
int flag = 1;
Person *onField = new Person[6];

Mat cv_map;

class Cylinder{

public:
	
	string colour;
    float x;
    float y;
    int isTheOne;
	
	Cylinder(){}

    Cylinder(string c, float xx, float yy){
        colour = c;
        x = xx;
        y = yy;
        isTheOne = 0;
    }
};


class Ring{

public:

	string colour;
    float x;
    float y;
    int isTheOne;

	Ring(){}
	
    Ring(string c, float xx, float yy){
        colour = c;
        x = xx;
        y = yy;
        isTheOne = 0;
    }
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Subscriber map_sub;
ros::Subscriber face_sub;
//ros::Subscriber ring_sub;
//Mat cv_map;
float map_resolution = 0;
tf::Transform map_transform;
bool map_avaiable = false;




void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
	mapFlag = 0;
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
	ROS_INFO("Map avaiable");
	map_avaiable = true;
}

class GoalSender
{
private:
	MoveBaseClient ac;
	ros::NodeHandle n;
	ros::Publisher cmd_vel_pub;
	ros::Subscriber move_sub;
	ros::Subscriber move_sub2;
    sound_play::SoundClient sc;
    move_base_msgs::MoveBaseGoal pBase, pMap;
    //tf::Transform map_transform;
    tf::TransformListener listener;
	
public:
	bool moving;
	GoalSender(ros::NodeHandle &nh): ac("move_base", true), n(nh)
	{		
		ROS_INFO("Start initializing approach");
		
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
		moving = false;
		move_sub = n.subscribe<geometry_msgs::PoseStamped>("new_ring", 100, &GoalSender::stop_goals, this);
		move_sub2 = n.subscribe<std_msgs::String>("go_on", 100, &GoalSender::new_goals, this);

		while(!ac.waitForServer(ros::Duration(1.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		ROS_INFO("Approach is set");
	}
	~GoalSender()
	{
	}
	void stop_goals(const geometry_msgs::PoseStamped::ConstPtr & msg)
	{
		moving = false;
	}
	void new_goals(const std_msgs::String::ConstPtr & msg)
	{
		moving = true;
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
		//goal_pub.publish(orientation);
		send_goal(orientation);
		//exit(EXIT_SUCCESS);
	}
	
	void send_goal(const move_base_msgs::MoveBaseGoal& goal)
	{
		ROS_INFO("Got goal and sending it now");
		ROS_INFO("x: %f, y: %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
		
		ac.sendGoal(goal);
		ROS_INFO("Waiting for result");
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Reached initial pose of approach");
		else
			ROS_INFO("The base failed to reach the goal");
			
	
	}
	
	void turn()
	{
		int time = 250;
		geometry_msgs::Twist base_cmd;
		base_cmd.angular.z = 0.6;
		ros::Rate r(20);
		ROS_INFO("Moving straight");
		for(int i = 0; i < time; ++i)
		{
			cmd_vel_pub.publish(base_cmd);
			r.sleep(); 
		}
		ROS_INFO("Finished turning");
	}
	
		
void faceCallback(const move_base_msgs::MoveBaseGoal& goal){
	xFace = goal.target_pose.pose.position.x;
	yFace = goal.target_pose.pose.position.y;
	send_goal(goal);
	goToFaceFlag = 1;
}

void personCallback(const std_msgs::String::ConstPtr& msg){
	name = msg->data.c_str();
	personFlag = 1;
}

void sleepok(int t, ros::NodeHandle &nh)
 {
   if (nh.ok()) sleep(t);
 }

    void ask(string question){
     //string fullSentence = name + ", " + question;
     cout<<question<<endl;
     sc.say(question);
     sleepok(2, n);
	}
//(int)pixels.at<uchar>(l, w);
int checkFree(int x, int y){
	Mat pixels = cv_map;
	//int val = (int)pixels.at<uchar>(l, w);
	cout<<"x: "<<x<<", y: "<<y<<" | colour: "<<pixels.at<uchar>(x + width, y)<<endl;
	cout<<"x: "<<x<<", y: "<<y<<" | colour: "<<pixels.at<int>(x + width, y)<<endl;
	if((long)cv_map.at<int>(x + width, y) == white){
		cout<<"x: "<<x<<", y: "<<y<<" | colour: "<<cv_map.at<uchar>(x - width, y)<<endl;
		if((long)cv_map.at<int>(x - width, y) == white){
			cout<<"x: "<<x<<", y: "<<y<<" | colour: "<<cv_map.at<uchar>(x, y + width)<<endl;
			if((long)cv_map.at<int>(x, y + width) == white){
				cout<<"x: "<<x<<", y: "<<y<<" | colour: "<<cv_map.at<uchar>(x, y - width)<<endl;
				if((long)cv_map.at<int>(x, y - width) == white){
					return 1;
				}
				return 0;
			}
			return 0;
		}
		return 0;
	}
	return 0;
}

void go2meters(int i){
        
    cout<<"Going 2 meters to the front"<<endl;
    pBase.target_pose.header.frame_id = "base_link";
    pBase.target_pose.pose.position.x = 0;
    pBase.target_pose.pose.position.y = 0;
    pBase.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    ros::Time current_transform = ros::Time::now();
    ROS_INFO("Waiting for transform");
    listener.waitForTransform(pBase.target_pose.header.frame_id, "map",current_transform, ros::Duration(3.0));
    listener.getLatestCommonTime(pBase.target_pose.header.frame_id, "map", current_transform, NULL);
    pBase.target_pose.header.stamp = current_transform;
    ROS_INFO("Doing the transform");
    listener.transformPose("map", pBase.target_pose, pMap.target_pose);
    float current_x = pMap.target_pose.pose.position.x;
    float current_y = pMap.target_pose.pose.position.y;
    int x = 0;
    int y = 0;

	//int val = (int)pixels.at<uchar>(l, w);
	//cv_map
	/*
	if(checkFree(current_x + dist, current_y)){
		x = current_x + dist;
		y = current_y;
	} else if(checkFree(current_x - dist, current_y)){
		x = current_x - dist;
		y = current_y;
	} else if(checkFree(current_x, current_y + dist)){
		x = current_x;
		y = current_y + dist;
	} else if(checkFree(current_x - dist, current_y)){
		x = current_x;
		y = current_y - dist;
	} else if(checkFree(current_x + dist, current_y + dist)){
		x = current_x + dist;
		y = current_y + dist;
	} else if(checkFree(current_x + dist, current_y - dist)){
		x = current_x + dist;
		y = current_y - dist;
	} else if(checkFree(current_x - dist, current_y - dist)){
		x = current_x - dist;
		y = current_y - dist;
	} else if(checkFree(current_x - dist, current_y + dist)){
		x = current_x - dist;
		y = current_y + dist;
	}
	* cout<<"x: "<<x<<", y: "<<y<<endl;
  */
	
  
     //pxl_goal(x, y, 0);
     int id = 0;
     if(i == 0) id = 1;
     else if(i == 6) id = 5;
     else id = i + 1;
     pxl_goal(onField[id].x, onField[id].y, 0);
     cout<<"Hello "<<onField[id].name<<endl;
     ask("Hello " + onField[id].name);
     pxl_goal(current_x, current_y, 0); 
}

double euclidean(double x1, double y1, double x2, double y2){
    double d = std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    return d;
}

void getReply(const std_msgs::String::ConstPtr& msg)
{
  reply = msg->data.c_str();
  std::istringstream iss(reply);
  std::vector<std::string> results((std::istream_iterator<std::string>(iss)),
                                 std::istream_iterator<std::string>());
  if(results.size() > 1){
    ask("I am bringing the " + results[1] + " ring");
    flag = 1;
		while(ros::ok()){
			ros::spinOnce();
            if(flag == 0) break;
		}
  }
  flag = 0;
}

int determineLying(string name, int id){
                ask(name + " Is " + onField[id].name + " lying");
                //wait for answer
                while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(flag == 0) break;
                }
                flag  = 1;
                if(reply == "yes"){
                    return 1;
                }
                return 0;
}

std::string femaleWhichLying(string name, int id){
    std::string answerRing = "";
                ask(name + " Is the diametar of the magical ring bigger than 10cm?");
                //wait for answer
                while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(flag == 0) break;
                }
                flag  = 1;
                go2meters(id);
                if(reply == "yes"){
                    ask(name + " Is the magical ring black?");
                    while(ros::ok()) {
                
                        //check if there is a message
                        ros::spinOnce();
                        if(flag == 0) break;
                    }
                    flag  = 1;
                    if(reply == "yes"){
                        answerRing = "blue";
                    } else {
                        answerRing = "black";
                    }
                } else {
                    ask(name + " Is the magical ring green?");
                    //wait for answer
                    while(ros::ok()) {
                        //check if there is a message
                        ros::spinOnce();
                        if(flag == 0) break;
                    }
                    flag  = 1;
                    if(reply == "yes"){
                        answerRing = "red";
                    } else {
                        answerRing = "green";
                    }
                    
                }
    return answerRing;
}

std::string femaleWhich(string name){
            ask(name + " What is the colour of the magical ring?");
            while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(flag == 0) break;
            }
            flag  = 1;
            return reply;
}

std::string femaleWhereLying(string name, int id){
    std::string answerRing = "";
    ask(name + " Is the magical tower blue?");
            //wait for answer
            while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(flag == 0) break;
            }
            flag  = 1;


            if(reply == "yes"){
                go2meters(id);
                ask(name + " Is the magical tower green?");
                //wait for answer
                while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(flag == 0) break;
            }
            flag  = 1;

                if(reply == "yes"){
                    go2meters(id);
                    ask(name + " Is the magical tower yellow?");

                    while(ros::ok()) {
                
                    //check if there is a message
                    ros::spinOnce();
                    if(flag == 0) break;
                    }
                    flag  = 1;

                    
                    if(reply == "yes"){
                        answerRing = "red";
                    } else {
                        answerRing = "yellow";
                    }
                } else {
                    answerRing = "green";
                }
            } else {
                answerRing = "blue";
            }
            return answerRing;
}

std::string femaleWhere(string name){
            ask(name + " What is the colour of the magical tower?");
            while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(flag == 0) break;
            }
            flag  = 1;
            return reply;
}

std::string maleWhere(string name, int id){

            string answerRing = "";
            
            ask(name + " Is the magical tower blue?");
            
            //wait for answer
            while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(flag == 0) break;
            }
            flag  = 1;


            if(reply == "no"){
                //sender.go2meters();
                ask(name + " Is the magical tower green?");
               
                //wait for answer
                while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(flag == 0) break;
            }
            flag  = 1;
                go2meters(id);
                if(reply == "no"){
                    
                    ask(name + " Is the magical tower yellow?");
                   

                    while(ros::ok()) {
                
                    //check if there is a message
                    ros::spinOnce();
                    if(flag == 0) break;
                    }
                    flag  = 1;

                    
                    if(reply == "no"){
                        answerRing = "red";
                    } else {
                        answerRing = "yellow";
                    }
                } else {
                    answerRing = "green";
                }
            } else {
                answerRing = "blue";
            }

            return answerRing;
}

std::string maleWhich(string name){
    string answerRing = "";
     ask(name + " Is the diametar of the magical ring bigger than 10cm?");
            
                //wait for answer
                while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(flag == 0) break;
                }
                flag  = 1;
                //sender.go2meters();
                if(reply == "yes"){
                    ask(name + " Is the magical ring green?");
                   
                    //wait for answer
                while(ros::ok()) {
                
                //check if there is a message
                    ros::spinOnce();
                    if(flag == 0) break;
                }
                flag  = 1;

                    if(reply == "yes"){
                        answerRing = "green";
                    } else {
                        answerRing = "red";
                    }
                } else {
                    ask(name + " Is the magical ring black?");
                    while(ros::ok()) {
                
                    //check if there is a message
                        ros::spinOnce();
                        if(flag == 0) break;
                    }
                    flag  = 1;
                    if(reply == "yes"){
                        answerRing = "black";
                    } else {
                        answerRing = "blue";
                    }
                }

        return answerRing;
}


};



int main(int argc, char **argv) {
	
	ros::init(argc, argv, "sound_play_test");
	    
    ros::NodeHandle n;
    
    GoalSender sender(n);
    ros::Subscriber sub1 = n.subscribe("speachFiltered", 1000, &GoalSender::getReply, &sender);
    ros::Subscriber person_sub = n.subscribe("person", 1000, &GoalSender::personCallback, &sender);
	map_sub =  n.subscribe("map", 10, &mapCallback);
	face_sub = n.subscribe("facesPos", 100, &GoalSender::faceCallback, &sender);

	ros::Publisher readyToRecognise = n.advertise<std_msgs::String>("recognize", 100);

	while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(mapFlag == 0) break;
     }
    int i = 0;
    Person p1("Peter", 25, 182, MALE, 0, 0, i++);
    Person p2("Tina", 33, 171, FEMALE, 0, 0, i++);
    Person p3("Forrest", 42, 183, MALE, 0, 0, i++);
    Person p4("Scarlett", 31, 160, FEMALE, 0, 0, i++);
    Person p5("Tesla", 45, 188, MALE, 0, 0, i++);
    Person p6("Adele", 29, 175, FEMALE, 0, 0, i++);
    Person p7("Harry", 18, 165, MALE, 0, 0, i++);
    Person p8("Ellen", 59, 170, FEMALE, 0, 0, i++);
    Person p9("Lindsey", 32, 178, FEMALE, 0, 0, i++);
    Person p10("Albert", 71, 180, MALE, 0, 0, i++);
    Person p11("Ilka", 26, 172, FEMALE, 0, 0, i++);
    Person p12("Elvis", 34, 181, MALE, 0, 0, i++);
    cout<<"i: "<<i<<endl;
    
    Person *allThePeople = new Person[12];
    allThePeople[0] = p1;
    allThePeople[1] = p2;
    allThePeople[2] = p3;
    allThePeople[3] = p4;
    allThePeople[4] = p5;
    allThePeople[5] = p6;
    allThePeople[6] = p7;
    allThePeople[7] = p8;
    allThePeople[8] = p9;
    allThePeople[9] = p10;
    allThePeople[10] = p11;
    allThePeople[11] = p12;

/*

	p9.x = 326;
	p9.y = 191;
	
	//p10.x = 300;
	//p10.y = 211;
	
	p4.x = 352;
	p4.y = 180;
	
	p11.x = 303;
	p11.y = 208;
	
	p7.x = 305;
	p7.y = 194;
	
	p12.x = 322;
	p12.y = 182;
	
	p5.x = 339;
	p5.y = 177;

	onField[0] = p5;
	onField[1] = p4;
	onField[2] = p11;
	onField[3] = p7;
	onField[4] = p12;
	onField[5] = p9;
	
	//cout<<"X: "<<onField[0].x<<endl;
    //cout<<"Y: "<<onField[0].y<<endl;
	
	
*/

int k = 0;
while(k<6){
	cout<<"Finding face number "<<k+1<<endl;
	while(ros::ok()){
		if(goToFaceFlag == 1) break;
		ros::spinOnce();
	}
	goToFaceFlag = 0;
	std_msgs::String msg;
	msg.data = "ready";
	readyToRecognise.publish(msg);
	while(ros::ok()){
		if(personFlag == 1)break;
		ros::spinOnce();
	}
	int j=0;
	for(int i=0; i<12; i++){
		if(allThePeople[i].name == name){
			onField[j] = allThePeople[i];
			j++;
		}
	}
	k++;
}
	
	//get which face it is
	
	//publish that we are ready for recoginiton
	
	//sub to the topic in order to get the recognised face

    Cylinder *towers;
    towers = new Cylinder[4];
    Cylinder c1("green", 278, 184);
    Cylinder c2("blue", 273, 247);
    Cylinder c3("yellow", 355, 179);
    Cylinder c4("red", 332, 170);
    towers[0] = c1;
    towers[1] = c2;
    towers[2] = c3;
    towers[3] = c4;
	
    Ring *rings;
    rings = new Ring[4];

    //fill persons on field up here
    
    //get indeces

    // we take the first man we can find, and go ask him
    string name = "";
    int idMan = 0;

    for(int i = 0; i<6; i++){
        if(onField[i].gender == MALE){
            name = onField[i].name;
            idMan = i;
            
            break;
        }
    }
	cout<<onField[idMan].name<<endl;
    sender.pxl_goal(onField[idMan].x, onField[idMan].y, 0); // --------------------------------------- can we do it this way? ------------------
    
    //ask questions
    int tellingTruth = 0;
    int firstLady = 0;
    for(int i=0; i<6; i++){
        if(onField[i].gender == FEMALE){
            
			//sender.pxl_goal(onField[i].x, onField[i].y, 0);
			int l = sender.determineLying(onField[idMan].name, i);
            if(l == 1){
				cout<<"Lying!"<<endl;
                tellingTruth = 0;
            } else {
				cout<<"Not lying!"<<endl;
                tellingTruth = 1;
            }
            if(tellingTruth){
                //firstLady = onField[i].id;
                firstLady = i;
                break;
            }
            if(i%2 == 0){
                //go 2 meters to the front and back
                sender.go2meters(idMan);
            }
        }
    }

    //go to the first lady
    cout<<"x: "<<onField[firstLady].x<<", y: "<<onField[firstLady].y<<endl;
    sender.pxl_goal(onField[firstLady].x, onField[firstLady].y, 0);
    
    
    int knowsWhichRing = 0;
    int knowsWhereRing = 0;
    int lWhich = 0;
    int lWhere = 0;
    string cyl;
    string mRing;
    //ask her who knows where the ring should be taken "What is the name of the person who knows where I should take the ring?"
    sender.ask(onField[firstLady].name + " what is the name of the person who knows where I should take the magical ring?");
    //wait for answer
    while(ros::ok()) {
                
        //check if there is a message
        ros::spinOnce();
        if(flag == 0) break;
    }
    flag  = 1;


    for(int i=0; i<6; i++){
        if(onField[i].name == reply){
            knowsWhereRing = i;
            break;
        }
    }

    sender.go2meters(firstLady);
    //ask her who knows which the ring should be taken "What is the name of the person who knows which ring is magical?"
    sender.ask(onField[firstLady].name + " what is the name of the person who knows which ring is magical?");
	//wait for answer
    while(ros::ok()) {
                
                //check if there is a message
                ros::spinOnce();
                if(flag == 0) break;
    }
    flag  = 1;



    for(int i=0; i<6; i++){
        if(onField[i].name == reply){
            knowsWhichRing = i;
            break;
        }
    }

    if(onField[knowsWhereRing].gender == FEMALE && onField[knowsWhichRing].gender == FEMALE){
        cout<<"Both female"<<endl;
        for(int i=0; i<6; i++){
			if(onField[i].gender == MALE){
				//go to the first man, and ask him if they're lying
                lWhere = sender.determineLying(onField[i].name, knowsWhereRing);
				lWhich = sender.determineLying(onField[i].name, knowsWhichRing);
				break;
			}
		}
		
        sender.pxl_goal(onField[knowsWhichRing].x, onField[knowsWhichRing].y, 0);
		if(lWhich){
				mRing = sender.femaleWhichLying(onField[knowsWhichRing].name, knowsWhichRing);
		} else {
                mRing = sender.femaleWhich(onField[knowsWhichRing].name);
        }
		
        sender.pxl_goal(onField[knowsWhereRing].x, onField[knowsWhereRing].y, 0);
		if(lWhere){
			    cyl = sender.femaleWhereLying(onField[knowsWhereRing].name, knowsWhereRing);
		} else {
                cyl = sender.femaleWhere(onField[knowsWhereRing].name);
        }

    } else if(onField[knowsWhereRing].gender == FEMALE || onField[knowsWhichRing].gender == FEMALE){
        //go to the one of these two who is a man, and ask him the info, and then ask if the woman is lying
        if(onField[knowsWhereRing].gender == MALE){
                sender.pxl_goal(onField[knowsWhereRing].x, onField[knowsWhereRing].y, 0);
                cyl = sender.maleWhere(onField[knowsWhereRing].name, knowsWhereRing);
                int whichLL = sender.determineLying(onField[knowsWhereRing].name, knowsWhichRing);

            sender.pxl_goal(onField[knowsWhichRing].x, onField[knowsWhichRing].y, 0);
            if(whichLL == 1){
                mRing = sender.femaleWhichLying(onField[knowsWhichRing].name, knowsWhichRing);
            } else {
                mRing = sender.femaleWhich(onField[knowsWhichRing].name);
            }
			//find out where the ring needs to be taken
			//find out if the woman is lying
			//go to the woman and find out what you need
		} else {
			//still the same, just the other way around
            
            sender.pxl_goal(onField[knowsWhichRing].x, onField[knowsWhichRing].y, 0);
            mRing = sender.maleWhich(onField[knowsWhichRing].name);

                sender.go2meters(knowsWhichRing);

                int whereLL = sender.determineLying(onField[knowsWhichRing].name, knowsWhereRing);
                
                if(whereLL == 1){
                    cyl = sender.femaleWhereLying(onField[knowsWhereRing].name, knowsWhereRing);
                    
                } else {
                    cyl = sender.femaleWhere(onField[knowsWhereRing].name);
                }
		}
    } else {
		//both are male
        //just go there and ask
         sender.pxl_goal(onField[knowsWhichRing].x, onField[knowsWhichRing].y, 0);
         mRing = sender.maleWhich(onField[knowsWhichRing].name);

         sender.pxl_goal(onField[knowsWhereRing].x, onField[knowsWhereRing].y, 0);
         cyl = sender.maleWhere(onField[knowsWhereRing].name, knowsWhereRing);   
	}



    //you presumably already have the rings, and the positions of the cylinders, so just bring it home
    //bow
    int cylinder = 0;
    //go to the position of the cylinder
    for(int i=0; i<4; i++){
        if(towers[i].colour == cyl){
            cylinder = i;
            break;
        }
    }
    sender.pxl_goal(towers[cylinder].x, towers[cylinder].y, 0);
    sender.ask("I am delivering the " + mRing + " ring to the " + cyl + " tower. Cheers!");
    return 0; 
}
