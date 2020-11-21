#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
//#include "mat.hpp"

using namespace std;
using namespace cv;

#define X_OFF_PXL 0.05//0.03	
#define Y_OFF_PXL -0.45//0.05

ros::Subscriber map_sub;

int **taken;

int unknown = -1;
int black = 100;
int white = 0;
int yellow = 20;
int green = 40;
int orange = 60;
int purple = 80;
int blue = 140;
int red = 120;
int forestGreen = 200;
int square = 3;

int rWidth = 4;
int rLength = 4;

Mat cv_map;
float map_resolution = 0;
tf::Transform map_transform;

int baseWidth = 0;
int baseLength = 0;
int width = 0;
int length = 0;

int numNodes = 0;
int *xes;
int *ys;
bool cont = 1;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool checkForGreen(int i, int j){
	if(taken[i][j] == white || taken[i][j] == orange || taken[i][j] == yellow){
		return true;
	} 
	return false;
}

bool checkForOrange(int i, int j){
	if(taken[i][j] == white || taken[i][j] == yellow){
		return true;
	} 
	return false;
}

void shape1_1(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j] == black && taken[i+2][j] == black && taken[i+3][j] == black){
            if(taken[i+3][j+1] == black){
                if(checkForGreen(i+1,j+2)){
                taken[i+1][j+2] = green;
            }
        }
    }
}
}

void shape1_2(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j] == black){
            if(taken[i+3][j+1] == black){
                if(checkForGreen(i+1,j-1))
                taken[i+1][j-1] = green;
            }
        }
    }
}

void shape1_3(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i][j+3] == black){
            if(taken[i+1][j+3] == black){
                if(checkForGreen(i+2,j+2))
                taken[i+2][j+2] = green;
            }
        }
    }
}

void shape1_4(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i+1][j+3] == black){
            if(taken[i][j+3] == black){
                if(checkForGreen(i-1,j+1))
                taken[i-1][j+1] = green;
            }
        }
    }
}


void shape1(int i, int j){
    shape1_1(i, j);
    shape1_2(i, j);
    shape1_3(i, j);
    shape1_4(i, j);
}



void shape2_1(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j] == black && taken[i+2][j] == black && taken[i+2][j+1] == black){
            if(checkForGreen(i+1,j+2))
            taken[i+1][j+2] = green;
        }
    }
}

void shape2_2(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j+1] == black && taken[i+2][j] == black && taken[i+2][j+1] == black){
            if(checkForGreen(i+1,j-1))
            taken[i+1][j-1] = green;
        }
    }
}

void shape2_3(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i][j+2] == black){
           if(checkForGreen(i-1,j+1))
            taken[i-1][j+1] = green;
        }
    }
}

void shape2_4(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i+1][j+2] == black){
            if(checkForGreen(i+2,j+1))
            taken[i+2][j+1] = green;
        }
    }
}

void shape2(int i, int j){
    shape2_1(i, j);
    shape2_2(i, j);
    shape2_3(i, j);
    shape2_4(i, j);
}



void shape3_1(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+2][j] == black && taken[i+2][j+1] == black && taken[i+2][j+2] == black){
            if(checkForGreen(i,j+2))
            taken[i][j+2] = green;
        }
    }
}

void shape3_2(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+2][j] == black && taken[i][j+1] == black && taken[i][j+2] == black){
           if(checkForGreen(i+2,j+2))
            taken[i+2][j+2] = green;
        }
    }
}

void shape3_3(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i][j+2] == black && taken[i+1][j+2] == black && taken[i+2][j+2] == black){
            if(checkForGreen(i+2,j))
            taken[i+2][j] = green;
        }
    }
}

void shape3_4(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+2][j] == black && taken[i+2][j-1] == black && taken[i+2][j-2] == black){
            if(checkForGreen(i,j-2))
            taken[i][j-2] = green;
        }
    }
}

void shape3(int i, int j){
    shape3_1(i, j);
    shape3_2(i, j);
    shape3_3(i, j);
    shape3_4(i, j);
}



void shape4_1(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j] == black && taken[i+2][j] == black && taken[i+3][j] == black && taken[i+4][j]){
            if(taken[i+4][j+1] == black){
                if(checkForGreen(i+2,j+2))
                taken[i+2][j+2] = green;
            }
        }
    }
}

void shape4_2(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j+1] == black && taken[i+4][j+1]){
            if(taken[i+4][j] == black){
                if(checkForGreen(i+2,j-1))
                taken[i+2][j-1] = green;
            }
        }
    }
}

void shape4_3(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i][j+3] == black && taken[i][j+4]){
            if(taken[i+1][j+4] == black){
                if(checkForGreen(i+2,j+2))
                taken[i+2][j+2] = green;
            }
        }
    }
}

void shape4_4(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i+1][j+3] == black && taken[i+1][j+4]){
            if(taken[i][j+4] == black){
                if(checkForGreen(i-1,j+2))
                taken[i-1][j+2] = green;
            }
        }
    }
}

void shape4(int i, int j){
    shape4_1(i, j);
    shape4_2(i, j);
    shape4_3(i, j);
    shape4_4(i, j);
}



void shape5_1(int i, int j){

        if(taken[i+1][j+1] == black && taken[i+2][j+1] == black){
            if(taken[i+3][j] == black){
                if(checkForOrange(i-1,j+2))
                taken[i-1][j+2] = orange;
                if(checkForOrange(i+4,j+2))
                taken[i+4][j+2] = orange;
            }
        }

}

void shape5_2(int i, int j){

        if(taken[i+1][j-1] == black && taken[i+2][j+1] == black){
            if(taken[i+3][j] == black){
                if(checkForOrange(i-1,j-2))
                taken[i-1][j-2] = orange;
                if(checkForOrange(i+4,j-2))
                taken[i+4][j-2] = orange;
            }
        }

}

void shape5_3(int i, int j){

        if(taken[i+1][j+1] == black && taken[i+1][j+2] == black){
            if(taken[i][j+3] == black){
                if(checkForOrange(i+2,j-1))
                taken[i+2][j-1] = orange;
                if(checkForOrange(i+2,j+4))
                taken[i+2][j+4] = orange;
            }
        }
}

void shape5_4(int i, int j){
    if(taken[i+1][j-1] == black){
        if(taken[i][j+1] == black){
            if(taken[i+1][j+2] == black){
                if(checkForOrange(i-1,j-2))
                taken[i-1][j-2] = orange;
                if(checkForOrange(i-1,j+3))
                taken[i-1][j+3] = orange;
            }
        }
    }
}

void shape5(int i, int j){
    shape5_1(i, j);
    shape5_2(i, j);
    shape5_3(i, j);
    shape5_4(i, j);
}



void shape6_1(int i, int j){

    if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j+1] == black && taken[i+4][j] == black){
        if(taken[i+3][j+1] == black){
            if(checkForOrange(i-1,j+2))
                taken[i-1][j+2] = orange;
            if(checkForOrange(i+5,j+2))
                taken[i+5][j+2] = orange;
        }
    }

}

void shape6_2(int i, int j){

        if(taken[i+1][j-1] == black && taken[i+2][j-1] == black && taken[i+3][j-1] == black){
            if(taken[i+4][j] == black){
                if(checkForOrange(i-1,j-2))
                    taken[i-1][j-2] = orange;
                if(checkForOrange(i+5,j-2))
                    taken[i+5][j-2] = orange;
            }
        }

}

void shape6_3(int i, int j){

    if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i+1][j+3] == black){
        if(taken[i][j+4] == black){
            if(checkForOrange(i+2,j-1))
                taken[i+2][j-1] = orange;
            if(checkForOrange(i+2,j+5))
                taken[i+2][j+5] = orange;
        }
    }
}

void shape6_4(int i, int j){
    if(taken[i+1][j-1] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black){
            if(taken[i+1][j+3] == black){
                if(checkForOrange(i-1,j-2))
                    taken[i-1][j-2] = orange;
                if(checkForOrange(i-1,j+4))
                    taken[i-1][j+4] = orange;
            }
        }
    }
}

void shape6(int i, int j){
    shape6_1(i, j);
    shape6_2(i, j);
    shape6_3(i, j);
    shape6_4(i, j);
}


void shape7_1(int i, int j) {

    if (taken[i + 1][j + 1] == black && taken[i + 2][j + 2] == black) {
        if (taken[i + 1][j - 1] == black){
            if (taken[i][j + 3] == white)
                taken[i][j + 3] = yellow;
        }
    }
}

void shape7_2(int i, int j){

    if(taken[i+1][j-1] == black && taken[i+2][j-2] == black) {
        if (taken[i + 1][j + 1] == black) {
            if (taken[i - 1][j - 2] == white)
                taken[i - 1][j - 2] = yellow;
        }
    }
}

void shape7_3(int i, int j){

    if(taken[i + 1][j + 1] == black && taken[i + 2][j + 2] == black){
        if(taken[i-1][j+1] == black){
            if(taken[i+2][j-1] == white)
                taken[i+2][j-1] = yellow;
        }
    }
}

void shape7_4(int i, int j){
    if(taken[i+1][j-1] == black && taken[i+2][j-2] == black){
        if(taken[i-1][j-1] == black){
                if(taken[i+3][j] == white)
                    taken[i+3][j] = yellow;
            }
    }
}
void shape7(int i, int j){
    shape7_1(i, j);
    shape7_2(i, j);
    shape7_3(i, j);
    shape7_4(i, j);
}



void shape8_1(int i, int j) {

    if (taken[i + 1][j - 1] == black && taken[i + 2][j - 1] == black && taken[i + 2][j - 2] == black) {
        if (taken[i + 3][j - 2] == black){
            if (taken[i+2][j + 2] == white)
                taken[i+2][j + 2] = yellow;
            if (taken[i+3][j] == white)
                taken[i+3][j] = yellow;
        }
    }
}

void shape8_2(int i, int j){

    if(taken[i][j+1] == black && taken[i+1][j+1] == black && taken[i+1][j+2] == black) {
        if (taken[i + 2][j + 3] == black) {
            if (taken[i+2][j] == white)
                taken[i+2][j] = yellow;
            if (taken[i+4][j+1] == white)
                taken[i+4][j+1] = yellow;
        }
    }
}

void shape8_3(int i, int j){

    if(taken[i + 1][j] == black && taken[i + 1][j-1] == black){
        if(taken[i+2][j-1] == black && taken[i + 3][j-2] == black){
            if(taken[i][j-2] == white)
                taken[i][j-2] = yellow;
            if(taken[i+1][j-4] == white)
                taken[i+1][j-4] = yellow;
        }
    }
}

void shape8_4(int i, int j){
    if(taken[i+1][j] == black && taken[i+1][j+1] == black && taken[i+1][j+2] == black){
        if(taken[i+2][j+3] == black){
            if(taken[i][j+3] == white)
                taken[i][j+3] = yellow;
            if(taken[i-2][j+2] == white)
                taken[i-2][j+2] = yellow;
        }
    }
}
void shape8(int i, int j){
    shape8_1(i, j);
    shape8_2(i, j);
    shape8_3(i, j);
    shape8_4(i, j);

}


void shape9_1(int i, int j) {

    if (taken[i + 1][j] == black && taken[i + 1][j + 1] == black && taken[i + 2][j+1] == black) {
        if (taken[i + 3][j + 2] == black){
            if (taken[i][j + 2] == white)
                taken[i][j + 2] = yellow;
            if (taken[i+1][j+4] == white)
                taken[i+1][j+4] = yellow;
        }
    }
}

void shape9_2(int i, int j){

    if(taken[i+1][j] == black && taken[i+1][j-1] == black && taken[i+1][j-2] == black) {
        if (taken[i + 2][j - 3] == black) {
            if (taken[i][j-3] == white)
                taken[i][j-3] = yellow;
            if (taken[i-2][j-2] == white)
                taken[i-2][j-2] = yellow;
        }
    }
}

void shape9_3(int i, int j){

    if(taken[i + 1][j+1] == black && taken[i + 2][j+1] == black){
        if(taken[i+2][j+2] == black && taken[i + 3][j+2] == black){
            if(taken[i+3][j] == white)
                taken[i+3][j] = yellow;
            if(taken[i+2][j-2] == white)
                taken[i+2][j-2] = yellow;
        }
    }
}

void shape9_4(int i, int j){
    if(taken[i+1][j-1] == black && taken[i+1][j-2] == black && taken[i+1][j-3] == black){
        if(taken[i+2][j-3] == black){
            if(taken[i+2][j] == white)
                taken[i+2][j] = yellow;
            if(taken[i+4][j-1] == white)
                taken[i+4][j-1] = yellow;
        }
    }
}
void shape9(int i, int j){
    shape9_1(i, j);
    shape9_2(i, j);
    shape9_3(i, j);
    shape9_4(i, j);
}



void shape10_1(int i, int j){
    
        if(taken[i+1][j] == black && taken[i+2][j] == black && taken[i+3][j] == black){
            if(taken[i+4][j+1] == black){
                if(checkForOrange(i+2,j+1))
                taken[i+2][j+1] = orange;
            }
        }
    
}

void shape10_2(int i, int j){
   
        if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j+1] == black && taken[i+4][j+1]){
            
                if(checkForOrange(i+2,j))
                taken[i+2][j] = orange;
            }
}

void shape10_3(int i, int j){
    if(taken[i+1][j-1] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i][j+3] == black){
           
                if(checkForOrange(i+1,j+1))
                taken[i+1][j+1] = orange;
            }
        
    }
}

void shape10_4(int i, int j){
    if(taken[i+1][j-1] == black){
        if(taken[i+1][j-2] == black && taken[i+1][j-3] == black && taken[i+1][j-4] == black){
                if(checkForOrange(i,j-2))
                taken[i][j-2] = orange;
            }
        }
}

void shape10(int i, int j){
    shape10_1(i, j);
    shape10_2(i, j);
    shape10_3(i, j);
    shape10_4(i, j);
}

void doesItBelong(int i, int j){
    shape1(i, j);
    shape2(i, j);
    shape3(i, j);
    shape4(i, j);
    shape5(i, j);
    shape6(i, j);
    shape7(i, j);
    shape8(i, j);
    shape9(i, j);
    shape10(i, j);

}
void checkNodes(int i, int j){

	int flag = 0;

	if(taken[i][j] == green){
		for (int m = i; m<i+square-1; m++){
			for(int n = j; n<j+square-1; n++){
				if(m!=i && n!=j){
					if(taken[m][n] == green){
						taken[m][n] = red;
					} else if(taken[m][n] == orange){
						taken[m][n] = blue;
					}  else if(taken[m][n] == yellow){
						taken[m][n] = purple;
					}
				}
			}
		}
		taken[i][j] = forestGreen;
		numNodes++;
	} else if(taken[i][j] == orange){
		for (int m = i; m<i+square-1; m++){
			for(int n = j; n<j+square-1; n++){
				if(m!=i && n!=j){
					if(taken[m][n] == green){
						flag = 1;
					} else if(taken[m][n] == orange){
						taken[m][n] = blue;
					}  else if(taken[m][n] == yellow){
						taken[m][n] = purple;
					}
				}
			}
		}
		if(!flag){
				taken[i][j] = forestGreen;
				numNodes++;
		}
	} else if(taken[i][j] == yellow){
		for (int m = i; m<i+square-1; m++){
			for(int n = j; n<j+square-1; n++){
				if(m!=i && n!=j){
					if(taken[m][n] == green){
						flag = 1;
					} else if(taken[m][n] == orange){
						flag = 1;
					}  else if(taken[m][n] == yellow){
						taken[m][n] = purple;
					}
				}
			}
		}
		if(!flag){
				taken[i][j] = forestGreen;
				numNodes++;
		}
	}
}

void check(){



    for(int i=0; i<width; i++){
        for(int j=0; j<length; j++){
            if(taken[i][j] == black){
                doesItBelong(i, j);
            }
        }
    }


    for(int i=0; i<width; i++){
        for(int j=0; j<length; j++){
            if(taken[i][j] != black && taken[i][j] != white){
                checkNodes(i, j);
            }
        }
    }

xes = new int[numNodes];
ys = new int[numNodes];
int z = 0;

    for(int i=0; i<width; i++){
    	for(int j=0; j<length; j++){
    		if(taken[i][j] == forestGreen){
    			xes[z] = i * rWidth + rWidth/2;
    			ys[z] = j * rLength + rLength/2;
    			z++;
    			ROS_INFO("Adding point: [%d, %d]", xes[z], ys[z]);
    		}
    	}
    }

}


void createGrid(const nav_msgs::OccupancyGridConstPtr& msg_map){
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    //int * occup = msg->data.c_str(); //[an array] The map data, in row-major order, starting with (0,0).
    //http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html


	int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;

	if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
	tf::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
   
    int size_y_rev = size_y-1;
    int ** pixels = new int*[baseWidth];
   
    for(int i=0; i<baseWidth; i++){
		pixels[i] = new int[baseLength];
	}

    
    for(int i=0; i<baseWidth; i++){
		for(int j=0; j<baseLength; j++){
			pixels[i][j] = (int)cv_map.data[i * baseWidth + j];
		}
	}
		
    /*
    

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
	
*/
	//int width = 544;
	//int length = 576;
	baseWidth = size_y_rev;
	baseLength = size_x;

//int pixels[baseWidth][baseLength] = cv_map_data;

    width = baseWidth/rWidth + 1;
    length = baseLength/rLength + 1;

    //int * m = new int[width/rWidth];
    //int * n = new int[width/rWidth];
    //int counter = 0;
    taken = new int*[width];
    for(int i = 0; i < width; i++) {
	taken[i] = new int[length];
    }
/*
    for(int i=0; i<baseWidth; i++){
        for(int j=0; j<baseLength; j++){
            pixels[i][j] = occup[i+j];
        }
    }

*/
    for(int i=0; i<baseWidth; i+=rWidth){
        for(int j=0; j<baseLength; j+=rLength){
            int flag = 0;

            for(int w=i; w<i + rWidth; 	w++){

                for(int l=j; l<j + rLength; l++){
                    if(pixels[w][l] != 0){
                        flag = pixels[w][l];
                    }
                }
            }

            taken[i/rWidth][j/rLength] = flag;
            ROS_INFO("%d", flag);
        }
        ROS_INFO("\n");
    }

    check();

    //ROS_INFO(taken);
}




class FindGoals{

private:
	MoveBaseClient ac;
	ros::NodeHandle n;

public:
	FindGoals(ros::NodeHandle &nh): ac("move_base", true), n(nh){
	}

	~FindGoals(){}

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
			ROS_INFO("Reached initial pose of approache");
		else
			ROS_INFO("The base failed to reach the initial pose for the approach");
	}

};


void stopMap(const std_msgs::String::ConstPtr& msg)
{
  cont = 0;
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char** argv) {
	int a = 1;

	//createGrid();
    ros::init(argc, argv, "send_goals_dyn");
    ROS_INFO("createGrid");
    ROS_INFO("a: %d", a);
    ros::NodeHandle n;
    
    ROS_INFO("NodeHandle");
    FindGoals fg(n);
    map_sub = n.subscribe("map", 10, &createGrid);
    //ros::Subscriber sub = n.subscribe("nav_msgs/OccupancyGrid", 1000, chatterCallback);    
int z = 0;

ros::Subscriber sub = n.subscribe("stopMap", 1000, &stopMap);
   
	ROS_INFO("subscribed");
    while(ros::ok()){
		
	ROS_INFO("while...");
		ros::spinOnce();
		
		if(z < numNodes && cont){
			fg.pxl_goal(xes[z], ys[z], 0);
			z++;
		} else {
			break;
		}
			
		cont = 1;
    }

    return 0;
}
