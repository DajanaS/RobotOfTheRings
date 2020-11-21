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

using namespace cv;
using namespace std;

int ** taken;

int baseWidth;
int baseLength;
int width = 0;
int length = 0;
int rWidth = 4;
int rLength = 4;
int gray = 127;
int unknown = -1;

int black = 0;
int white = 255;
int yellow = 20;
int green = 40;
int orange = 60;
int purple = 80;
int blue = 140;
int red = 120;
int forestGreen = 200;
int square = 5;

int numNodes = 0;
int *xes;
int *ys;
int *xAr;
int *yAr;
bool cont = 1;
int numGreen = 0;
int breakingPoint = 5;
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool isAllowed(int i, int j){
	if(i >= 0 && i <width && j>=0 && j<length){
		return true;
	}
	return false;
}

bool checkForGreen(int i, int j){
	//cout<<"1.Green ["<<i<<"]["<<j<<"]: "<<endl;
	if(i >= 0 && i <width && j>=0 && j<length){
		//cout<<"2.Green ["<<i<<"]["<<j<<"]: "<<taken[i][j]<<endl;
		if(taken[i][j] == white || taken[i][j] == orange || taken[i][j] == yellow){
			return true;
		}
	}
	 
	return false;
}

bool checkForOrange(int i, int j){
	//cout<<"1.Orange ["<<i<<"]["<<j<<"]: "<<endl;
	if(i >= 0 && i <width && j>=0 && j<length){
		//cout<<"2.orange ["<<i<<"]["<<j<<"]: "<<taken[i][j]<<endl;
			if(taken[i][j] == white || taken[i][j] == yellow){
		return true;
		}
	}
	 
	return false;
}

bool checkForYellow(int i, int j){
	//cout<<"Yellow ["<<i<<"]["<<j<<"]: "<<endl;
	if(i >= 0 && i <width && j>=0 && j<length){
		//cout<<"Yellow ["<<i<<"]["<<j<<"]: "<<endl;
		if(taken[i][j] == white){
		return true;
	} 
	}
	
	return false;
}

void shape1_1(int i, int j){
	
	if(isAllowed(i, j+1) && isAllowed(i+1, j) && isAllowed(i+2, j) && isAllowed(i+3, j) && isAllowed(i+3, j+1)){
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
}

void shape1_2(int i, int j){
    if(isAllowed(i, j+1) && isAllowed(i+1, j+1) && isAllowed(i+2, j+1) && isAllowed(i+3, j) && isAllowed(i+3, j+1)){
		if(taken[i][j+1] == black){
        if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j] == black){
            if(taken[i+3][j+1] == black){
                if(checkForGreen(i+1,j-1))
                taken[i+1][j-1] = green;
            }
        }
    }
	}
}

void shape1_3(int i, int j){
     if(isAllowed(i+1, j) && isAllowed(i, j+1) && isAllowed(i, j+2) && isAllowed(i, j+3) && isAllowed(i+1, j+3)){
			if(taken[i+1][j] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i][j+3] == black){
            if(taken[i+1][j+3] == black){
                if(checkForGreen(i+2,j+2))
                taken[i+2][j+2] = green;
            }
        }
    }
	}
}

void shape1_4(int i, int j){
	//cout<<"Green [no problem]: "<<endl;
	if(isAllowed(i+1, j) && isAllowed(i+1, j+1) && isAllowed(i+1, j+2) && isAllowed(i+1, j+3) && isAllowed(i, j+3)){
		//cout<<"Green [problem]: "<<endl;	
		if(taken[i+1][j] == black){
			//cout<<"1: "<<endl;	
			if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i+1][j+3] == black){
				//cout<<"2: "<<endl;	
				if(taken[i][j+3] == black){		
					//cout<<"3: "<<endl;				
					if(checkForGreen(i-1,j+1)){
						//cout<<"Green [Green]: "<<endl;
						taken[i-1][j+1] = green;
					} else {
						//cout<<"Red Green: "<<endl;
					}
					
				}
			}
		}
	}
	
}


void shape1(int i, int j){
   
    shape1_1(i, j);
   //cout<<"11: does it belong."<<endl;
    shape1_2(i, j);
    //cout<<"12: does it belong."<<endl;
    shape1_3(i, j);
    //cout<<"13: does it belong."<<endl;
    //cout<<"["<<i<<"]["<<j<<"]: "<<taken[i][j]<<endl;
    shape1_4(i, j);
   // cout<<"14: does it belong."<<endl;
}



void shape2_1(int i, int j){
	if(isAllowed(i, j+1) && isAllowed(i+1, j) && isAllowed(i+2, j) && isAllowed(i+2, j+1)){
		if(taken[i][j+1] == black){
        if(taken[i+1][j] == black && taken[i+2][j] == black && taken[i+2][j+1] == black){
            if(checkForGreen(i+1,j+2))
            taken[i+1][j+2] = green;
        }
    }
	}
    
}

void shape2_2(int i, int j){
	if(isAllowed(i, j+1) && isAllowed(i+1, j+1) && isAllowed(i+2, j) && isAllowed(i+2, j+1)){
		if(taken[i][j+1] == black){
        if(taken[i+1][j+1] == black && taken[i+2][j] == black && taken[i+2][j+1] == black){
            if(checkForGreen(i+1,j-1))
            taken[i+1][j-1] = green;
        }
    }
	}
    
}

void shape2_3(int i, int j){
	if(isAllowed(i+1, j) && isAllowed(i+1, j+1) && isAllowed(i+1, j+2) && isAllowed(i, j+2)){
		if(taken[i+1][j] == black){
        if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i][j+2] == black){
           if(checkForGreen(i-1,j+1))
            taken[i-1][j+1] = green;
        }
    }
	}
    
}

void shape2_4(int i, int j){
	if(isAllowed(i+1, j) && isAllowed(i, j+1) && isAllowed(i, j+2) && isAllowed(i+1, j+2)){
		if(taken[i+1][j] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i+1][j+2] == black){
            if(checkForGreen(i+2,j+1))
            taken[i+2][j+1] = green;
        }
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
    if(isAllowed(i+1, j) && isAllowed(i+2, j) && isAllowed(i+2, j+1) && isAllowed(i+2, j+2)){
			if(taken[i+1][j] == black){
        if(taken[i+2][j] == black && taken[i+2][j+1] == black && taken[i+2][j+2] == black){
            if(checkForGreen(i,j+2))
            taken[i][j+2] = green;
        }
    }
	}
    
}

void shape3_2(int i, int j){
	if(isAllowed(i+1, j) && isAllowed(i+2, j) && isAllowed(i, j+1) && isAllowed(i, j+2)){
			if(taken[i+1][j] == black){
        if(taken[i+2][j] == black && taken[i][j+1] == black && taken[i][j+2] == black){
           if(checkForGreen(i+2,j+2))
            taken[i+2][j+2] = green;
        }
    }
	}
    
}

void shape3_3(int i, int j){
	if(isAllowed(i, j+1) && isAllowed(i, j+2) && isAllowed(i+1, j+2) && isAllowed(i+2, j+2)){
			if(taken[i][j+1] == black){
        if(taken[i][j+2] == black && taken[i+1][j+2] == black && taken[i+2][j+2] == black){
            if(checkForGreen(i+2,j))
            taken[i+2][j] = green;
        }
    }
	}
    
}

void shape3_4(int i, int j){
	if(isAllowed(i+1, j) && isAllowed(i+2, j) && isAllowed(i+2, j-1) && isAllowed(i+2, j-2)){
			if(taken[i+1][j] == black){
        if(taken[i+2][j] == black && taken[i+2][j-1] == black && taken[i+2][j-2] == black){
            if(checkForGreen(i,j-2))
            taken[i][j-2] = green;
        }
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
    
    if(isAllowed(i, j+1) && isAllowed(i+1, j) && isAllowed(i+2, j) && isAllowed(i+3, j) && isAllowed(i+4, j) && isAllowed(i+4, j+1)){
			if(taken[i][j+1] == black){
        if(taken[i+1][j] == black && taken[i+2][j] == black && taken[i+3][j] == black && taken[i+4][j]){
            if(taken[i+4][j+1] == black){
                if(checkForGreen(i+2,j+2))
                taken[i+2][j+2] = green;
            }
        }
    }
	}
    
}

void shape4_2(int i, int j){
	if(isAllowed(i, j+1) && isAllowed(i+1, j+1) && isAllowed(i+2, j+1) && isAllowed(i+3, j+1) && isAllowed(i+4, j+1) && isAllowed(i+4, j)){
			if(taken[i][j+1] == black){
        if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j+1] == black && taken[i+4][j+1]){
            if(taken[i+4][j] == black){
                if(checkForGreen(i+2,j-1))
                taken[i+2][j-1] = green;
            }
        }
    }
	}
    
}

void shape4_3(int i, int j){
	if(isAllowed(i+1, j) && isAllowed(i, j+1) && isAllowed(i, j+2) && isAllowed(i, j+3) && isAllowed(i, j+4) && isAllowed(i+1, j+4)){
		if(taken[i+1][j] == black){
			if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i][j+3] == black && taken[i][j+4]){
				if(taken[i+1][j+4] == black){
					if(checkForGreen(i+2,j+2))
					taken[i+2][j+2] = green;
				}
			}
		}
	}
}
void shape4_4(int i, int j){
	if(isAllowed(i+1, j) && isAllowed(i+1, j+1) && isAllowed(i+1, j+2) && isAllowed(i+1, j+3) && isAllowed(i+1, j+4) && isAllowed(i, j+4)){
		if(taken[i+1][j] == black){
			if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i+1][j+3] == black && taken[i+1][j+4]){
				if(taken[i][j+4] == black){
					if(checkForGreen(i-1,j+2))
					taken[i-1][j+2] = green;
				}
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
	if(isAllowed(i+1, j+1) && isAllowed(i+2, j+1) && isAllowed(i+3, j)){
			if(taken[i+1][j+1] == black && taken[i+2][j+1] == black){
				if(taken[i+3][j] == black){
					if(checkForOrange(i-1,j+2))
					taken[i-1][j+2] = orange;
					if(checkForOrange(i+4,j+2))
					taken[i+4][j+2] = orange;
				}
			}

	}
}

void shape5_2(int i, int j){
	if(isAllowed(i+1, j-1) && isAllowed(i+2, j+1) && isAllowed(i+3, j)){
			if(taken[i+1][j-1] == black && taken[i+2][j+1] == black){
				if(taken[i+3][j] == black){
					if(checkForOrange(i-1,j-2))
					taken[i-1][j-2] = orange;
					if(checkForOrange(i+4,j-2))
					taken[i+4][j-2] = orange;
				}
			}

	}
}

void shape5_3(int i, int j){
	if(isAllowed(i+1, j+1) && isAllowed(i+1, j+2) && isAllowed(i, j+3)){
        if(taken[i+1][j+1] == black && taken[i+1][j+2] == black){
            if(taken[i][j+3] == black){
                if(checkForOrange(i+2,j-1))
                taken[i+2][j-1] = orange;
                if(checkForOrange(i+2,j+4))
                taken[i+2][j+4] = orange;
            }
        }
	}
}
void shape5_4(int i, int j){
	if(isAllowed(i+1, j-1) && isAllowed(i, j+1) && isAllowed(i+1, j+2)){
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
}

void shape5(int i, int j){
    shape5_1(i, j);
    shape5_2(i, j);
    shape5_3(i, j);
    shape5_4(i, j);
}



void shape6_1(int i, int j){

if(isAllowed(i+1, j+1) && isAllowed(i+2, j+1) && isAllowed(i+3, j+1) && isAllowed(i+4, j) && isAllowed(i+3, j+1)){
		if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j+1] == black && taken[i+4][j] == black){
			if(taken[i+3][j+1] == black){
				if(checkForOrange(i-1,j+2))
					taken[i-1][j+2] = orange;
				if(checkForOrange(i+5,j+2))
					taken[i+5][j+2] = orange;
			}
		}

	}
}

void shape6_2(int i, int j){
if(isAllowed(i+1, j-1) && isAllowed(i+2, j-1) && isAllowed(i+3, j-1) && isAllowed(i+4, j)){
        if(taken[i+1][j-1] == black && taken[i+2][j-1] == black && taken[i+3][j-1] == black){
            if(taken[i+4][j] == black){
                if(checkForOrange(i-1,j-2))
                    taken[i-1][j-2] = orange;
                if(checkForOrange(i+5,j-2))
                    taken[i+5][j-2] = orange;
            }
        }
	}
}

void shape6_3(int i, int j){
if(isAllowed(i+1, j+1) && isAllowed(i+1, j+2) && isAllowed(i+1, j+3) && isAllowed(i, j+4)){
    if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i+1][j+3] == black){
        if(taken[i][j+4] == black){
            if(checkForOrange(i+2,j-1))
                taken[i+2][j-1] = orange;
            if(checkForOrange(i+2,j+5))
                taken[i+2][j+5] = orange;
        }
    }
}
}

void shape6_4(int i, int j){
	if(isAllowed(i+1, j-1) && isAllowed(i, j+1) && isAllowed(i, j+2) && isAllowed(i+1, j+3)){
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
}

void shape6(int i, int j){
	
    shape6_1(i, j);
    shape6_2(i, j);
    shape6_3(i, j);
    shape6_4(i, j);
    
}


void shape7_1(int i, int j) {
if(isAllowed(i+1, j+1) && isAllowed(i+2, j+2) && isAllowed(i+1, j-1)){
		if (taken[i + 1][j + 1] == black && taken[i + 2][j + 2] == black) {
			if (taken[i + 1][j - 1] == black){
				if (checkForYellow(i, j+3))
					taken[i][j + 3] = yellow;
			}
		}
	}
}

void shape7_2(int i, int j){
if(isAllowed(i+1, j-1) && isAllowed(i+2, j-2) && isAllowed(i+1, j+1)){
		if(taken[i+1][j-1] == black && taken[i+2][j-2] == black) {
			if (taken[i + 1][j + 1] == black) {
				if (checkForYellow(i-1, j-2))
					taken[i - 1][j - 2] = yellow;
			}
		}
	}
}
void shape7_3(int i, int j){
if(isAllowed(i+1, j+1) && isAllowed(i+2, j+2) && isAllowed(i-1, j+1)){
		if(taken[i + 1][j + 1] == black && taken[i + 2][j + 2] == black){
			if(taken[i-1][j+1] == black){
				if(checkForYellow(i+2, j-1))
					taken[i+2][j-1] = yellow;
			}
		}
	}
}

void shape7_4(int i, int j){
   if(isAllowed(i+1, j-1) && isAllowed(i+2, j-2) && isAllowed(i-1, j-1)){
		if(taken[i+1][j-1] == black && taken[i+2][j-2] == black){
			if(taken[i-1][j-1] == black){
					if(checkForYellow(i+3, j))
						taken[i+3][j] = yellow;
				}
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
if(isAllowed(i+1, j-1) && isAllowed(i+2, j-1) && isAllowed(i+2, j-2) && isAllowed(i+3, j-2)){
		if (taken[i + 1][j - 1] == black && taken[i + 2][j - 1] == black && taken[i + 2][j - 2] == black) {
			if (taken[i + 3][j - 2] == black){
				if (checkForYellow(i+2, j+2))
					taken[i+2][j + 2] = yellow;
				if (checkForYellow(i+3, j))
					taken[i+3][j] = yellow;
			}
		}
	}
}

void shape8_2(int i, int j){
if(isAllowed(i, j+1) && isAllowed(i+1, j+1) && isAllowed(i+1, j+2) && isAllowed(i+2, j+3)){
		if(taken[i][j+1] == black && taken[i+1][j+1] == black && taken[i+1][j+2] == black) {
			if (taken[i + 2][j + 3] == black) {
				if (checkForYellow(i+2, j))
					taken[i+2][j] = yellow;
				if (checkForYellow(i+4, j+1))
					taken[i+4][j+1] = yellow;
			}
		}
	}
}

void shape8_3(int i, int j){
if(isAllowed(i+1, j) && isAllowed(i+1, j-1) && isAllowed(i+2, j-1) && isAllowed(i+3, j-2)){
		if(taken[i + 1][j] == black && taken[i + 1][j-1] == black){
			if(taken[i+2][j-1] == black && taken[i + 3][j-2] == black){
				if(checkForYellow(i, j-2))
					taken[i][j-2] = yellow;
				if(checkForYellow(i+1, j-4))
					taken[i+1][j-4] = yellow;
			}
		}
		
	}
}

void shape8_4(int i, int j){
	if(isAllowed(i+1, j) && isAllowed(i+1, j+1) && isAllowed(i+1, j+2) && isAllowed(i+3, j-2)){
		if(taken[i+1][j] == black && taken[i+1][j+1] == black && taken[i+1][j+2] == black){
			if(taken[i+3][j-2] == black){
				if(checkForYellow(i, j+3))
					taken[i][j+3] = yellow;
				if(checkForYellow(i-2, j+2))
					taken[i-2][j+2] = yellow;
			}
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
if(isAllowed(i+1, j-1) && isAllowed(i+2, j-1) && isAllowed(i+2, j-2) && isAllowed(i+3, j+2)){
		if (taken[i + 1][j-1] == black && taken[i + 2][j - 1] == black && taken[i + 2][j-2] == black) {
			if (taken[i + 3][j + 2] == black){
				if (checkForYellow(i, j+2))
					taken[i][j + 2] = yellow;
				if (checkForYellow(i+1, j+4))
					taken[i+1][j+4] = yellow;
			}
		}
	}
}

void shape9_2(int i, int j){
if(isAllowed(i+1, j) && isAllowed(i+1, j-1) && isAllowed(i+1, j-2) && isAllowed(i+2, j-3)){
		if(taken[i+1][j] == black && taken[i+1][j-1] == black && taken[i+1][j-2] == black) {
			if (taken[i + 2][j - 3] == black) {
				if (checkForYellow(i, j-3))
					taken[i][j-3] = yellow;
				if (checkForYellow(i-2, j-2))
					taken[i-2][j-2] = yellow;
			}
		}
	}
}

void shape9_3(int i, int j){

if(isAllowed(i+1, j+1) && isAllowed(i+2, j+1) && isAllowed(i+2, j+2) && isAllowed(i+3, j+2)){
		if(taken[i + 1][j+1] == black && taken[i + 2][j+1] == black){
			if(taken[i+2][j+2] == black && taken[i + 3][j+2] == black){
				if(checkForYellow(i+3, j))
					taken[i+3][j] = yellow;
				if(checkForYellow(i+2, j-2))
					taken[i+2][j-2] = yellow;
			}
		}
	}
}

void shape9_4(int i, int j){
if(isAllowed(i+1, j-1) && isAllowed(i+1, j-2) && isAllowed(i+1, j-3) && isAllowed(i+2, j-3)){
		if(taken[i+1][j-1] == black && taken[i+1][j-2] == black && taken[i+1][j-3] == black){
			if(taken[i+2][j-3] == black){
				if(checkForYellow(i+2, j))
					taken[i+2][j] = yellow;
				if(checkForYellow(i+4, j-1))
					taken[i+4][j-1] = yellow;
			}
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
  if(isAllowed(i+1, j) && isAllowed(i+2, j) && isAllowed(i+3, j) && isAllowed(i+4, j+1)){  
        if(taken[i+1][j] == black && taken[i+2][j] == black && taken[i+3][j] == black){
            if(taken[i+4][j+1] == black){
                if(checkForOrange(i+2,j+1))
                taken[i+2][j+1] = orange;
            }
        }
    }
}

void shape10_2(int i, int j){
   if(isAllowed(i+1, j+1) && isAllowed(i+2, j+1) && isAllowed(i+3, j+1) && isAllowed(i+4, j+1)){
        if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j+1] == black && taken[i+4][j+1]){
            
                if(checkForOrange(i+2,j))
                taken[i+2][j] = orange;
            }
      }
}

void shape10_3(int i, int j){
   if(isAllowed(i+1, j-1) && isAllowed(i, j+1) && isAllowed(i, j+2) && isAllowed(i, j+3)){
		if(taken[i+1][j-1] == black){
			if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i][j+3] == black){
					if(checkForOrange(i+1,j+1))
					taken[i+1][j+1] = orange;
				}
			}
		}
}

void shape10_4(int i, int j){
    if(isAllowed(i+1, j-1) && isAllowed(i+1, j-2) && isAllowed(i+1, j-3) && isAllowed(i+1, j-4)){
    if(taken[i+1][j-1] == black){
        if(taken[i+1][j-2] == black && taken[i+1][j-3] == black && taken[i+1][j-4] == black){
                if(checkForOrange(i,j-2))
                taken[i][j-2] = orange;
            }
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
    //cout<<"Start: does it belong."<<endl;
    shape1(i, j);    
    //cout<<"2: does it belong."<<endl;
    shape2(i, j);
    //cout<<"3: does it belong."<<endl;
    shape3(i, j);
    //cout<<"4: does it belong."<<endl;
    shape4(i, j);
    //cout<<"5: does it belong."<<endl;
    shape5(i, j);
    //cout<<"6: does it belong."<<endl;
    shape6(i, j); 
    //cout<<"7: does it belong."<<endl;
    shape7(i, j);
    //cout<<"8: does it belong."<<endl;
    shape8(i, j);
    //cout<<"9: does it belong."<<endl;
    shape9(i, j);
    //cout<<"10: does it belong."<<endl;
	shape10(i, j);
    //cout<<"End: does it belong."<<endl;

}
void checkNodes(int i, int j){

	int flag = 0;
	//cout<<"neutral"<<endl;
	if(taken[i][j] == green){
		//cout<<"Green"<<endl;
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
		//cout<<"forestGreen"<<endl;
		
	} else if(taken[i][j] == orange){
		//cout<<"Orange"<<endl;
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
				//cout<<"forestGreen"<<endl;
		}
	} else if(taken[i][j] == yellow){
		//cout<<"Yeallow"<<endl;
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
				//cout<<"forestGreen"<<endl;
		}
	}
}

double euclidean(double x1, double y1, double x2, double y2){
	double d = std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
	return d;
}


int getLeft(int i, int j){
    int left = 0;
    
    if(taken[i][j-1]  == black){
		left += 2;
		//cout<<"Left 1 taken"<<endl;
	} else if(taken[i][j-2] == black){
		//cout<<"Left 2 taken"<<endl;
		left  += 1;
	}
	return left;
}

int getRight(int i, int j){
    int right = 0;
    
    if(taken[i][j+1]  == black){
		//cout<<"Right 1 taken"<<endl;
		right += 2;
	}else if(taken[i][j+2] == black){
		//cout<<"Right 2 taken"<<endl;
		right  += 1;
	}
	return right;
}

int getUp(int i, int j){
    
    int up = 0;
    
    if(taken[i-1][j]  == black){
		//cout<<"Up 1 taken"<<endl;
		up += 2;
	}else if(taken[i-2][j] == black){
		//cout<<"Up 2 taken"<<endl;
		up  += 1;
	}
	return up;
}


int getDown(int i, int j){
    int down = 0;
    
    if(taken[i+1][j]  == black){
		//cout<<"Down 1 taken"<<endl;
		down += 2;
	}else if(taken[i+2][j] == black){
		//cout<<"Down 2 taken"<<endl;
		down  += 1;
	}
	
	return down;
}


void checkWall(int &i, int &j){
	
	
	
    int up = getUp(i, j);
    int down = getDown(i, j);
    int left = getLeft(i, j);
    int right = getRight(i, j);
	int total = 0;

	total = left + right + up + down;
	
	if(up > down){
		//cout<<"Moving down: "<<up<<" -- "<<down<<endl;
		int dif = up - down;
		if(dif != down){
			//check out how it would be then
			int leftT = getLeft(i+dif, j);
			int rightT = getRight(i+dif, j);
			int downT = getDown(i+dif, j);
			int upT = getUp(i+dif, j);
			int totalT = leftT + rightT + upT + downT;
			
			if(totalT <= total){
			    if(upT + downT <= up + down){
			        if(leftT + rightT <= left + right){
			            //here we move// change the place of the node
			            i += dif;
			        }
			    }
			}
			
		}
		
	} else if(down > up){
		//cout<<"Moving up: "<<up<<" -- "<<down<<endl;
	    int dif = down - up;
	    if(dif != up){
	        int leftT = getLeft(i-dif, j);
			int rightT = getRight(i-dif, j);
			int downT = getDown(i-dif, j);
			int upT = getUp(i-dif, j);
			int totalT = leftT + rightT + upT + downT;
			
			if(totalT <= total){
			    if(leftT + rightT <= left + right){
			         //here we move// change the place of the node
			         i -= dif;
			    }
			}
	        
	    }
	}
	
	
	up = getUp(i, j);
    down = getDown(i, j);
    left = getLeft(i, j);
    right = getRight(i, j);
	total = 0;

	total = left + right + up + down;
	
	//check for left and right again
	
	if(left > right){
		//cout<<"Moving to the right: "<<left<<" -- "<<right<<endl;
		int dif = left - right;
		if(dif != right){
			int leftT = getLeft(i, j+dif);
			int rightT = getRight(i, j+dif);
			int downT = getDown(i, j+dif);
			int upT = getUp(i, j+dif);
			int totalT = leftT + rightT + upT + downT;
			
			if(totalT <= total){
				if(upT + downT <= up + down){
					//here we move// change the place of the node
					j += dif;
				}
			}
		}
	} else if(right > left){
		//cout<<"Moving to the left: "<<left<<" -- "<<right<<endl;
		int dif = right - left;
		if(dif != left){
			int leftT = getLeft(i, j-dif);
			int rightT = getRight(i, j-dif);
			int downT = getDown(i, j-dif);
			int upT = getUp(i, j-dif);
			int totalT = leftT + rightT + upT + downT;
			
			if(totalT <= total){
				if(upT + downT <= up + down){
					//here we move// change the place of the node
					j -= dif;
				}
			}
		}
	}
	/*
	int * ij = new int[2];
	ij[0] = i;
	ij[1] = j;
	return ij;
	*/
}

void check(){

//cout<<"Before does it be	long"<<endl;
//cout<<"Width: "<<width<<endl;
//cout<<"Length: "<<length<<endl;

    for(int i=0; i<width; i++){
        for(int j=0; j<length; j++){
           //cout<<"["<<i<<"]["<<j<<"]: "<<taken[i][j]<<endl;
            if(taken[i][j] == black){
                doesItBelong(i, j);
            }
        }
       // cout<<endl;
    }
  /*
    int flag = 0;
    for(int i = 0;  i< width; i++){
		flag = 0;
		for(int j = 0; j<length; j++){
			//cout<<taken[i][j]<<" ";
			if(taken[i][j] != gray){
				flag = 1;
			}
		}
		if(flag){
			for(int j=0; j<length; j++){
				if(taken[i][j] == black){
					cout<<taken[i][j]<<" ";
				} else if( taken[i][j] != white && taken[i][j] != gray){
					cout<<taken[i][j]<<" ";
				}else {
					cout<<"_ ";
				}
			}
			cout<<" "<<endl;
			cout<<" "<<endl;
		}
		
	}
	* */
    

//cout<<"After does it belong"<<endl;
//cout<<"width:"<<width<<endl;
//cout<<"length:"<<length<<endl;
    for(int i=0; i<width; i++){
        for(int j=0; j<length; j++){
			//cout<<"["<<i<<"]["<<j<<"]"<<taken[i][j]<<endl;
            if(taken[i][j] != black && taken[i][j] != white && taken[i][j] != gray){
                //cout<<"["<<i<<"]["<<j<<"]: "<<taken[i][j]<<endl;
                checkNodes(i, j);
                //cout<<"checking nodes"<<endl;
            }
        }
    }
    
    //cout<<"after check nodes"<<endl;

xes = new int[numNodes];
ys = new int[numNodes];

int *xtemp;
int *ytemp;

int z = 0;

    for(int i=0; i<width; i++){
    	for(int j=0; j<length; j++){
			//ROS_INFO("Xes and Ys");
    		if(taken[i][j] == forestGreen){
				//cout<<"Checking point: ["<<i<<"] ["<<j<<"]"<<endl;
    			int a = i;
    			int b = j;
    			
    			checkWall(a, b);
    			double x = a * rWidth + rWidth/2;
    			double y = b * rLength + rLength/2;
    			///int * ij = new int[2];
    			
    			//x = ij[0];
    			//y = ij[1];
    			//cout<<"Before adding: ["<<a<<"] ["<<b<<"]"<<endl;
    			//cout<<"Before Euclidean: ["<<i<<"] ["<<j<<"]"<<"-> ["<<x<<"] ["<<y<<"]"<<endl;
    			if(z == 0){
					
					xes[z] = x;
					ys[z] = y;
					//cout<<"Adding point: ["<<xes[z]<<"] ["<<ys[z]<<"]"<<endl;
					z++;
					
				} else {
					
					//ROS_INFO("Euclidean: %.2f", euclidean(x, y, xes[z-1], ys[z-1]));
					int isFar = 0;
					for(int k=0;k<z;k++){
						if(euclidean(x, y, xes[k], ys[k]) <= breakingPoint){
							isFar = 1;
						}
					}
					
					
					if(!isFar){
						//cout<<"Euclidean: "<<euclidean(x, y, xes[z-1], ys[z-1])<<endl;
						
						//ROS_INFO("Euclidean: %.2f", euclidean(x, y, xes[z-1], ys[z-1]));
						//ROS_INFO("Adding point: [%d, %d]", xes[z], ys[z]);
						xes[z] = x;
						ys[z] = y;
						
						z++;
					}
				}
				
    			
    		numNodes = z;	
    			//cout<<"Hooray!"<<endl;
    			//ROS_INFO("Adding point: [%d, %d]", xes[z], ys[z]);
    		}
    	}
    }
    xtemp = new int[numNodes];
	ytemp = new int[numNodes];
	xAr = new int[numNodes];
	yAr = new int[numNodes];
	
    
    for(int i =0; i<numNodes; i++){
		
		xtemp[i] = xes[i];
		ytemp[i] = ys[i];
	}
	cout<<"Num nodes: "<<numNodes<<endl;
	
	
	for(int i=0; i<numNodes; i++){
		xes[i] = xtemp[numNodes-i-1];
		ys[i] = ytemp[numNodes-i-1];
		//cout<<"Adding point: ["<<xes[i]<<"] ["<<ys[i]<<"] <-------------------"<<endl;
		//cout<<"Adding point: ["<<xes[i]<<"] ["<<ys[i]<<"] <-------------------"<<endl;
	}
	//cout<<"---------------------------------------------------------------------"<<endl;
    //cout<<"Adding point: ["<<xes[0]<<"] ["<<ys[0]<<"] <-------------------"<<endl;
    xAr[0] = xes[0];
    yAr[0] = ys[0];
    //cout<<"Adding point: ["<<xes[0]<<"] ["<<ys[0]<<"] <-------------------"<<endl;
    //cout<<"Adding point: ["<<xAr[0]<<"] ["<<yAr[0]<<"] <-------------------"<<endl;
    float min = 10000;
    int counter = 0;
    int xMin = 1;
    int yMin = 1;
    //cout<<"Adding point: ["<<xes[1]<<"] ["<<ys[1]<<"] <-------------------"<<endl;
    
    for(int k=0; k<numNodes-1; k++){
		//cout<<"Checking point: ["<<xAr[k]<<"] ["<<yAr[k]<<"]"<<endl;
		for(int u=1; u<numNodes; u++){
			//cout<<"*************************"<<endl;
			//cout<<"Xes and Ys: ["<<xMin<<"] ["<<yMin<<"]"<<endl;
			//cout<<"--------: ["<<xes[xMin]<<"] ["<<ys[yMin]<<"]"<<endl;
			if(xes[u] != -1 || ys[u] != -1){
				//cout<<"mini mini me: ["<<xMin<<"] ["<<yMin<<"] <-------------------"<<endl;
				float euc = euclidean(xAr[k], yAr[k], xes[u], ys[u]);
				//cout<<"Euclid: "<<euc<<endl;
				//cout<<"Mini Mouse: "<<min<<endl;
				if(euc < min){
					//cout<<"Xes and Ys: ["<<xMin<<"] ["<<yMin<<"]"<<endl;
					//cout<<"++++++++++: ["<<xes[xMin]<<"] ["<<ys[yMin]<<"]"<<endl;
					
					min = euc;
					xMin = u;
					yMin = u;
					//cout<<"Xes and Ys: ["<<xMin<<"] ["<<yMin<<"]"<<endl;
					//cout<<"++++++++++: ["<<xes[xMin]<<"] ["<<ys[yMin]<<"]"<<endl;
					//cout<<"#################### "<<endl;
						
				}
			}
		}
					
		//cout<<"---------------------------------------------------------------------"<<endl;
				counter++;
				xAr[k+1] = xes[xMin];
				yAr[k+1] = ys[yMin];
				//cout<<"Adding point: ["<<xAr[k+1]<<"] ["<<yAr[k+1]<<"] <-------------------"<<endl;
				//cout<<"Adding point: ["<<k+1<<"] ["<<k+1<<"] <-------------------"<<endl;
				xes[xMin] = -1;
				ys[yMin] = -1;
				min = 10000;
			
			
		//cout<<"---------------------------------------------------------------------"<<endl;
		
	}
    //cout<<"Count: "<<counter<<endl;
    //rearange the positions
    for(int i=0; i<numNodes; i++){
		cout<<"Adding point: ["<<xAr[i]<<"] ["<<yAr[i]<<"] <-------------------"<<endl;
	}
    
//cout<<"after xes and ys"<<endl;
}

Mat cv_map;

void createGrid(){
 //int * occup = msg->data.c_str(); 

	//cout<<"Create grid"<<endl;
	/*Mat image;
	image = imread("comp2_map_2.pgm", CV_LOAD_IMAGE_COLOR);
	if(! image.data){
		cout<<"Couldn't open image." << std::endl;
	}

cout<<"Loaded map"<<endl;
*/
    //int ** pixels = new int*[baseWidth];
    Mat pixels = cv_map;
    
    
 //   namedWindow("Somethin", CV_WINDOW_AUTOSIZE);
    //cvtColor(image, pixels, CV_BGR2GRAY);


	baseWidth = 512;
	baseLength = 480;

//int pixels[baseWidth][baseLength] = cv_map_data;

    width = baseWidth/rWidth;
    length = baseLength/rLength;

    //int * m = new int[width/rWidth];
    //int * n = new int[width/rWidth];
    //int counter = 0;
    //cout<<"Initialising taken"<<endl;
    taken = new int*[width];
    
    for(int i = 0; i < width; i++) {
		taken[i] = new int[length];
		
    }


//cout<<"Before taken"<<endl;
    for(int i=0; i<baseWidth; i+=rWidth){
        for(int j=0; j<baseLength; j+=rLength){
            int flag = white;
			//cout<<"Taken flag: ["<<i<<"]["<<j<<"]: "<<endl;
            for(int w=i; w<i + rWidth; 	w++){

                for(int l=j; l<j + rLength; l++){
					//cout<<"Taken val: ["<<l<<"]["<<w<<"]: "<<endl;
                    int val = (int)pixels.at<uchar>(l, w);
                    //cout<<val<<" ";
                    if(flag != black){
						if(val != white){
						flag = val;
					}
					}
                    
                }
                //cout<<endl;
            }
				taken[i/rWidth][j/rLength] = flag;
        }
    }
/*  
   int flag = 0;
    for(int i = 0;  i< width; i++){
		flag = 0;
		for(int j = 0; j<length; j++){
			//cout<<taken[i][j]<<" ";
			if(taken[i][j] != gray){
				flag = 1;
			}
		}
		if(flag){
			for(int j=0; j<length; j++){
				if(taken[i][j] == black){
					cout<<taken[i][j]<<" ";
				} else {
					cout<<"_ ";
				}
			}
			cout<<" "<<endl;
			cout<<" "<<endl;
		}
		
	}
	
	*/
	
//cout<<"Filed taken, check is next."<<endl;
    check();

    for(int i=0; i<numNodes; i++){
		//cout<<"Pixels: "<< xes[i] <<" "<< ys[i]<< endl;
	}

    //ROS_INFO(taken);
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Subscriber map_sub;
//ros::Subscriber ring_sub;
//Mat cv_map;
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
	ros::Subscriber move_sub3;
	
public:
	bool moving;
	GoalSender(ros::NodeHandle &nh): ac("move_base", true), n(nh)
	{		
		ROS_INFO("Start initializing approach");
		
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
		moving = false;
		move_sub = n.subscribe<geometry_msgs::PoseStamped>("new_ring", 100, &GoalSender::stop_goals, this);
		move_sub3 = n.subscribe<geometry_msgs::PoseStamped>("new_face", 100, &GoalSender::stop_goals, this);
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
};

/*
int main(){
	createGrid();
	cout<<"Size: "<<numNodes<<endl;
}*/



int main(int argc, char** argv) {

    ros::init(argc, argv, "pic_goals");
    ros::NodeHandle n;
    GoalSender sender(n);

    map_sub = n.subscribe("map", 10, &mapCallback);

int z = 5;
    ROS_INFO("Start spinning");
    while(ros::ok()) {
		if(map_avaiable){createGrid(); map_avaiable = false; sender.moving = true;}
		if(sender.moving)
		{
			sender.pxl_goal(xAr[z], yAr[z], 0);
			z++;
			if(z == numNodes) z = 5;
			sender.turn();
			//break;
		}
        ros::spinOnce();
    }
    return 0;

}
