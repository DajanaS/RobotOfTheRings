#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

int * taken;

int width = 544;
int length = 576;

int unknown = -1;
int black = 100;
int white = 0;
int yellow = 20;

void createGrid(const std_msgs::String::ConstPtr& msg){
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    int * occup = msg->data.c_str(); //[an array] The map data, in row-major order, starting with (0,0).
    //http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html


    int pixels[width][length];

    int rWidth = 4;
    int rLength = 4;

    //int * m = new int[width/rWidth];
    //int * n = new int[width/rWidth];
    //int counter = 0;
    taken = (width/rWidth) + 1;

    for(int i=0; i<width; i++){
        for(int j=0; j<length; j++){
            pixels[i][j] = occup[i+j];
        }
    }


    for(int i=0; i<width; i+=rWidth){
        for(int j=0; j<length; j+=rLength){
            int flag = 0;

            for(int w=i; w<i + rWidth; 	w++){

                for(int l=j; l<j + rLength; l++){
                    if(pixels[w][l] != 0){
                        flag = pixels[w][l];
                    }
                }
            }

            taken[i/rWidth][j/rLength] = flag;
            ROS_INFO(flag);
        }
        ROS_INFO("\n");
    }

    //ROS_INFO(taken);
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
}

void shape1(int i, int j){
    shape1_1(i, j);
    shape1_2(i, j);
    shape1_3(i, j);
    shape1_4(i, j);
}

void shape1_1(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j] == black && taken[i+2][j] == black && taken[i+3][j] == black){
            if(taken[i+3][j+1] == black){
                if(taken[i+1][j+2] == white)
                taken[i+1][j+2] = yellow;
            }
        }
    }
}

void shape1_2(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j] == black){
            if(taken[i+3][j+1] == black){
                if(taken[i+1][j-1] == white)
                taken[i+1][j-1] = yellow;
            }
        }
    }
}

void shape1_3(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i][j+3] == black){
            if(taken[i+1][j+3] == black){
                if(taken[i+2][j+2] == white)
                taken[i+2][j+2] = yellow;
            }
        }
    }
}

void shape1_4(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i+1][j+3] == black){
            if(taken[i][j+3] == black){
                if(taken[i-1][j+1] == white)
                taken[i-1][j+1] = yellow;
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

void shape2_1(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j] == black && taken[i+2][j] == black && taken[i+2][j+1] == black){
            if(taken[i+1][j+2] == white)
            taken[i+1][j+2] = yellow;
        }
    }
}

void shape2_2(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j+1] == black && taken[i+2][j] == black && taken[i+2][j+1] == black){
            if(taken[i+1][j-1] == white)
            taken[i+1][j-1] = yellow;
        }
    }
}

void shape2_3(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i][j+2] == black){
           if( taken[i-1][j+1] == white)
            taken[i-1][j+1] = yellow;
        }
    }
}

void shape2_4(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i+1][j+2] == black){
            if(taken[i+2][j+1] == white)
            taken[i+2][j+1] = yellow;
        }
    }
}

void shape3(int i, int j){
    shape3_1(i, j);
    shape3_2(i, j);
    shape3_3(i, j);
    shape3_4(i, j);
}

void shape3_1(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+2][j] == black && taken[i+2][j+1] == black && taken[i+2][j+2] == black){
            if(taken[i][j+2] == white)
            taken[i][j+2] = yellow;
        }
    }
}

void shape3_2(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+2][j] == black && taken[i][j+1] == black && taken[i][j+2] == black){
           if(taken[i+2][j+2] == white)
            taken[i+2][j+2] = yellow;
        }
    }
}

void shape3_3(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i][j+2] == black && taken[i+1][j+2] == black && taken[i+2][j+2] == black){
            if(taken[i+2][j] == white)
            taken[i+2][j] = yellow;
        }
    }
}

void shape3_4(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+2][j] == black && taken[i+2][j-1] == black && taken[i+2][j-2] == black){
            if(taken[i][j-2] == white)
            taken[i][j-2] = yellow;
        }
    }
}

void shape4(int i, int j){
    shape4_1(i, j);
    shape4_2(i, j);
    shape4_3(i, j);
    shape4_4(i, j);
}

void shape4_1(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j] == black && taken[i+2][j] == black && taken[i+3][j] == black && taken[i+4][j]){
            if(taken[i+4][j+1] == black){
                if(taken[i+2][j+2] == white)
                taken[i+2][j+2] = yellow;
            }
        }
    }
}

void shape4_2(int i, int j){
    if(taken[i][j+1] == black){
        if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j] == black && taken[i+4][j+1]){
            if(taken[i+4][j] == black){
                if(taken[i+2][j-1] == white)
                taken[i+2][j-1] = yellow;
            }
        }
    }
}

void shape4_3(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black && taken[i][j+3] == black && taken[i][j+4]){
            if(taken[i+1][j+4] == black){
                if(taken[i+2][j+2] == white)
                taken[i+2][j+2] = yellow;
            }
        }
    }
}

void shape4_4(int i, int j){
    if(taken[i+1][j] == black){
        if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i+1][j+3] == black && taken[i+1][j+4]){
            if(taken[i][j+4] == black){
                if(taken[i-1][j+2] == white)
                taken[i-1][j+2] = yellow;
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

void shape5_1(int i, int j){

        if(taken[i+1][j+1] == black && taken[i+2][j+1] == black){
            if(taken[i+3][j] == black){
                if(taken[i-1][j+2] == white)
                taken[i-1][j+2] = yellow;
                if(taken[i+4][j+2] == white)
                taken[i+4][j+2] = yellow;
            }
        }

}

void shape5_2(int i, int j){

        if(taken[i+1][j-1] == black && taken[i+2][j+1] == black){
            if(taken[i+3][j] == black){
                if(taken[i-1][j-2] == white)
                taken[i-1][j-2] = yellow;
                if(taken[i+4][j-2] == white)
                taken[i+4][j-2] = yellow;
            }
        }

}

void shape5_3(int i, int j){

        if(taken[i+1][j+1] == black && taken[i+1][j+2] == black){
            if(taken[i][j+3] == black){
                if(taken[i+2][j-1] == white)
                taken[i+2][j-1] = yellow;
                if(taken[i+2][j+4] == white)
                taken[i+2][j+4] = yellow;
            }
        }
}

void shape5_4(int i, int j){
    if(taken[i+1][j-1] == black){
        if(taken[i][j+1] == black){
            if(taken[i+1][j+2] == black){
                if(taken[i-1][j-2] == white)
                taken[i-1][j-2] = yellow;
                if(taken[i-1][j+3] == white)
                taken[i-1][j+3] = yellow;
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

void shape6_1(int i, int j){

    if(taken[i+1][j+1] == black && taken[i+2][j+1] == black && taken[i+3][j+1] == black && taken[i+4][j] == black){
        if(taken[i+3][j+1] == black){
            if(taken[i-1][j+2] == white)
                taken[i-1][j+2] = yellow;
            if(taken[i+5][j+2] == white)
                taken[i+5][j+2] = yellow;
        }
    }

}

void shape6_2(int i, int j){

        if(taken[i+1][j-1] == black && taken[i+2][j-1] == black && taken[i+3][j-1] == black){
            if(taken[i+4][j] == black){
                if(taken[i-1][j-2] == white)
                    taken[i-1][j-2] = yellow;
                if(taken[i+5][j-2] == white)
                    taken[i+5][j-2] = yellow;
            }
        }

}

void shape6_3(int i, int j){

    if(taken[i+1][j+1] == black && taken[i+1][j+2] == black && taken[i+1][j+3] == black){
        if(taken[i][j+4] == black){
            if(taken[i+2][j-1] == white)
                taken[i+2][j-1] = yellow;
            if(taken[i+2][j+5] == white)
                taken[i+2][j+5] = yellow;
        }
    }
}

void shape6_4(int i, int j){
    if(taken[i+1][j-1] == black){
        if(taken[i][j+1] == black && taken[i][j+2] == black){
            if(taken[i+1][j+3] == black){
                if(taken[i-1][j-2] == white)
                    taken[i-1][j-2] = yellow;
                if(taken[i-1][j+4] == white)
                    taken[i-1][j+4] = yellow;
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

void shape8(int i, int j){
    shape8_1(i, j);
    shape8_2(i, j);
    shape8_3(i, j);
    shape8_4(i, j);
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

void shape9(int i, int j){
    shape9_1(i, j);
    shape9_2(i, j);
    shape9_3(i, j);
    shape9_4(i, j);
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

int check(){

    for(int i=0; i<width; i++){
        for(int j=0; j<length; j++){
            if(taken[i][j] == black){
                doesItBelong(i, j);
            }
        }
    }
    
    //go through all the yellow ones and group them together, than produce the coordinates, than you're done
}


int main() {

    ros::init(argc, argv, "createGrid");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("nav_msgs/OccupancyGrid", 1000, chatterCallback);
    ros::spin();
    return 0;
}