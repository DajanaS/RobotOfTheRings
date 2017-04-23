#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

#ifndef PI
#define PI 3.14159265359
#endif

using namespace std;

int createGoals(double goals[100][4],std::string path)
{
	std::ifstream infile(path.c_str());
    std::string strLine;
    std::string line;
    int lineCounter = 1;
    double x = 0, y = 0, z = 0, w = 0;
    int i = 0;

    //Read File Line By Line
    while (infile)
    {
        std::getline(infile, strLine);
        std::istringstream iss(strLine);
        if (lineCounter == 17)
        {
            lineCounter = 0;
            goals[i][0] = x;
            goals[i][1] = y;
            goals[i][2] = z;
            goals[i][3] = w;
         //   ROS_INFO("%d CORDINATES %f %f %f %f",i, goals[i][0], goals[i][1], goals[i][2], goals[i][3]);
            i++;
        }

        if (lineCounter == 9 || lineCounter == 10 || lineCounter == 15 || lineCounter == 16)
        {
            switch (lineCounter)
            {
            case 9:
                if (!(iss >> line >> x)) { break; }
                break;
            case 10:
                if (!(iss >> line >> y)) { break; }
                break;
            case 15:
                if (!(iss >> line >> z)) { break; }
                break;
            case 16:
                if (!(iss >> line >> w)) { break; }
                break;
            }
        }
        lineCounter++;
    }
	infile.close();
    return i;
}

int createGoals_short(double goals[100][4],std::string path)
{
	std::ifstream infile(path.c_str());
    std::string strLine;
    std::string line;
    int lineCounter = 1;
    double x = 0, y = 0, z = 0, w = 0;
    int i = 0;

    //Read File Line By Line
    while (infile)
    {
        std::getline(infile, strLine);
        std::istringstream iss(strLine);
        if (lineCounter == 11)
        {
            lineCounter = 0;
            goals[i][0] = x;
            goals[i][1] = y;
            goals[i][2] = sin(-PI/2);
            goals[i][3] = cos(-PI/2);
         //   ROS_INFO("%d CORDINATES %f %f %f %f",i, goals[i][0], goals[i][1], goals[i][2], goals[i][3]);
            i++;
        }

        if (lineCounter == 8 || lineCounter == 9)// || lineCounter == 15 || lineCounter == 16)
        {
            switch (lineCounter)
            {
            case 8:
                if (!(iss >> line >> x)) { break; }
                break;
            case 9:
                if (!(iss >> line >> y)) { break; }
                break;
         /*   case 15:
                if (!(iss >> line >> z)) { break; }
                break;
            case 16:
                if (!(iss >> line >> w)) { break; }
                break;*/
            }
        }
        lineCounter++;
    }
	infile.close();
    return i;
}
