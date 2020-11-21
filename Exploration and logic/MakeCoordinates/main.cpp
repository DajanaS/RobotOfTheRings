#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
std::ifstream infile("reachable_goals.txt");

using namespace std;

class Point
{
    double x;
    double y;
    double z;
    double w;

public:
    Point(double x, double y, double z, double w)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }

};

Point** createGoals(Point **arr)
{
    return arr;
}

int main()
{
    std::string strLine;
    std::string line;
    int lineCounter = 1;
    Point **coordinates_array = new Point*[100000];
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
            coordinates_array[i] = new Point(x, y, z, w);
            cout<<"x: "<<x<<" y: "<<y<<" z: "<<z<<" w: "<<w<<endl;
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

    createGoals(coordinates_array);

    return 0;
}
