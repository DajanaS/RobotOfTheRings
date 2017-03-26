#include <iostream>
#include <cmath>

using namespace std;

class Point
{
    double x;
    double y;
public:
    Point(double x, double y)
    {
        this->x = x;
        this->y = y;
    }
    double getX()
    {
        return x;
    }
    double getY()
    {
        return y;
    }
};

Point wantedPosition(Point p1, Point p2)   // (x1,y1) -> Robot's position, (x2, y2) -> Face coordinates
{
    double x1 = p1.getX();
    double y1 = p1.getY();
    double x2 = p2.getX();
    double y2 = p2.getY();
    double x3,y3;

    int d = 10;
    double k = (y2-y1)/(x2-x1);
    double c = y1 - k*x1;
    double a = pow(k,2) + 1;
    double b = (-2*x2) + (-2*k*(k*x1 - y1 + y2));
    c = pow(x2,2) + pow((pow(k,x1) - y1 + y2),2) - pow(d,2);

    double x31 = (-b + sqrt((pow(b,2)) - 4*a*c ))/(2*a);
    double x32 = (-b - sqrt((pow(b,2)) - 4*a*c ))/(2*a);
    double y31 = k*(x31 - x1) + y1;
    double y32 = k*(x32 - x1) + y1;

    double d1 = sqrt(pow((x31 - x1),2) + pow((y31 - y1),2));
    double d2 = sqrt(pow((x32 - x1),2) + pow((y32 - y1),2));

    if(d1<d2)
    {
        x3 = x31;
        y3 = y31;
    }
    if(d2<d1)
    {
        x3 = x32;
        y3 = y32;
    }

    Point p(x3,y3);

    return p;
}

int main()
{
    Point p1(100,300);
    Point p2(500,800);
    Point p = wantedPosition(p1,p2);
    cout << p.getX() <<" "<< p.getY() << endl;
    return 0;
}
