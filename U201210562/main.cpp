#include <iostream>
#include "robot.hpp"
#include "Frame.hpp"


int main(int argc, char** argv) {
	
	Robot myRobot(5,7);   
	Vector2d v1(0,0);
	Vector2d v2(1,1);
	Vector2d v3(5,7);
	Vector2d v4(5,9);
	
	Vector2d p1(2,2);
	Vector2d p2(2,2);
	Vector2d p3(-1,3);
	Vector2d p4(-10,-8);	
	
     
    Point origin(v1);
    Point origin1(v2);
    Point origin2(v3);
    Point origin3(v4);   
     
    Point point1(p1);
    Point point2(p2);
    Point point3(p3);
    Point point4(p4);   
     
    Frame WF(origin,0);
    Frame TF1(origin1,30);
    Frame TF2(origin2,60);
    Frame TF3(origin3,90);  
     
     
    myRobot.PTPmove(WF,point1);
    myRobot.PTPmove(TF1,point2);
    myRobot.PTPmove(TF2,point3);
    myRobot.PTPmove(TF3,point4);
     
     
    return 0;
}
