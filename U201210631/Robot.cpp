#include<iostream>
#include"Robot.h"
#include"Solver.h"
using namespace std;
Robot::Robot(double a1,double a2,double d1min,double d2min,double d1max,double d2max){
	arm1=a1;
	arm2=a2;
	deg1min=d1min;
	deg2min=d2min;
	deg1max=d1max;
	deg2max=d2max;
}
void Robot::SetRobot(double a1,double a2,double d1min,double d2min,double d1max,double d2max){
	arm1=a1;
	arm2=a2;
	deg1min=d1min;
	deg2min=d2min;
	deg1max=d1max;
	deg2max=d2max;
}
void Robot::PTPMove(WorldFrame wf,TaskFrame tf,Point p){
	Solver s;
	s.WtoT(tf,p);
	s.TtoJ(p,arm1,arm2);
	print(p);
}
void Robot::PTPMove(TaskFrame tf,Point p){
	Solver s;
	s.TtoJ(p,arm1,arm2);
	print(p);
}
void Robot::PTPMove(JointFrame jf,Point p){
	print(p);
}
void Robot::print(Point &p){
	if(p.x>=deg1min||p.y<=deg1max){
		cout<<"关节坐标为：("<<p.x<<','<<p.y<<')'<<endl;
	}
	else cout<<"无法旋转到该位置"<<endl; 
}
