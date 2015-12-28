#include<iostream>
#include"Robot.h"
using namespace std;

void Robot::PTPmove( WorldFrame f,Point p){
	Point WorldframePoint=solver.FrameToWF(f,p);
	Point AngleMatrix=solver.NegativeKinematics(L1,L2,WorldframePoint);
	Point PKResult=solver.PositiveKinematics(L1,L2,AngleMatrix);
	Point WorldframePoint1(0,30);
	Point AngleMatrix1=solver.NegativeKinematics(L1,L2,WorldframePoint1);
	InsightPoint.updatePoint(PKResult.x,PKResult.y);
	cout<<"机械手两关节转角"<<"<"<<AngleMatrix1.x<<","<<AngleMatrix1.y<<">"<<endl;
	cout<<"点的位置为"<<"<"<<InsightPoint.x<<"," <<InsightPoint.y<<">"<<endl;
	
}

void Robot::PTPmove(JointFrame f,Point p){
	Point WorldframePoint=solver.FrameToWF(f,p);
	Point AngleMatrix=solver.NegativeKinematics(L1,L2,WorldframePoint);
	Point PKResult=solver.PositiveKinematics(L1,L2,AngleMatrix);
	InsightPoint.updatePoint(PKResult.x,PKResult.y);
	cout<<"机械手两关节转角"<<"<"<<AngleMatrix.x<<","<<AngleMatrix.y<<">"<<endl;
	cout<<"点的位置为"<<"<"<<InsightPoint.x<<"," <<InsightPoint.y<<">"<<endl;
}

void Robot::PTPmove(TaskFrame f,Point p){
	Point WorldframePoint=solver.FrameToWF(f,p);
	Point AngleMatrix=solver.NegativeKinematics(L1,L2,WorldframePoint);
	Point PKResult=solver.PositiveKinematics(L1,L2,AngleMatrix);
	InsightPoint.updatePoint(PKResult.x,PKResult.y);
	cout<<"机械手两关节转角"<<"<"<<AngleMatrix.x<<","<<AngleMatrix.y<<">"<<endl;
	cout<<"点的位置为"<<"<"<<PKResult.x<<"," <<PKResult.y<<">"<<endl;
	
}

void Robot::addTFrame(TaskFrame frame){
	TaskFrameVec.push_back(frame);
}

void Robot::deleteFrame(){
	TaskFrameVec.pop_back();
}
