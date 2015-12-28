#include "Robot.h"
#include<iostream>
using namespace std;
Robot::Robot(double l1,double l2)
{	L1=l1;
	L2=l2;
	Point O_Joint1,O_Joint2(l1);
	JointFrame1=Frame(O_Joint1);
	JointFrame2=Frame(O_Joint2);
}
void Robot::PTPMove(Frame Input_Frame,Point Input_point)
{	AimPoint_WorldFrame=Input_Frame.InputPoint_WorldFrame(Input_point);
	solver.Neg_calculation(L1,L2,AimPoint_WorldFrame);
	if (solver.number==0)
		cout<<"该点超出活动范围"<<endl;
	else if(solver.number==1) 
			cout<<"机械手两关节转角"<<'('<<solver.Angle[0].x<<','<<solver.Angle[0].y<<')'<<endl;
		 else 
			if(solver.number==2){
			cout<<"机械手两关节第一组转角"<<'('<<solver.Angle[0].x<<','<<solver.Angle[0].y<<')'<<endl;
			cout<<"机械手两关节第二组转角"<<'('<<solver.Angle[1].x<<','<<solver.Angle[1].y<<')'<<endl;
		  	}
	solver.number=0;
}
void Robot::PTPMove(Point Input_Angle)
{	solver.Pos_calculation(L1,L2,Input_Angle);
	cout<<"关节3在世界坐标系中的坐标"<<'('<<solver.point_World.x<<','<<solver.point_World.y<<')'<<endl;
}
