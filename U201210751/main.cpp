#include<iostream>
#include"Robot.h"
using namespace std;
int main()
{	                                 		//输入的角度均为XX度，范围-180~180 ，超过自动转换，反解时解得范围-180~180 
	Point a(5,0),b(15,0),c(25,0);
	Point P1(0,10),P2(15,20/sqrt(3)),P3(15,0),P4(15,15);
	Robot myRobot(10,20);
	Frame WF; 
	Frame TF1(a);
	Frame TF2(b);
	Frame TF3(c);
	cout<<"PTPMove(WF,P1)得到的关节角"<<endl;
	myRobot.PTPMove(WF,P1);
	cout<<"PTPMove(TF1,P2)得到的关节角"<<endl;
	myRobot.PTPMove(TF1,P2);			//此点在世界坐标系中的坐标如果为原点，可能会出现错误，例如如果L1=L2，那么关节角1,2分别为Angle，180-Angle，这样存在无数解 
	cout<<"PTPMove(TF2,P3)得到的关节角"<<endl;
	myRobot.PTPMove(TF2,P3);
	cout<<"PTPMove(TF3,P4)得到的关节角"<<endl;
	myRobot.PTPMove(TF3,P4);
	return 0;	
}
