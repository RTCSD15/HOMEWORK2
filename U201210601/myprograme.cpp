#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
using Eigen::MatrixXd;
using namespace std;
float pi=3.1415926;
class jointframe{
private:
	double deg1;
	double deg2;
public:
	jointframe();
	jointframe(double deg3,double deg4);
	double getdeg1();
	double getdeg2();
};
jointframe::jointframe(){
	deg1=deg2=0;
}
jointframe::jointframe(double deg3,double deg4){
	deg1=deg3;
	deg2=deg4;
}
double jointframe::getdeg1(){
	return deg1;
}
double jointframe::getdeg2(){
	return deg2;
}

class worldframe{
private:
	double wfx;
	double wfy;
	double wfdeg;
public:
	worldframe();
	worldframe(double wfx1,double wfy1,double wfdeg1);
	MatrixXd getWF();
	double getX();
	double getY();
};
worldframe::worldframe(){
	wfx=wfy=0;
	wfdeg=0*pi/180;
}
worldframe::worldframe(double wfx1,double wfy1,double wfdeg1){
	wfx=wfx1;
	wfy=wfy1;
	wfdeg=wfdeg1*pi/180;
}
MatrixXd worldframe::getWF(){
	MatrixXd WF(3,3);
	WF(0,0)=cos(-wfdeg);
	WF(0,1)=sin(-wfdeg);
	WF(0,2)=-wfx;
	WF(1,0)=-sin(-wfdeg);
	WF(1,1)=cos(-wfdeg);
	WF(1,2)=-wfy;
	WF(2.0)=WF(2,1)=0;
	WF(2,2)=1;
	return WF;
}
double worldframe::getX(){
	return wfx;
}
double worldframe::getY(){
	return wfy;
}

class taskframe{
private:
	double tfx;
	double tfy;
	double tfdeg;
public:
	taskframe();
	taskframe(double tfx1,double tfy1,double tfdeg1);
	MatrixXd getTF();
	double getX();
	double getY();
};
taskframe::taskframe(){
	tfx=tfy=0;
	tfdeg=0*pi/180;
}
taskframe::taskframe(double tfx1,double tfy1,double tfdeg1){
	tfx=tfx1;
	tfy=tfy1;
	tfdeg=tfdeg1*pi/180;
}
MatrixXd taskframe::getTF(){
	MatrixXd TF(3,3);
	TF(0,0)=cos(tfdeg);
	TF(0,1)=sin(tfdeg);
	TF(0,2)=tfx;
	TF(1,0)=-sin(tfdeg);
	TF(1,1)=cos(tfdeg);
	TF(1,2)=tfy;
	TF(2.0)=TF(2,1)=0;
	TF(2,2)=1;
	return TF;
}
double taskframe::getX(){
	return tfx;
}
double taskframe::getY(){
	return tfy;
}


class Solver{
private:
	double x,y;
	double theta3,theta4;
	worldframe WF1;
public:
	Solver();
	MatrixXd getXY(double a,double b,double l1,double l2);
	double getdeg1(double c,double d,double l1,double l2);
	double getdeg2(double c,double d,double l1,double l2);
};
Solver::Solver(){
	x=y=0;
	theta3=theta4=0;
}
MatrixXd Solver::getXY(double a,double b,double l1,double l2){
	MatrixXd T(3,3),P(3,1),A(3,1);
	double ang1,ang2,arm1,arm2;
	ang1=a;
	ang2=b;
	arm1=l1;
	arm2=l2;
	T(0,0)=cos(ang1+ang2);
	T(0,1)=-sin(ang1+ang2);
	T(0,2)=arm2*cos(ang1+ang2)+arm1*cos(ang1);
	T(1,0)=sin(ang1+ang2);
	T(1,1)=cos(ang1+ang2);
	T(1,2)=arm2*sin(ang1+ang2)+arm1*sin(ang1); 
	T(2,0)=T(2,1)=0;
	T(2,2)=1;
	P(0,0)=WF1.getX();
	P(1,0)=WF1.getY();
	P(2,0)=1;
	A=T*P;
	return A;
}
double Solver::getdeg1(double c,double d,double l1,double l2){
	double theta34;
	double arm1,arm2,l;
	x=c;
	y=d;
	l=abs(sqrt(x*x+y*y));
	arm1=l1;
	arm2=l2;
	theta34=atan2(y,x)/pi*180;
	theta3=theta34+acos((-arm2*arm2+arm1*arm1+l*l)/(2*arm1*l))/pi*180;
	theta4=acos((-l*l+arm1*arm1+arm2*arm2)/(2*arm1*arm2))/pi*180+180;
	return theta3;  
}
double Solver::getdeg2(double c,double d,double l1,double l2){
	double theta34;
	double arm1,arm2,l;
	x=c;
	y=d;
	l=abs(sqrt(x*x+y*y));
	arm1=l1;
	arm2=l2;
	theta34=atan2(y,x)/pi*180;
	theta3=theta34+acos((-arm2*arm2+arm1*arm1+l*l)/(2*arm1*l))/pi*180;
	theta4=acos((-l*l+arm1*arm1+arm2*arm2)/(2*arm1*arm2))/pi*180+180;
	return theta4;  
}

class Robot{
private:
	double theta1;
	double theta2;
	double seg1;
	double seg2;
	jointframe JF;
	worldframe WF;
	std::vector<taskframe> TF;
	Solver normal;
	Solver inverse;
public:
	Robot();
	Robot(double alpha,double beta,double lenth1,double lenth2);
	void PTPMove(jointframe JF1,double X,double Y);
	void PTPMove(worldframe WF1,double X,double Y);
	void PTPMove(taskframe TF1,worldframe WF2,double X,double Y);
}; 
Robot::Robot(){
	theta1=180/pi*180;
	theta2=270/pi*180;
	seg1=0;seg2=0;
}
Robot::Robot(double alpha,double beta,double lenth1,double lenth2){
	if(alpha>180||alpha<100||beta<220||beta>300)
	{
		cout<<"初始角度错误，请重新输入。"<<endl; 
	}
	theta1=alpha/pi*180;
	theta2=beta/pi*180;
	seg1=lenth1;
	seg2=lenth2;
}
void Robot::PTPMove(jointframe JF1,double X,double Y){
	double x,y,c,d;
	MatrixXd B(3,1);
	c=X+JF1.getdeg1();
	d=Y+JF1.getdeg2();
	x=c*pi/180;
	y=d*pi/180;
	if(c>180||c<100||d<220||d>300)
	{
		cout<<"无法旋转到该位置"<<endl; 
	}
	else
	{
	B=normal.getXY(x,y,seg1,seg2);
	cout<<B(0,0)<<','<<B(1,0)<<endl;	
	}
	
}
void Robot::PTPMove(worldframe WF1,double X,double Y){
	double deg1,deg2,x,y;
	MatrixXd WFs(3,3),coord(3,1),ans(3,1);
	coord(0,0)=X;
	coord(1,0)=Y;
	coord(2,0)=1;
	WFs=WF1.getWF();
	ans=WFs*coord;
	x=ans(0,0);
	y=ans(1,0);
	deg1=inverse.getdeg1(x,y,seg1,seg2);
	deg2=inverse.getdeg2(x,y,seg1,seg2);
	if(deg1>180||deg1<100||deg2<220||deg2>300)
	{
		cout<<"无法旋转到该位置"<<endl;
	}
	else
	{	
	     cout<<deg1<<','<<deg2<<endl;
	}
}
void Robot::PTPMove(taskframe TF1,worldframe WF2,double X,double Y){
	double deg1,deg2,x,y;
	MatrixXd WFs(3,3),TFs(3,3),coordt(3,1),anst(3,1);
	coordt(0,0)=X;
	coordt(1,0)=Y;
	coordt(2,0)=1;
	TFs=TF1.getTF();
	WFs=WF2.getWF();
	anst=WFs*TFs*coordt;
	x=anst(0,0);
	y=anst(1,0);
	deg1=inverse.getdeg1(x,y,seg1,seg2);
	deg2=inverse.getdeg2(x,y,seg1,seg2);
	if(deg1>180||deg1<100||deg2<220||deg2>300)
	{
		cout<<"无法旋转到该位置"<<endl;
	}
	else
	{
	    cout<<deg1<<','<<deg2<<endl;
	}
}

int main()
{
	Robot myRobot(120,240,8,4);
	jointframe JF;
	worldframe WF;
	taskframe TF1(2,2,30),TF2(1,2,0),TF3(1,1,60);
	myRobot.PTPMove(JF,130,260);
	myRobot.PTPMove(WF,-2,9);
	myRobot.PTPMove(TF1,WF,-2,3);
	myRobot.PTPMove(TF2,WF,-1,7);
	myRobot.PTPMove(TF3,WF,-2,3);
	return 0;
}
