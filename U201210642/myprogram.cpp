#ifndef FRAME_H
#define FRAME_H
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#define pi 3.1415926
using Eigen::MatrixXd;
using namespace std;

class jointframe{
private:
    double deg1;
    double deg2;
public:
    jointframe(double deg=0,double deg4=0);
    double getdeg1();
    double getdeg2();
};

class worldframe{
private:
    double wfx;
    double wfy;
    double wfdeg;
public:
    worldframe();
};



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
#endif
#include "frame.h"
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
worldframe::worldframe(){
    wfx=wfy=0;
    wfdeg=0*pi/180;
}
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
#ifndef SOLVER_H
#define SOLVER_H
#include"frame.h"

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
#endif #include"solver.h"
Solver::Solver(){
    x=y=0;
    theta3=theta4=0;
}
MatrixXd Solver::getXY(double a,double b,double l1,double l2){
    MatrixXd T(3,3);
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
    return T;
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
#ifndef ROBOT_H
#define ROBOT_H
#include"solver.h"
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
    void PTPMove(taskframe TF1,double X,double Y);
}; 
#endif#include"robot.h"
Robot::Robot(){
    theta1=180/pi*180;
    theta2=270/pi*180;
    seg1=0;seg2=0;
}
Robot::Robot(double alpha,double beta,double lenth1,double lenth2){
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
    cout<<"关节坐标为"<<'('<<B(0,0)<<','<<B(1,0)<<')'<<endl;    
    }
    
}
void Robot::PTPMove(worldframe WF1,double X,double Y){
    double deg1,deg2,x,y;
    x=X;
    y=Y;
    deg1=inverse.getdeg1(x,y,seg1,seg2);
    deg2=inverse.getdeg2(x,y,seg1,seg2);
    if(deg1>180||deg1<100||deg2<220||deg2>300)
    {
        cout<<"无法旋转到该位置"<<endl;
    }
    else
    {    
         cout<<"关节坐标为"<<'('<<deg1<<','<<deg2<<')'<<endl;
    }
}
void Robot::PTPMove(taskframe TF1,double X,double Y){
    double deg1,deg2,x,y;
    MatrixXd TFs(3,3),coordt(3,1),anst(3,1);
    coordt(0,0)=X;
    coordt(1,0)=Y;
    coordt(2,0)=1;
    TFs=TF1.getTF();
    anst=TFs*coordt;
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
        cout<<"关节坐标为"<<'('<<deg1<<','<<deg2<<')'<<endl;
    }
}
#include <iostream>
#include"robot.h"
/* run this program using the console pauser or add your own getch, system("pause") or input loop */

int main(int argc, char** argv) {
	Robot Robot(140,200,6,4);
    jointframe JF;
    worldframe WF;
    taskframe TF1(4,3,35),TF2(2,2,0),TF3(0,1,40);
    Robot.PTPMove(JF,150,240);
    Robot.PTPMove(WF,-1,7);
    Robot.PTPMove(TF1,-2,3);
    Robot.PTPMove(TF2,-1,5);
    Robot.PTPMove(TF3,-1,3);
    return 0;
}
