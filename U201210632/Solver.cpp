#include "Solver.h"
#include<Eigen/Dense>
using namespace Eigen;
#define PI 3.1415926
void Solver::WtoT(TaskFrame t,Point &p){
    MatrixXd m(3,3);
    MatrixXd pt(1,3);
    m(0,0)=cos(t.deg*PI/180);
    m(0,1)=sin(t.deg*PI/180);
    m(0,2)=0;
    m(1,0)=-sin(t.deg*PI/180);
    m(1,1)=cos(t.deg*PI/180);
    m(1,1)=0;
    m(2,0)=t.x,
    m(2,1)=t.y;
    m(2,2)=0;
    pt(0,0)=p.x;
    pt(0,1)=p.y;
    pt(0,2)=1;
    pt*=m;
    p.x=pt(0,0);
    p.y=pt(0,1);    
}
void Solver::TtoW(TaskFrame t,Point &p){
    MatrixXd m(3,3);
    MatrixXd pt(1,3);
    m(0,0)=cos(-t.deg*PI/180);
    m(0,1)=sin(-t.deg*PI/180);
    m(0,2)=0;
    m(1,0)=-sin(-t.deg*PI/180);
    m(1,1)=cos(-t.deg*PI/180);
    m(1,1)=0;
    m(2,0)=t.x;
    m(2,1)=t.y;
    m(2,2)=0;
    pt(0,0)=-p.x;
    pt(0,1)=-p.y;
    pt(0,2)=1;
    pt*=m;
    p.x=pt(0,0);
    p.y=pt(0,1);
}
void Solver::TtoJ(Point &p,double a1,double a2){
    double l,deg1,deg2,deg3;
    l=sqrt(p.x*p.x+p.y*p.y);
    deg1=atan(p.y/p.x);
    deg2=acos((a1*a1+l*l-a2*a2)/(2*a1*l));
    deg3=acos((a1*a1+a2*a2-l*l)/(2*a1*a2));
    p.x=(deg1+deg2)*180/PI;
    p.y=deg3*180/PI+180;
}
void Solver::JtoT(Point &p,double a1,double a2){
    p.x=a1*cos(p.x)+a2*cos(p.y);
    p.y=a1*sin(p.x)+a2*sin(p.y);
}
