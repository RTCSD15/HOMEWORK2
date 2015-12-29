 #include <iostream>
 #include<Eigen/Dense>
 #include<cmath>
 #define PI 3.1415926
 using namespace std;
 using namespace Eigen;
 class Point{
     public:
          double x;
          double y;
          
          Point(double a=0,double b=0);
          void SetP(double a,double b);
          
  };
  Point::Point(double a,double b){
      x=a;
      y=b;
  }
  void Point::SetP(double a,double b){
      x=a;
      y=b;
  }
  
  class WorldFrame{
      
  };
  class TaskFrame{
      public:
          double x;
          double y;
          double ang;
      
          TaskFrame(double a=0,double b=0,double d=0);
          void WtoT(Point &p);
          void TtoW(Point &p);
          void TtoJ(Point &p,double a1,double a2);
  };
  
  TaskFrame::TaskFrame(double a,double b,double A){
      x=a;
      y=b;
      ang=A;
  }
  
  void TaskFrame::WtoT(Point &p){
      MatrixXd m(3,3);
      MatrixXd pt(1,3);
      double deg;
      deg=ang*PI/180;
      m(0,0)=cos(deg);
      m(0,1)=sin(deg);
      m(0,2)=0;
      m(1,0)=-sin(deg);
      m(1,1)=cos(deg);
      m(1,1)=0;
      m(2,0)=x,
      m(2,1)=y;
      m(2,2)=0;
      pt(0,0)=p.x;
      pt(0,0)=p.y;
      pt(0,2)=1;
      pt*=m;
      p.x=pt(0,0);
      p.y=pt(0,1);    
  }
  void TaskFrame::TtoW(Point &p){
      MatrixXd m(3,3);
      MatrixXd pt(1,3);
      double deg;    
      deg=ang*PI/180;
      m(0,0)=cos(deg);
      m(0,1)=sin(deg);
      m(0,2)=0;
      m(1,0)=-sin(deg);
      m(1,1)=cos(deg);
      m(1,1)=0;
      m(2,0)=x;
      m(2,1)=y;
      m(2,2)=0;
      pt(0,0)=-p.x;
      pt(0,0)=-p.y;
      pt(0,2)=1;
      pt*=m;
      p.x=pt(0,0);
      p.y=pt(0,1);
  }
  void TaskFrame::TtoJ(Point &p,double a1,double a2){
      double l,deg1,deg2,deg3;
      l=sqrt(p.x*p.x+p.y*p.y);
      deg1=atan(p.y/p.x);
      deg2=acos((a1*a1+l*l-a2*a2)/(2*a1*l));
      deg3=acos((a1*a1+a2*a2-l*l)/(2*a1*a2));
      p.x=(deg1+deg2)*180/PI;
      p.y=deg3*180/PI+180;
  }
  class JointFrame{
  
  };
 
 class Robot{
     private:
         double arm1,arm2,deg1min,deg2min,deg1max,deg2max;
     public:
         Robot(double a1=1,double a2=1,double d1min=0,double d2min=0,double d1max=180,double d2max=360);
         void SetRobot(double a1=1,double a2=1,double d1min=0,double d2min=0,double d1max=180,double d2max=360);
         void PTPMove(WorldFrame wf,TaskFrame tf,Point p);
         void PTPMove(TaskFrame tf,Point P);
         void PTPMove(JointFrame jf,Point P);
         void print(Point &p);
 };
 
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
     tf.WtoT(p);
     tf.TtoJ(p,arm1,arm2);
     print(p);
 }
 void Robot::PTPMove(TaskFrame tf,Point p){
     tf.TtoJ(p,arm1,arm2);
     print(p);
 }
 void Robot::PTPMove(JointFrame jf,Point p){
     print(p);
 }
 void Robot::print(Point &p){
     if(p.x>=deg1min||p.y<=deg1max){
         cout<<"机器人的关节坐标为：("<<p.x<<','<<p.y<<')'<<endl;
     }
     else cout<<"无法旋转到该位置"<<endl; 
 }
 
 
 
 int main(int argc, char** argv) {
     Robot myRobot(10,10);
     WorldFrame WF;
     TaskFrame TF1(0,0,0),TF2(2,2,30),TF3(1,3,60);
     JointFrame JF;
     Point P1(0,0),P2(1,1),P3(2,2),P4(2,1),P5(3,7);
     myRobot.PTPMove(JF,P1);
     myRobot.PTPMove(WF,TF1,P2);
     myRobot.PTPMove(TF1,P3);
     myRobot.PTPMove(TF2,P4);
     myRobot.PTPMove(TF3,P5);
     return 0;
 }
