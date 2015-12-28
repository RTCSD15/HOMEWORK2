//Point2D.h 点类
#ifndef POINT2D
#define POINT2D

#include<Eigen\Dense>
using namespace Eigen;

class Point2D
{
private:
    double x;
    double y;

public:
    //Point2D(){};
    Point2D(double X = 0.0,double Y = 0.0){x = X; y = Y;}

    double getX(){return x;}
    double getY(){return y;}

    Vector2d  PTV();
    Point2D & operator+(const Point2D &);
    Point2D & operator-(const Point2D &);
    Point2D & operator=(const Point2D &);
};

#endif POINT2D


//Point2D.cpp
#include "Point2D.h"

Point2D & Point2D::operator+(const Point2D & p)
{
    Point2D temp(x+p.x,y+p.y);
    return temp;
}

Point2D & Point2D::operator-(const Point2D & p)
{
    Point2D temp(x-p.x,y-p.y);
    return temp;
}

Point2D & Point2D::operator=(const Point2D & p)
{
    Point2D temp(p.x,p.y);
    return temp;
}

Vector2d  Point2D::PTV()
{
    Vector2d temp;
    temp(0) = getX();
    temp(1) = getY();
    return temp;
}


//Frames.h 坐标系类
#ifndef Frame
#define Frame

#include"Point2D.h"

class Frames
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frames(){};
    Frames(double,double,double,double,double,double);
    ~Frames(){};
    Matrix2d  rotation(Frames &);    //两坐标系之间的旋转矩阵
    Vector2d  move(Frames &);        //两坐标系之间的移动向量

private:
    Point2D originalPoint;
    Vector2d XAxis;
    Vector2d YAxis;
};

#endif Frame


//Frames.cpp
#include"Frames.h"
#include"math.h"
#include<cmath>
#include<iostream>
using namespace std;

#define pi 3.141592654

Frames::Frames(double px,double py,double vxx,double vxy,double vyx,double vyy)
    :originalPoint(px,py),XAxis(vxx,vxy),YAxis(vyx,vyy)
{
    
    if(XAxis.dot(YAxis) != 0.0)
    {
        cout << "输入的坐标轴不垂直！" << endl;
    }
}

//两坐标系之间的旋转矩阵
Matrix2d  Frames::rotation(Frames& fram)
{
    double s1;
    double c1;
    double theta;
    Matrix2d temp;
    if(fram.XAxis(0) !=0 && XAxis(0)!= 0)
        theta = atan(fram.XAxis(1)/fram.XAxis(0))-atan(XAxis(1)/XAxis(0));
    if(fram.XAxis(0) !=0 && XAxis(0)== 0)
    {
        if(XAxis(1)== 1)
            theta = atan(fram.XAxis(1)/fram.XAxis(0))- 90/180*pi;
        if(XAxis(1)== -1)
            theta = atan(fram.XAxis(1)/fram.XAxis(0))- 180/180*pi;
    }
    if(fram.XAxis(0) ==0 && XAxis(0)!= 0)
    {
        double k = atan(XAxis(1)/XAxis(0));
        if(fram.XAxis(1)== 1)
            theta = (double)90/180*pi - k;
        if(fram.XAxis(1)== -1)
            theta = (double)180/180*pi - atan(XAxis(1)/XAxis(0));
    }
    s1 = sin(theta);
    c1 = cos(theta);
    if(abs(s1) < 0.000000000000001)
        s1 = 0;
    if(abs(c1) < 0.000000000000001)
        c1 = 0;
    temp << c1,-s1,
            s1,c1;
    return temp;
}


//两坐标系之间的移动向量
Vector2d  Frames::move(Frames & fra)
{
    Vector2d temp;
    temp = fra.originalPoint.PTV()-originalPoint.PTV();
    return temp;
}


//Solver.h  解算类
#ifndef SOLVER
#define SOLVER

#include"Frames.h"

class Solver 
{
public:
    Solver(){};
    ~Solver(){};
    Vector2d  directSolution(double link1,double link2,double th1,double th2);
    Matrix2d  inverSolution(double link1,double link2,Frames &, Point2D &);
private:

};




#endif SOLVER

//Solver.cpp
#include"Solver.h"
#include<iostream>
using namespace std;
#include<cmath>
#define pi 3.141592654

Vector2d Solver::directSolution(double link1,double link2,double th1,double th2)
{
    Vector2d temp;
    temp(0) = link1*cos(th1) + link2*cos(th1+th2);
    temp(1) = link1*sin(th1) + link2*sin(th1+th2);

    return temp;
}

Matrix2d  Solver::inverSolution(double link1,double link2,Frames &f,Point2D & p)
{
    Vector3d point;
    Vector2d result;    //p在基坐标系上的坐标值
    Vector2d moveF;        //坐标系f与极坐标系原点之间的向量
    Matrix2d rota;        //坐标系f与基坐标系之间的旋转矩阵
    Matrix3d T;            //坐标系f与基坐标系之间的转换矩阵
    Matrix2d Theta;    //关节角
    Vector2d temp;
    Frames base(0.0,0.0,1.0,0.0,0.0,1.0);

    //把坐标系f的点p转换到基坐标
    rota = base.rotation(f);
    moveF = base.move(f);
    moveF = moveF.transpose();
    T << rota,moveF,
        0,0,1;
    temp = p.PTV();
    point << temp,1;
    point = point.transpose();
    point = T*point;
    result << point(0),point(1);

    //判断点是否在找工作空间
    double dis = sqrt(result.dot(result));
    if(dis>(link1+link2) || dis<(link1-link2))
    {
        Matrix2d zeros(2,2);
        cout << "坐标点不在工作空间！"<< endl;
        return zeros;
    }

    //求反解关节值
    else
    {
    double beita = atan2(result(1),result(0));
    double fi;
    double theta11,theta12,theta21,theta22;

    fi = acos((dis*dis+link1*link1-link2*link2)/(2*link1*dis));
    theta12 = acos((dis*dis-link1*link1-link2*link2)/(2*link1*link2))*180/pi;

    if(theta12<0)
        theta11 = (beita + fi)*180/pi;
    else 
        theta11 = (beita - fi)*180/pi;

    theta22 = -acos((dis*dis-link1*link1-link2*link2)/(2*link1*link2))*180/pi;

    if(theta22<0)
        theta21 = (beita + fi)*180/pi;
    else 
        theta21 = (beita - fi)*180/pi;
    Theta << theta11,theta12,
        theta21,theta22;
    return Theta;
    }
    
}


//Robot.h 机器人类
#ifndef ROBOT
#define ROBOT

#include<vector>    
#include"Solver.h"
using namespace std;


class Robot:Solver
{
public:
    Robot(){};
    Robot(vector<Frames> &fr,double link1 = 100,double link2 = 100,
        double the1=0,double the2 = 0);
    ~Robot(){};

    void PTPMove(Frames &,Point2D &);
    void thetaToTCP(double,double);

    inline double getLink1(){return linkage1;}
    inline double getLink2(){return linkage2;}
    inline double getTheta1(){return theta1;}
    inline double getTheta2(){return theta2;}

private:
    vector <Frames> fra;
    double linkage1;
    double linkage2;
    double theta1;
    double theta2;
};




#endif ROBOT


//Robot.cpp
#include"Robot.h"
#include<iostream>
using namespace std;

Robot::Robot(vector<Frames>& fr,double link1,double link2
             ,double the1,double the2):
             linkage1(link1),linkage2(link2)
             ,theta1(the1),theta2(the2)
{
    for(int i=0; i < fr.size(); i++)
        fra.push_back(fr[i]);
    /*if (fr.size() == 0)
    {
        Frames base(0.0,0.0,1.0,0.0,0.0,1.0);
        fra.push_back(base);
    }*/
}

void Robot::PTPMove(Frames & f,Point2D & p)
{
    Matrix2d temp;
    Matrix2d zeros(2,2);
    temp = inverSolution(linkage1,linkage2,f,p);
    if (temp != zeros)
        cout << "关节值： " <<endl << temp << endl;
    cout << endl;
}
void Robot::thetaToTCP(double th1,double th2)
{
    Vector2d temp;
    temp = directSolution(linkage1,linkage2,th1,th2);
    cout << "运动到的点坐标：" << endl << temp;
    cout << endl;
}



//测试程序

#include"Robot.h"
#include <iostream>
using namespace std;

int main()
{
    Point2D p(90,92);    //要运动到的目标点坐标
    Point2D p1(500,1305);
    vector<Frames> robotFrames;
    Frames WF(0.0,0.0,1.0,0.0,0.0,1.0);    //世界坐标系
    robotFrames.push_back(WF);
    Frames J1(0.0,0.0,1.0,0.0,0.0,1.0);    //关节1坐标系
    robotFrames.push_back(J1);
    Frames J2(200.0,0.0,1.0,0.0,0.0,1.0);    //关节2坐标系
    robotFrames.push_back(J2);
    Frames TCP(200.0,150.0,0.0,1.0,-1.0,0.0);    //工具坐标系
    robotFrames.push_back(TCP);

    vector<int> b;
    for(int i = 0; i< 5;i++)
        b.push_back(i);
    //工作坐标系
    Frames TF1(1.0,1.0,0.8,0.6,-0.6,0.8);
    Frames TF2(1.0,2.0,0.0,1.0,-1.0,0.0);

    Robot myrobot(robotFrames,200.0,150.0,0.0,90.0);

    myrobot.PTPMove(WF,p);
    myrobot.PTPMove(J1,p);
    myrobot.PTPMove(J2,p);
    myrobot.PTPMove(TF1,p);
    myrobot.PTPMove(TF2,p);
    myrobot.PTPMove(TCP,p);

    myrobot.PTPMove(WF,p1);
    myrobot.PTPMove(J1,p1);
    myrobot.PTPMove(J2,p1);
    myrobot.PTPMove(TF1,p1);
    myrobot.PTPMove(TF2,p1);
    myrobot.PTPMove(TCP,p1);

    system("pause");
     return 1;
}Enter file contents here
