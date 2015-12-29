//Point.h
 1 #ifndef Point_h
 2 #define Point_h
 3 class Point
 4 {
 5     private :
 6         double px,py;
 7     public :
 8     
 9         double getpx()
10         {
11             return px;
12         }
13         double getpy()
14         {
15             return py;
16         }
17         void setp(double x, double y)
18         {
19             px=x;
20             py=y;
21         }
22 };
23 #endif

复制代码
复制代码

 //Solver.h
 1 #ifndef Solver_h
 2 #define Solver_h
 3 #include <math.h> 
 4 #include <iostream>
 5 #include"Point.h"
 6 using namespace std;
 7 class Solver 
 8 {
 9     private:
10         double ang1,ang2;
11         Point  p;
12     public:
13     
14         void setp(Point p1)
15         {
16             p=p1;
17         }
18         void setang(double a1, double a2)
19         {
20             ang1=a1/180*3.1415926;
21             ang2=a2/180*3.1416926;
22         }
23         void outang ()
24         {
25             cout<<"第一机械臂的转角是"<<ang1/3.1415926*180<<endl;
26             cout<<"第二机械臂的转角是"<<ang2/3.1415926*180<<endl;
27         }
28         void outxy ()
29         {
30             cout<<"x坐标是"<<p.getpx()<<endl;
31             cout<<"y坐标是"<<p.getpy()<<endl;
32         }
33         void angtdik()
34         {
35             p.setp(100*cos(ang1)+100*cos(ang2),100*sin(ang1)+100*sin(ang2));
36         
37         }
38         void diktang()
39         {
40             double sx=p.getpx();
41             double sy=p.getpy();
42             double q=atan(sy/sx);
43             double l=sqrt(sx*sx+sy*sy);
44             ang1=q+acos(-l/200);
45             ang2=acos((l*l-20000)/20000)-3.1415926-ang1;
46         }
47 };
48 #endif

复制代码
复制代码

//Robot.h
#ifndef Robot_h
#define Robot_h
#include "Coord.h"
#include "Trans.h"
#include "Solver.h"
class Robot
{
    private:
        double l1,l2,rang1,rang2;
    public:
        Robot()
        {
            l1=100;
            l2=100;
            rang1=0;
            rang2=0;
        }
        void PTPmove(Coord c1,Point p)
        {
            Trans T;
            Solver S;
            Point p1;
            p1=T.TTW (c1,p);
            S.setp(p1);
            S.diktang();
            S.outang();
            
        }
};
#endif

复制代码
复制代码

//Coord.h
#ifndef Coord_h
#define Coord_h
#include "Point.h"
class Coord
{
    private :
        Point p1;
        double ang;
    public :
        /*Coord(Point p,double ang1)
        {
            p1=p;
            ang=ang1;
        }*/
        void setcoord(Point p,double ang1 )
        {
            p1=p;
            ang=ang1;
        }
        Point getp1()
        {
            return p1;
        }
        double getang()
        {
            return ang/180*3.1415926;
        }
};
#endif

复制代码
复制代码

//Trans.h
#ifndef Trans_h
#define Trans_h
#include "Point.h"
#include "Coord.h"
#include <math.h>
class Trans
{
    public :

        Point TTW (Coord c1,Point p3)
        {
            Point p1=c1.getp1();
            Point p2;
            double ang=c1.getang();
            double x,y,x1,y1;
            x=p3.getpx();
            y=p3.getpy();
            x1=x*cos(ang)-y*sin(ang)+p1.getpx();
            y1=x*sin(ang)+y*cos(ang)+p1.getpy();
            p2.setp(x1,y1);
            return p2;            
        }    

};
#endif

复制代码
复制代码

//main.cpp
#include <iostream>
#include "Point.h"
#include "Solver.h"
#include "Robot.h"
#include "Coord.h"
#include "Trans.h" 
using namespace std;
int  main ()
{
    Robot r;
    Point p;
    Point p1;
    p.setp(30,20);
    p1.setp(40,60);
    Coord t;
    t.setcoord(p,45);
    r.PTPmove(t,p1);
    return 0;
}

复制代码
