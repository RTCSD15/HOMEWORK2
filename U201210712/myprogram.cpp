#include <iostream>
#include <math.h>
#define PI 3.1415926
using namespace std;

class Point{
	private:
		double x;
		double y;
	public: 
		Point(){}
		Point(double a,double b){
			x=a;
			y=b;
		}
		double getX(){
			return x;
		}
		double getY(){
			return y;
		}
		void setX(double a){
			x=a;
		}
		void setY(double b){
			y=b;
		}
		
};

class Joint{
	private:
		double deg1;
		double deg2;
	public: 
		Joint(){}
		Joint(int a,int b){
			deg1=a;
			deg2=b;
		}
		void setDeg1(double rad){
			deg1=rad*180/PI;
		}
		void setDeg2(double rad){
			deg2=rad*180/PI;
		}
		double getDeg1(){
			return deg1;
		}
		double getDeg2(){
			return deg2;
		}
		
};

class Frame{
	private:
		Point origin;
		double degree;
	public:
		Frame(){}
	 	Frame(Point orig,double deg){
		 	origin=orig;
		 	degree=deg;
		 }
		Point getPoint(){
			return origin;
		}
		double getDegree(){
			return degree;
		}
		
};

class Solver{
	private:
		Joint joint;
		Frame frame;
	public:
		Point move(Point p1,Point p2){
			double tempx=p1.getX()+p2.getX();
			double tempy=p1.getY()+p2.getY();
			Point point(tempx,tempy);
			return point;
		}
		Point rotate(Point p,double deg){
			double tempx;
			double tempy;
			tempx=p.getX()*cos(PI*deg/180)-p.getY()*sin(PI*deg/180);
			tempy=p.getX()*sin(PI*deg/180)+p.getY()*cos(PI*deg/180);
            Point point(tempx,tempy);
			return point;
		}
		
		//将用户坐标系转化为世界坐标系 
		Point FrameStandardize(Frame fr,Point po){
			Point point1=rotate(po,fr.getDegree());
			Point point2=move(point1,fr.getPoint());
		    return point1;
		}
		//在世界坐标系下反算关节坐标 
		FrameToJoint(Point po,double arm1,double arm2){
			double len= sqrt(po.getX()*po.getX()+po.getY()*po.getY());
			
			if(len>=(arm1+arm2)||len<=abs(arm1-arm2)){
				cout<<"坐标超出范围，机器人无法达到"<<endl;
			}else{
				double rad1=acos((arm1*arm1+len*len-arm2*arm2)/(2*arm1*len));
				double rad2=acos((arm1*arm1+arm2*arm2-len*len)/(2*arm1*arm2));
				double rad11=atan(po.getY()/po.getX());
				double rad22=PI;
				joint.setDeg1(rad1+rad11);
				joint.setDeg2(rad2+rad22);
				cout<<"关节1应转动角度为："<<joint.getDeg1()<<" 关节2应转动角度为："<<joint.getDeg2()<<endl;
			}
					
		}
		//将世界坐标系下的关节坐标还原为用户坐标系 
		FrameReturn(Frame fr){
		}
		JointToFrame(Point po){
			
		}
		
};
class Robot{
	private:
		double arm1;
		double arm2;
		double arm1Range;
		double arm2Range;
	public:
		Robot(){}
		Robot(double a,double b){
			arm1=a;
			arm2=b;
		}
		void PTPmove(Frame fr,Point po){
			Solver solver;
			Point point;
			
			point=solver.FrameStandardize(fr,po);
			solver.FrameToJoint(point,arm1,arm2);
				
		}
};
 
int main(){
	Robot myRobot(4,3);   //定义机器人的关节长度 
	
	Point origin(0,0);
	Point origin1(2,3);
	Point origin2(6,1);
	Point origin3(5,9);   //设置一组坐标系原点在世界坐标系中的坐标值 
	
	Point point1(-5,1);
	Point point2(-3,-1);
	Point point3(4,2);
	Point point4(3,7);   // 设置每个坐标系中机器人运动目标坐标点 
	
	Frame WF(origin,0);
	Frame TF1(origin1,30);
	Frame TF2(origin2,60);
	Frame TF3(origin3,90);  //建立世界坐标系与用户坐标系 
	
	
	myRobot.PTPmove(WF,point1);
	myRobot.PTPmove(TF1,point2);
	myRobot.PTPmove(TF2,point3);
	myRobot.PTPmove(TF3,point4);
	
	
	return 0;
}
