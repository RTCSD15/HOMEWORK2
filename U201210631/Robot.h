#ifndef ROBOT
#define ROBOT
#include "Point.h"
#include"Frame.h"
class Robot{
	private:
		double arm1,arm2,deg1min,deg2min,deg1max,deg2max;
	public:
		Robot(double a1=1,double a2=1,double d1min=0,double d2min=0,double d1max=180,double d2max=360);
		void SetRobot(double a1=1,double a2=1,double d1min=0,double d2min=0,double d1max=180,double d2max=360);
		void PTPMove(WorldFrame wf,TaskFrame tf,Point p);
		void PTPMove(TaskFrame tf,Point p);
		void PTPMove(JointFrame jf,Point p);
		void print(Point &p);
};

#endif
