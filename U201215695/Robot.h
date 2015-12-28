#ifndef _ROBOT_H_
#define _ROBOT_H_ 
#include"Solver.h"
//#include"Frame.h"
#include <vector>

class Robot{
	private:
		double JointAngle1,JointAngle2,L1,L2;
		Point InsightPoint;
		WorldFrame WF0;
		JointFrame JF0;
		TaskFrame  TF0;
		Solver solver;
		std::vector<TaskFrame> TaskFrameVec;
	public:
		Robot(double length1,double length2){
			L1=length1;L2=length2;
		}
	    void PTPmove(WorldFrame f,Point p);
	    void PTPmove(JointFrame f,Point p);
	    void PTPmove(TaskFrame f,Point p);
	    void addTFrame(TaskFrame frame);
	    void deleteFrame();
	};
#endif
