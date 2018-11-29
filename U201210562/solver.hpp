#ifndef _SOLVER_HPP_
#define _SOLVER_HPP_
#define PI 3.1415926

#include "Frame.hpp"
#include "JointFrame.hpp"
#include "Point.hpp"
#include "math.h"

class Solver {
	
	private: 
		JointFrame jointframe;
		Frame frame;
		
	public:
		Solver();
		Solver(JointFrame jf,Frame f);
		Point move(Point p1,Point p2);
		Point rotate(Point p,double deg);
		Point FrameToWF(Frame f,Point p);
		void FrameToJF(Point p,double arm1,double arm2);		
};

#endif 
