#ifndef _SOLVER_H_
#define _SOLVER_H_ 
#include<math.h>
#include"Point.h"
#include"Frame.h"

class Solver{
	private:
		Point SolverResult;
	public:
		Solver();
		Point PositiveKinematics(double L1,double L2,Point Angles);
		Point NegativeKinematics(double L1,double L2,Point point);
		Point FrameToWF(WorldFrame frame,Point AimPoint);
		Point FrameToWF(JointFrame frame,Point AimPoint);
		Point FrameToWF(TaskFrame frame,Point AimPoint);
		
	};
#endif
