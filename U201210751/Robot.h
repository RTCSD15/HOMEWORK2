#include<math.h>
#include "Solver.h"
class Robot
{	public:
		double L1,L2;
		Frame JointFrame1,JointFrame2;
		Point AimPoint_WorldFrame;
		Solver solver;
	public:
		Robot(double =0,double =0);
		void PTPMove(Frame ,Point );// 给定坐标系和点，求关节角 
		void PTPMove(Point );	//给定关节角，求关节3的世界坐标系 

};
