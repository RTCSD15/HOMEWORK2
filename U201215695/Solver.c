#include"Solver.h"
#include"Point.h"
#include<cmath>

Solver::Solver(){
	SolverResult.x=0;
	SolverResult.y=0;
}

Point Solver::NegativeKinematics(double L1,double L2,Point point){
	double  squ=sqrt(point.x*point.x+point.y*point.y);
	SolverResult.x=acos((squ*squ+L1*L1-L2*L2)/(2*L1*squ))+acos(point.x/squ);
	SolverResult.y=acos((squ*squ+L2*L2-L1*L1)/(2*L2*squ))+acos(point.x/squ);
	SolverResult.updatePoint(SolverResult.x,SolverResult.y);
	return SolverResult;
}

Point Solver::PositiveKinematics(double L1,double L2,Point Angles){
	SolverResult.x=L1 *cos(Angles.x/180*M_PI)+L2 *cos(Angles.y/180*M_PI);
	SolverResult.y=L1 *sin(Angles.x/180*M_PI)+L2 *sin(Angles.y/180*M_PI);
	SolverResult.updatePoint(SolverResult.x,SolverResult.y);
	return SolverResult;
}



Point Solver::FrameToWF(WorldFrame frame,Point AimPoint){
	return frame.To_WF(AimPoint);}
Point Solver::FrameToWF(JointFrame frame,Point AimPoint){
	return frame.To_WF(AimPoint);}
Point Solver::FrameToWF(TaskFrame frame,Point AimPoint){
	return frame.To_WF(AimPoint);}
