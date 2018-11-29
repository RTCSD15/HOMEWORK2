#ifndef _TASKFRAME_HPP_
#define _TASKFRAME_HPP_

#include "Frame.hpp"


using namespace Eigen;

class TaskFrame: public Frame {
	
	private:
		Vector2d origin;
		double rotDegree;
	
	public:
		TaskFrame();
		TaskFrame(Vector2d origin,double rotdegree);
		
};


#endif 
