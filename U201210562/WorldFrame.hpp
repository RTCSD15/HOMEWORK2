#ifndef _WORLDFRAME_
#define _WORLDFRAME_

#include "Frame.hpp"

using namespace Eigen;
class WorldFrame:public Frame {
	
	private:
		Vector2d origin;
		double rotDegree;
		
	public:
		WorldFrame();
		void show();
		
};

#endif
