#ifndef _POINT_HPP_
#define _POINT_HPP_

#include <Eigen/Dense>

using namespace Eigen;

class Point {
	
	private:
		Vector2d pos;
		
	public:
		Point();
		
		Point(Vector2d v);
		
		Vector2d getPoint();
		
		void setPoint(Vector2d v);		
};

#endif
