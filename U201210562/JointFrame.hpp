#ifndef _JOINTFRAME_HPP_
#define _JOINTFRAME_HPP_

#include "Frame.hpp"


class JointFrame {
	
	private:
		double jointDegree[2] ;
	
	public:
		JointFrame();
		JointFrame(double d1,double d2);
		void setDeg(double d1,double d2);
		double getDeg1();
		double getDeg2();
		
};		

#endif 
