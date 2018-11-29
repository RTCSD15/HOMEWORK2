	#ifndef _ROBOT_HPP_
	#define _ROBOT_HPP_
	
	#include "Frame.hpp"
	#include "Solver.hpp"
	#include <vector>
	
	class Robot {
		private:
			double armLength[2];
			vector<Frame> fv;
			
		public:
			Robot();
			Robot(double a1,double a2);
			void PTPmove(Frame f,Point p);
			void addFrame(Frame frame);
			void deleteFrame();
					
	};
	
	
	#endif 
