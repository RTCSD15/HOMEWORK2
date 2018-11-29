#include "robot.hpp"

Robot::Robot() {};

Robot::Robot(double a1,double a2) {
	armLength[0] = a1;
	armLength[1] = a2;
}

void Robot::PTPmove(Frame f,Point p) {
	Solver solver;
	Point point;
	point = solver.FrameToWF(f,p);
	solver.FrameToJF(point,armLength[0],armLength[1]);
}

void Robot::addFrame(Frame frame) {
	fv.push_back(frame);
} 

void Robot::deleteFrame() {
	fv.pop_back();
}

