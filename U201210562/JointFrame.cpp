#include "JointFrame.hpp"

JointFrame::JointFrame() {
	jointDegree[0] = 0;
	jointDegree[1] = 0;
}

JointFrame::JointFrame(double d1,double d2) {
	jointDegree[0] = d1;
	jointDegree[1] = d2;
}

void JointFrame::setDeg(double d1,double d2) {
	jointDegree[0] = d1;
	jointDegree[1] = d2;
}

double JointFrame::getDeg1() {
	return jointDegree[0];
}

double JointFrame::getDeg2() {
	return jointDegree[1];
}

