#include "TaskFrame.hpp"

TaskFrame::TaskFrame() {
	origin[0] = 0;
	origin[1] = 0;
	rotDegree = 0;
}

TaskFrame::TaskFrame(Vector2d origin,double rotdegree) {
	this->origin = origin;
	rotDegree = rotdegree;
}
