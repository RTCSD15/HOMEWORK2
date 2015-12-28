#include "Point.hpp"

Point::Point() {};

Point::Point(Vector2d v) {
	pos = v;
}

Vector2d Point::getPoint() {
	return pos;
}

void Point::setPoint(Vector2d v) {
	pos = v;
}
