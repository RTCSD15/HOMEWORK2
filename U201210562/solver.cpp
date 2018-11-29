#include "solver.hpp"

Solver::Solver(){};

Solver::Solver(JointFrame jf,Frame f) {
	jointframe = jf;
	frame = f;
}

Point Solver::move(Point p1,Point p2) {
	Vector2d v;
	v = p1.getPoint()+p2.getPoint();
	Point point(v);
	return point;
}

Point Solver::rotate(Point p,double deg) {
	Matrix2d m;
	Vector2d v;
	double rad = deg*PI/180;
	m(0,0) = cos(rad);
	m(0,1) = - sin(rad);
	m(1,0) = sin(rad);
	m(1,1) = cos(rad);
	v = m*p.getPoint();
	Point point(v);
	return point;
}

Point Solver::FrameToWF(Frame f,Point p) {
	Point p1 = rotate(p,f.getDegree());
	Point p2 = move(p1,f.getPoint());
	return p2;
}

void Solver::FrameToJF(Point p,double arm1,double arm2) {
	Vector2d v = p.getPoint();
	double l = sqrt(v[0]*v[0]+v[1]*v[1]);
	if(l>=(arm1+arm2)||l<=abs(arm1-arm2)) {
		cout<<"机械手无法到达该位置！"<<endl;
	} else {
		double rad1=acos((arm1*arm1+l*l-arm2*arm2)/(2*arm1*l));
		double rad2=acos((arm1*arm1+arm2*arm2-l*l)/(2*arm1*arm2));
		double rad3=atan(v[1]/v[0]);
		jointframe.setDeg(rad1+rad3,rad2+PI);
		cout<<"关节1转角为："<<jointframe.getDeg1()*180/PI<<'\t';
		cout<<"关节2转角为："<<jointframe.getDeg2()*180/PI<<endl;
	}	
}
