#include <iostream>
#include "Robot.h"

int main(){
	Point one(5,20),two(0,30),three(0,20),a(0,0),c(10,0),d(5,0),e(10,0);
	Robot MyRobot(10,20);
	WorldFrame wframe(a,0);
	JointFrame jframe1(a,0);
	JointFrame jframe2(c,0);
	TaskFrame tframe1(d,0);
	TaskFrame tframe2(e,90);
	MyRobot.PTPmove(tframe1,one);
	MyRobot.PTPmove(wframe,two);
	MyRobot.PTPmove(jframe1,three);
}
