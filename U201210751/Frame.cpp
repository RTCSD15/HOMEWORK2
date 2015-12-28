#include "Frame.h"
Frame::Frame (double InputAngle)
{	O_Angle=InputAngle*M_PI/180;//转换成弧度制 
}
Frame::Frame (Point InputOrigin,double InputAngle)
{	O_Angle=InputAngle*M_PI/180;
	O_Point=InputOrigin;
	point_WorldFrame=InputOrigin;   //确保在没有调用两个InputPoint函数时，point_WorldFrame仍是在世界坐标系的坐标
}                                    //只不过此时point_Frame是某坐标系的原点 
Point Frame::InputPoint_WorldFrame(Point Input_Frame)
{	point_Frame=Input_Frame;
	point_WorldFrame.x=O_Point.x+Input_Frame.x*cos(O_Angle)-Input_Frame.y*sin(O_Angle);
	point_WorldFrame.y=O_Point.y+Input_Frame.x*sin(O_Angle)+Input_Frame.y*cos(O_Angle);	
	return point_WorldFrame;
}
Point Frame::InputPoint_Frame(Point Input_WorldFrame)
{	point_WorldFrame=Input_WorldFrame;
	point_Frame.x=(Input_WorldFrame.x-O_Point.x)*cos(O_Angle)+(Input_WorldFrame.y-O_Point.y)*sin(O_Angle);
	point_Frame.y=(Input_WorldFrame.y-O_Point.y)*cos(O_Angle)-(Input_WorldFrame.x-O_Point.x)*sin(O_Angle);
	return point_Frame;
}
