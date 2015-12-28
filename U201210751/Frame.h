#include<math.h>
#include "Point.h"
class Frame
{	public:
		Point point_Frame; //在某坐标系的坐标 
		Point point_WorldFrame;//某坐标系的点在世界坐标系中的坐标 
		Point O_Point;		//某坐标系原点相对于世界坐标系的坐标值，默认值为原点	 
		double O_Angle;	    //某坐标系相对于世界坐标系x轴正方向的转角，默认值为0，单位：弧度；逆时针为+，角度  -pi~pi
	public:
		Frame(double =0);					//关于原点和转角的构造函数 ，默认值为世界坐标系自身 
		Frame(Point ,double =0);		
		Point InputPoint_WorldFrame(Point );	// 给定某坐标系中的坐标，返回在世界坐标系的坐标 
		Point InputPoint_Frame(Point );      // 给定在世界坐标系的坐标，返回某坐标系的坐标 
};
