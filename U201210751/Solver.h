#include<math.h>
#include "Frame.h"
class Solver
{	public:
		Point Angle[2];               //关节角1,2的角度 
		Point point_World;			// 给定关节角，得到的在世界坐标系中的坐标 
		int number;           //解得关节角的对数，重根视为1 
	public:
		Solver(){number=0;
		}
		void Pos_calculation(double ,double ,Point );//正运动学为把机器人的关节坐标变换成笛卡尔坐标
		void Neg_calculation(double ,double ,Point );//	逆运动学为把机器人的笛卡尔坐标变换成关节坐标	
};
