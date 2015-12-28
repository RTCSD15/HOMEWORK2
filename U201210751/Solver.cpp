#include<math.h>
#include"Solver.h"
void Solver::Pos_calculation(double length1,double length2,Point Input_Angle)
{	Angle[0].x=Input_Angle.x*M_PI/180;	                             //给定的两关节角存储在Angle【0】 
	Angle[0].y=Input_Angle.y*M_PI/180;								//得到的世界坐标系的点存储在 point_World
	point_World.x=length1*cos(Angle[0].x)+length2*cos(Angle[0].y);  
	point_World.y=length1*sin(Angle[0].x)+length2*sin(Angle[0].y);	
}
void Solver::Neg_calculation(double l1,double l2,Point Input_Point)
{	point_World.x=Input_Point.x;
	point_World.y=Input_Point.y;
	double x=point_World.x;
	double y=point_World.y;
	double m,delt;
	double result[2][2]={2*M_PI,2*M_PI,2*M_PI,2*M_PI};      //第一个下标0--关节1,1--关节2 
	double is_boor[2]={0,0};
	double temp[2][4];										//第一个下标0--关节1的临时变量，1--关节2的临时变量 
	int i,j;
	m=(x*x+y*y+l2*l2-l1*l1)/(2*l2);
	delt=pow(y,4)+(x*x-m*m)*y*y;
	if(delt<0)
		number=0;
	else 
		{	
			temp[1][0]=(m*x+sqrt(delt))/(x*x+y*y);      // 此时，若number>=0，则关节角1有两种可能,重根视为2个；可能的关节角为  0，1，                
			temp[1][2]=(m*x-sqrt(delt))/(x*x+y*y);       // 则temp[0][]依次存储关节1的cos0，sin0，cos1，sin1的计算值，此时可能大于1
			if(y==0){                                    // temp[1][]依次存储关节2的 cos0，sin0，cos1，sin1的计算值，
				temp[0][0]=(x-l2*temp[1][0])/l1;
				temp[0][2]=(x-l2*temp[1][2])/l1;
				if (fabs(temp[1][0])<=1){
					result[1][0]=acos(temp[1][0]);
					result[1][1]=-acos(temp[1][2]);
				}
				if (fabs(temp[0][0])<=1){
					result[0][0]=-acos(temp[0][0]);
					result[0][1]=acos(temp[0][2]);
				}		
			}
			else{
				temp[1][1]=(m-x*temp[1][0])/y;
				temp[1][3]=(m-x*temp[1][2])/y;
				for(i=0;i<4;i+=2){
					temp[0][i]=(x-l2*temp[1][i])/l1;
					temp[0][i+1]=(y-l2*temp[1][i+1])/l1;
				}	
				for(j=0;j<2;j++){
					for(i=0;i<4;i+=2){
						if(fabs(temp[j][i])<=1)					// 对函数取反余弦函数 取值范围 0,π 但关节角的活动范围为 负180-180；因此可以根据sin 值的正负确定关节角的 
							if(temp[j][i+1]<0)					//正负，result 2依次存储关节角2 的两种可能角度  0,1 ，但是也有可能不存在，此时是默认值 2*M_PI
								result[j][i/2]=-acos(temp[j][i]);  //例如 可能角0不存在，则result2【0】= 2*M_PI
					        else 
								result[j][i/2]=acos(temp[j][i]);
					}
				}	
		    }
			for(i=0;i<2;i++){
				if ((fabs(result[0][i])<=M_PI)&&(fabs(result[1][i])<=M_PI)){
					is_boor[i]=1;					// 若 (fabs(result[0][i])<=M_PI)&&(fabs(result[1][i])<=M_PI)为真，说明关节角1,2的可能角i--i 		
					number++;						//	均存在，注：可能会出现一个小于M_PI，一个大于，这样也即是说不能同时存在i-i满足方程 		
				}									//这样也相对于方程组无界；is_boor[i]=1表示i--i存在 ，即方程组有解，能找到关节角1,2满足方程组 
				result[0][i]= result[0][i]*180/M_PI; 
				result[1][i]= result[1][i]*180/M_PI;
			}
			if(number==1){
				if (is_boor[0]==1){									//若number=1，说明只有一个解，那么 is_boor只有一个为1，将该数据赋值给Angle【0】，Angle【1】为默认值 
					Angle[0].x=result[0][0];
					Angle[0].y=result[1][0];
				}
				else{
					Angle[0].x=result[0][1];
					Angle[0].y=result[1][1];
				}
			}		
			else
				if (number==2){
					for(i=0;i<2;i++){	
						Angle[i].x=result[0][i];								//number=2时，两个是解，赋值给Angle【0】【1】 
						Angle[i].y=result[1][i];
					}
					if((Angle[0].x==Angle[1].x)&&(Angle[0].y==Angle[1].y))		//若两解相同，令number=1； 
						number=1;												//最终的效果  无解，Angle【0】【1】为默认值 number=0 
				}															//            只有一解，Angle【0】为解，Angle【1】为默认值 number=1 
  	}																	//            有两相同解，Angle【0】【1】均为解 number=1 
}
