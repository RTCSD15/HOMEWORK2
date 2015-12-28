 #ifndef _FRAME_H_
 #define _FRAME_H_
  
 #include<Eigen/Dense>
 #include<cmath>
 #include"Point.h"
 using namespace Eigen;
 
 class Frame{
 	protected:
 		Point OrigialPoint,AimPoint;
 		double OrigialAngal;
 	public:
 		Frame(){
 			OrigialPoint.x=0;OrigialPoint.y=0;OrigialAngal=0.0;
		 }
 		Frame(Point OrigialPoint0,double OrigialAngal0=0){
 			this->OrigialPoint=OrigialPoint0;
 			this->OrigialAngal=OrigialAngal0;
		 }
		 void UpdateMatrix(Point Zero){
		 	AimPoint=Zero;
		 }
		 Point To_WF(Point twpoint);
 };
  
  
 class WorldFrame:public Frame{
 	public:
 		WorldFrame(){
 			OrigialPoint.x=0;OrigialPoint.y=0;OrigialAngal=0.0;
		 }
 		WorldFrame(Point OrigialPoint0,double OrigialAngal0=0);
 		~WorldFrame(){
		 }
 
 };
 
 
 
 class JointFrame:public Frame{
 		public:
 		JointFrame(){
 			OrigialPoint.x=0;OrigialPoint.y=0;OrigialAngal=0.0;
		 }
 		JointFrame(Point OrigialPoint0,double OrigialAngal0=0);
 		~JointFrame(){
		 }
 		

 };
 
 
 
 class TaskFrame:public Frame{
 		public:
 		TaskFrame(){
 			OrigialPoint.x=0;OrigialPoint.y=0;OrigialAngal=0.0;
		 }
 		TaskFrame(Point OrigialPoint0,double OrigialAngal0=0);
 		~TaskFrame(){
		 }

  
 };
 
 
 #endif
