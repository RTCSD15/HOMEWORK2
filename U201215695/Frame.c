 #include"Frame.h"
 
 Point Frame::To_WF(Point twpoint){
      Matrix2d RotateMatrix;
      double OrigialAngal0=OrigialAngal/180*M_PI;
      Point wfpoint; 
      RotateMatrix<< cos(OrigialAngal0),sin(OrigialAngal0),
            -sin(OrigialAngal0),cos(OrigialAngal0);
      Vector2d v1(twpoint.x,twpoint.y),v2(OrigialPoint.x,OrigialPoint.y),v3;
      v3=RotateMatrix*v1+v2;
      wfpoint.updatePoint(v3(0),v3(1));
      return wfpoint;
 }
 
 WorldFrame::WorldFrame(Point OrigialPoint0,double OrigialAngal0):Frame(OrigialPoint0,OrigialAngal0){
 			this->OrigialPoint=OrigialPoint0;
 			this->OrigialAngal=OrigialAngal0;
		 }
		 
JointFrame::JointFrame(Point OrigialPoint0,double OrigialAngal0):Frame(OrigialPoint0,OrigialAngal0){
 			this->OrigialPoint=OrigialPoint0;
 			this->OrigialAngal=OrigialAngal0;
		 }
		
TaskFrame::TaskFrame(Point OrigialPoint0,double OrigialAngal0):Frame(OrigialPoint0,OrigialAngal0){
 			this->OrigialPoint=OrigialPoint0;
 			this->OrigialAngal=OrigialAngal0;
		 }
