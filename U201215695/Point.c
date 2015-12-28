 #include"Point.h" 
  #include<iostream>
  
  Point::Point(double a,double b){
     this->x=a;
     this->y=b;
 }

 void Point::updatePoint(double a,double b){
     this->x=a;
     this->y=b;    
 }
