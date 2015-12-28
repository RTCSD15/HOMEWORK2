#include<iostream>
#include"Point.h"
using namespace std; 
Point::Point(double a,double b){
	x=a;
	y=b;
}
void Point::SetP(double a,double b){
	x=a;
	y=b;
}
void Point::PrintP(){
	cout<<'('<<x<<','<<y<<')'<<endl;
}
