#ifndef _FRAME_HPP_
#define _FRAME_HPP_

#include "Eigen/Dense"
#include <iostream>
#include "Point.hpp"

using namespace std;

class Frame{
    private:
        Point origin;
        double degree;
    public:
        Frame(){}
        Frame(Point orig,double deg){
            origin=orig;
            degree=deg;
         }
        Point getPoint(){
            return origin;
        }
        double getDegree(){
            return degree;
        }
         
};

#endif 
