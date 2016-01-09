#ifndef SOLVER
#define SOLVER
#include "Frame.h"
#include "Point.h"
class Solver{
    public:
        void WtoT(TaskFrame t,Point &p);
        void TtoW(TaskFrame t,Point &p);
        void TtoJ(Point &p,double a1,double a2);
        void JtoT(Point &p,double a1,double a2);
};
#endif
