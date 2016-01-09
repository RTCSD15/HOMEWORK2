#include "Robot.h"
/* run this program using the console pauser or add your own getch, system("pause") or input loop */

int main(int argc, char** argv) {
    Robot myRobot(8,8);
    WorldFrame WF;
    TaskFrame TF1(0,0,0),TF2(1,2,30),TF3(3,5,90);
    JointFrame JF;
    Point P1(0,0),P2(1,3),P3(2,4),P4(3,6),P5(0,3);
    myRobot.PTPMove(JF,P1);
    myRobot.PTPMove(WF,TF1,P2);
    myRobot.PTPMove(TF1,P3);
    myRobot.PTPMove(TF2,P4);
    myRobot.PTPMove(TF3,P5);
    return 0;

}
