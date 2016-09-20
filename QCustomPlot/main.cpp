#include "widget.h"
#include <QApplication>
//#include "ARF_C++/arf.h"
#include <QDebug>
//#include <iostream>
//using namespace std;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;

    w.show();

    return a.exec();
}
//    //起点位置
//    Point Start(0, 0);
//    //障碍物个数
//    int Obs_Number = 7;
//    //目标和障碍物位置
//    Point Goal(10, 10);
//    Point Obstacle[7]= {
//        Point(1, 1),
//        Point(3, 2.5),
//        Point(4, 4.5),
//        Point(3, 6),
//        Point(6, 2),
//        Point(5, 5.5),
//        Point(9, 9),
//    };

//    APF_Model b(7, 7, 2, 0.2);
//    b.Compute_Road (Start,Goal,Obstacle,Obs_Number);
