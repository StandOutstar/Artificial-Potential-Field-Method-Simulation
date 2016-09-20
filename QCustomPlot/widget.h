#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>


namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    double PI;
    explicit Widget(QWidget *parent = 0,double pi = 3.1416);
    ~Widget();
    //吸引力计算
    double Compute_Attract(double current[2], double goal[2], double attract_gain, double Fa[2], double *aptr);
    //斥力计算
    void Compute_Repulsion(double r, double current[2], const QVector<double> &obs_x, const QVector<double> &obs_y, int Obs_Number, double repulse_gain, double Po, double Fr[2], double *rptr);
    //计算路径,返回值是路径点数
    int Compute_Road(double start[2], double goal[2], const QVector<double> &obs_x, const QVector<double> &obs_y);
    //符号函数
    int sign(double x);
    //仿真入口
    void setDemo();
private:
    Ui::Widget *ui;
};

#endif // WIDGET_H
