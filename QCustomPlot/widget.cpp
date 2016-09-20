#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <QtCore/qmath.h>

Widget::Widget(QWidget *parent,double pi) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    PI = pi;
    setDemo();

}

Widget::~Widget()
{
    delete ui;
}

//吸引力计算
double Widget::Compute_Attract(double current[2], double goal[2], double attract_gain, double Fa[2], double *aptr)
{
    double r;//距离暂存
    double deltax,deltay;//差值暂存
    double angle_a;//角度暂存

    Fa[0] = 0;
    Fa[1] = 0;

    //计算距离
    deltax = goal[0] - current[0];
    deltay = goal[1] - current[1];
    r = sqrt(deltax*deltax + deltay*deltay);

    //计算角度
    //angle_a = sign(deltay) * acos(deltax/r);

    angle_a = atan2(deltay,deltax);

    *aptr = angle_a;

    //计算分量
    Fa[0] = attract_gain * r * cos(angle_a);
    Fa[1] = attract_gain * r * sin(angle_a);

    return r;
}
//斥力计算
void Widget::Compute_Repulsion(double r_g, double current[2], const QVector<double> &obs_x, const QVector<double> &obs_y, int Obs_Number, double repulse_gain, double Po, double Fr[2], double *rptr)
{
    double deltax,deltay;//差值暂存
    double r;//距离暂存
    double temp_x,temp_y;//当前坐标暂存
    double Fsum;
    double frtx,frty;
    double obs_distance[Obs_Number];
    double obs_angle[Obs_Number];

    temp_x = current[0];
    temp_y = current[1];

    Fr[0] = 0;
    Fr[1] = 0;

    //计算角度和距离
    for (int i = 0; i < Obs_Number; ++i)
    {
        /* code */
        deltax = obs_x[i] - temp_x;
        deltay = obs_y[i] - temp_y;

        r = sqrt(deltax*deltax + deltay*deltay);

        obs_distance[i] = r;

        //obs_angle[i] = sign(deltay)*acos(deltax/r);

        obs_angle[i] = atan2 (deltay, deltax);
    }

    r_g = 1;//可以通过此处控制是否加入改进

    //计算斥力
    for (int i = 0; i < Obs_Number; ++i)
    {
        /* code */
        if (obs_distance[i] > Po)
        {
            /* code */
            frtx = 0;
            frty = 0;
        }
        else
        {
            //此处进行了改进，增加了当前位置到目标点距离这个影响因素。
            Fsum = - repulse_gain * r_g *(1/obs_distance[i] - 1/Po) * (1 / (obs_distance[i]*obs_distance[i]));//这个负号是因为斥力是由障碍物指向物体的，而计算出的角度是由物体指向障碍物的。
            frtx = Fsum * cos(obs_angle[i]);
            frty = Fsum * sin(obs_angle[i]);
        }
        Fr[0] += frtx;
        Fr[1] += frty;
    }

    *rptr = atan2(-Fr[1],-Fr[0]);
}

//计算路径,返回值是路径点数
int Widget::Compute_Road(double start[2], double goal[2], const QVector<double> &obs_x, const QVector<double> &obs_y)
{
    //引力增益系数
    double attract_gain = 7;
    //斥力增益系数
    double repulse_gain = 7;
    //障碍物影响距离
    double Po = 3;
    //障碍物个数
    double Obs_Number = obs_x.size ();
    //步长
    double StepLength = 0.2;
    //循环迭代最大次数
    int max_circletime = 200;

    int s = 0;//存储最后路径点数
    double position_angle;
    double xnext,ynext;
    double current_point[2];

    double FsumX,FsumY;
    double Fa[2],Fr[2];

    double r_temp;//存储当前位置到目标点距离

    double Fa_angle;
    double Fr_angle;

    double *faangle_ptr = &Fa_angle;
    double *frangle_ptr = &Fr_angle;

    current_point[0] = start[0];
    current_point[1] = start[1];

    //定义两个可变数组存放绘图的坐标数据
    QVector<double> road_x(200), road_y(200);

    for(int i = 0; i < max_circletime; ++i)
    {
        //记录路径点
        road_x[i] = current_point[0];
        road_y[i] = current_point[1];

        //吸引力计算
        r_temp = Compute_Attract(current_point, goal, attract_gain, Fa, faangle_ptr);
        //斥力计算
        Compute_Repulsion(r_temp, current_point, obs_x, obs_y, Obs_Number, repulse_gain, Po, Fr, frangle_ptr);

        //计算合力
        FsumX = Fa[0] + Fr[0];
        FsumY = Fa[1] + Fr[1];

        //计算合力角度
        position_angle = atan2(FsumY,FsumX);//发现atan2更好用。返回值是-π~π；

        //通过此处可以控制是否改进障碍物在起始与目标点之间的情况
        if((Fa_angle == Fr_angle) && (FsumY < 0.0001) && (FsumX < 0.0001))
            position_angle += 0.1;//施加一个扰动（逃逸力）

        //计算下一路径点
        xnext = current_point[0] + StepLength*cos(position_angle);
        ynext = current_point[1] + StepLength*sin(position_angle);

        //更新当前点
        current_point[0] = xnext;
        current_point[1] = ynext;

        s = i;

        //判断是否到达目标点
        if ((qAbs(goal[0] - current_point[0]) < 0.2) && (qAbs(goal[1] - current_point[1]) < 0.2))
        {
            /* code */
            s += 2;
            road_x[i+1] = current_point[0];
            road_y[i+1] = current_point[1];

            road_x[i+2] = goal[0];
            road_y[i+2] = goal[1];

            break;
        }
    }

    //定义两个可变数组存放绘图的坐标数据
    QVector<double> road_xx(s+1), road_yy(s+1);
    for(int i = 0; i < s+1; ++i)
    {
        road_xx[i] = road_x[i];
        road_yy[i] = road_y[i];
    }
    //绘制路径
    // create empty curve objects and add them to customPlot:
    QCPCurve *road = new QCPCurve(ui->qCustomPlot->xAxis, ui->qCustomPlot->yAxis);
    ui->qCustomPlot->addPlottable(road);

    // pass the data to the curves:
    road->setData(road_xx, road_yy);
    // color the curves:
    road->setPen(QPen(Qt::black));
    road->setName ("Road");
    //road->setBrush(QBrush(QColor(0, 0, 255, 20)));

//    //绘制路径
//    //向绘图区域QCustomPlot(从widget提升来的)添加一条曲线
//    ui->qCustomPlot->addGraph();

//    //添加数据
//    ui->qCustomPlot->graph(3)->setData(road_xx,road_yy);
//    ui->qCustomPlot->graph(3)->setPen(QPen(Qt::black));
//    ui->qCustomPlot->graph(3)->setLineStyle(QCPGraph::lsLine );
//    ui->qCustomPlot->graph(3)->setScatterStyle(QCPScatterStyle::ssNone);
//    ui->qCustomPlot->graph(3)->setName("Road");

    //设置坐标轴标签名称
    ui->qCustomPlot->xAxis->setLabel("x");
    ui->qCustomPlot->yAxis->setLabel("y");

    //设置坐标轴显示范围,否则我们只能看到默认的范围
    ui->qCustomPlot->xAxis->setRange(0,10);
    ui->qCustomPlot->yAxis->setRange(0,10);

    //关闭自动刻度步长
    ui->qCustomPlot->xAxis->setAutoTickStep(false);
    ui->qCustomPlot->yAxis->setAutoTickStep(false);
    //关闭自动子刻度
    ui->qCustomPlot->xAxis->setAutoSubTicks(false);
    ui->qCustomPlot->yAxis->setAutoSubTicks(false);
    //设置主刻度步长1
    ui->qCustomPlot->xAxis->setTickStep (1);
    ui->qCustomPlot->yAxis->setTickStep (1);
    //设置子刻度数4，即分成5份
    ui->qCustomPlot->xAxis->setSubTickCount (4);
    ui->qCustomPlot->yAxis->setSubTickCount (4);
    //设置刻度长度5
    ui->qCustomPlot->xAxis->setTickLength (5, 0);
    ui->qCustomPlot->yAxis->setTickLength (5, 0);

//    //重绘，这里可以不用，官方例子有，执行setData函数后自动重绘
//    //我认为应该用于动态显示或者是改变坐标轴范围之后的动态显示，我们以后探索
//    ui->qCustomPlot->replot();

    qDebug()<<"步数="<<s;
    qDebug()<<"路径点数="<<road_xx.size ();
    qDebug()<<"障碍物数量="<<Obs_Number;
    qDebug()<<"π的取值"<<PI;

    if(s == 199)
        qDebug()<<"目标不可达";
    else
        qDebug()<<"目标可达";
    return s;
}
int Widget::sign(double x)//此函数弃用了，这是之前计算角度时用的，后来用atan2了
{
    if(x > 0)
        return 1;
    else if(x == 0)
        return 0;
    else if(x < 0)
        return -1;
    else
        return 1;
}
void Widget::setDemo()
{
    //QVector<double> start(2),goal(2);
    //start<<0<<0;
    //goal<<10<<10;
    double start[2] = {1, 1};
    double goal[2] = {9, 9};
    //障碍物坐标
    //double Obstacle[7][2] = {{1,1}, {3,2.5}, {4,4.5}, {3,6}, {6,2}, {5,5.5}, {9,9}};

    //绘制起点
    QVector<double> start_x,start_y;
    start_x<<start[0];
    start_y<<start[1];
    //向绘图区域QCustomPlot(从widget提升来的)添加一条曲线
    ui->qCustomPlot->addGraph();
    //添加数据
    ui->qCustomPlot->graph(0)->setData(start_x,start_y);
    ui->qCustomPlot->graph(0)->setPen(QPen(Qt::darkGreen));
    ui->qCustomPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->qCustomPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->qCustomPlot->graph(0)->setName("Start");


    //绘制障碍物点
    QVector<double> obs_x;
    obs_x<<5;

    QVector<double> obs_y;//定义坐标
    obs_y<<5;
//    QVector<double> obs_x;
//    obs_x<<3<<6<<8;

//    QVector<double> obs_y;//定义坐标
//    obs_y<<4<<3<<6;

//    QVector<double> obs_x;
//    obs_x<<1<<2.5<<4<<6;

//    QVector<double> obs_y;//定义坐标
//    obs_y<<2<<1<<5<<4;

    //向绘图区域QCustomPlot(从widget提升来的)添加一条曲线
    ui->qCustomPlot->addGraph();
    //添加数据
    ui->qCustomPlot->graph(1)->setData(obs_x,obs_y);
    ui->qCustomPlot->graph(1)->setPen(QPen(Qt::red));
    ui->qCustomPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->qCustomPlot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->qCustomPlot->graph(1)->setName("Obstacle");


    //绘制目标点
    QVector<double> goal_x,goal_y;
    goal_x<<goal[0];
    goal_y<<goal[1];
    //向绘图区域QCustomPlot(从widget提升来的)添加一条曲线
    ui->qCustomPlot->addGraph();
    //添加数据
    ui->qCustomPlot->graph(2)->setData(goal_x,goal_y);
    ui->qCustomPlot->graph(2)->setPen(QPen(Qt::blue));
    ui->qCustomPlot->graph(2)->setLineStyle(QCPGraph::lsNone);
    ui->qCustomPlot->graph(2)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->qCustomPlot->graph(2)->setName("Goal");

    //开启图例显示
    ui->qCustomPlot->legend->setVisible(true);//true false
    ui->qCustomPlot->legend->setFont(QFont("Helvetica", 9));

    //重设图例位置为左上
    ui->qCustomPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);

    Compute_Road(start, goal, obs_x,obs_y);


}
