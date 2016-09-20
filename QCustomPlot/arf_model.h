#ifndef ARF_MODEL_H
#define ARF_MODEL_H
#include <QtCore/qmath.h>

#define PI 3.1415


//吸引力计算
void Compute_Attract(double current[2], double goal[2], double Fa[2], double attract_gain);

//斥力计算
void Compute_Repulsion(double current[2], double obs[7][2], double Fr[2], int Obs_Number, double repulse_gain, double Po);

//计算路径,返回值是路径点数
int Compute_Road(double start[2], double goal[2], double obs[7][2], double road[200][2]);

int sign(double x);


#endif
