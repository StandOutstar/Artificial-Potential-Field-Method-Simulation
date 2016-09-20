#include "arf_model.h"

//吸引力计算
void Compute_Attract(double current[2], double goal[2], double attract_gain, double Fa[2])
{
	double r;//距离暂存
	double deltax,deltay;//差值暂存
	double angle_a;//角度暂存

	//计算距离
	deltax = goal[0] - current[0];
	deltay = goal[1] - current[1];
	r = sqrt(deltax*deltax + deltay*deltay);

	//计算角度
	angle_a = sign(deltay) * acos(deltax/r);

	//计算分量
	Fa[0] = attract_gain * r * cos(angle_a);
	Fa[1] = attract_gain * r * sin(angle_a);
}

//斥力计算
void Compute_Repulsion(double current[2], double obs[7][2], int Obs_Number, double repulse_gain, double Po, double Fr[2])
{
	double deltax,deltay;//差值暂存
	double r;//距离暂存
	double temp_x,temp_y;//当前坐标暂存
	double Fsum;
	double frtx,frty;
	double obs_distance[7];
	double obs_angle[7];

	temp_x = current[0];
	temp_y = current[1];

	//计算角度和距离
	for (int i = 0; i < Obs_Number; ++i)
	{
		/* code */
		deltax = obs[i][0] - temp_x;
		deltay = obs[i][1] - temp_y;

		r = sqrt(deltax*deltax + deltay*deltay);

		obs_distance[i] = r;

		obs_angle[i] = sign(deltay)*acos(deltax/r);	 
	}

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
            Fsum = -repulse_gain * (1/obs_distance[i] - 1/Po) * (1 / (obs_distance[i]*obs_distance[i]));
			frtx = Fsum * cos(obs_angle[i]);
			frty = Fsum * sin(obs_angle[i]);
		}

		Fr[0] += frtx;
		Fr[1] += frty;
	}
}

//计算路径,返回值是路径点数
int Compute_Road(double start[2], double goal[2], double obs[7][2], double road[200][2])
{


	//引力增益系数
	double attract_gain = 7;

	//斥力增益系数
	double repulse_gain = 7;
	//障碍物影响距离
	double Po = 2;
	//障碍物个数
	double Obs_Number = 7;

	//步长
	double StepLength = 0.2;

	//循环迭代最大次数
	int max_circletime = 200;

	int s = 0;//存储最后路径点数
	double position_angle;
	double xnext,ynext;
	double current_point[2];
//	double last_point[2];
	double FsumX,FsumY;
	double Fa[2],Fr[2];

	current_point[0] = start[0];
	current_point[1] = start[1];

	for(int i = 0; i < max_circletime; ++i)
	{


		//吸引力计算
		Compute_Attract(current_point, goal, attract_gain, Fa);
		//斥力计算
		Compute_Repulsion(current_point, obs, Obs_Number, repulse_gain, Po, Fr);

		//计算合力
		FsumX = Fa[0] + Fr[0];
		FsumY = Fa[1] + Fr[1];

		//计算合力角度
		if (FsumX > 0)
		{
			/* code */
			position_angle = atan(FsumY/FsumX);
		}
		else
		{
			position_angle = PI + atan(FsumY/FsumX);
		}

		//计算下一路径点
		xnext = current_point[0] + StepLength*cos(position_angle);
		ynext = current_point[1] + StepLength*sin(position_angle);

//		//更新坐标
//		last_point[0] = current_point[0];
//		last_point[1] = current_point[1];

		current_point[0] = xnext;
		current_point[1] = ynext;	

		//记录路径点
		road[i][0] = current_point[0];
		road[i][1] = current_point[1];

		s = i;

		//判断是否到达目标点
		if ((abs(goal[0] - current_point[0]) < 1) && (abs(goal[1] - current_point[1]) < 1))
		{
			/* code */
			break;
		}			
	}


	return s;
}
int sign(double x)
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

