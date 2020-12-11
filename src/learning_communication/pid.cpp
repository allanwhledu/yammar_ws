#include <iostream>
#include <math.h>


/////////////////////////
using namespace std;
double G_targetLocation=0.2;	//目标追踪位置
double y = 0.0;	//目标当前位置
float Kp=0.3713585, Ki=0.017385, Kd=1.94784;



int main()
{
	double error_p=0;		//kp误差
	double error_i = 0;		//ki误差
	double error_d = 0;		//kd误差
	double error_dp = 0;	//上一次的kp误差、用于计算kd误差
    double w=0;
	 double theta=0;

	error_dp= G_targetLocation - y;	//整个系统第一次计算kd时、 e(k)-e(k-1)、
													//error_dp表示e(k-1)
	while (1) {
		error_p = G_targetLocation - y;
		error_i += error_p;
		error_d = error_p - error_dp;
		error_dp = error_p;
		w += Kp * error_p + Ki * error_i + Kd * error_d;
		theta += w;
		y =sin(theta);
		cout << "当前位置为：" << y << "当前的误差为" << G_targetLocation - y<<endl;
		//system("pause");
	} 
	//system("pause");
}