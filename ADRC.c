#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#include"ADRC.h"

TD_data TDdata;
ESO_data ESOdata;
NLSEF_data NLSEFdata;

float init_adrc()//初始化三个函数中的输入输出
{

TDdata.x1 = 0.0; TDdata.x2 = 0.0;

ESOdata.z1 = 0.0; ESOdata.z2 = 0.0; ESOdata.z3 = 0.0; ESOdata.u1 = 0.0; ESOdata.u2 = 0.0;

NLSEFdata.u = 0.0; NLSEFdata.u0 = 0.0; NLSEFdata.e1 = 0.0; NLSEFdata.e2 = 0.0;
}


void TD(float expect)//expect为目标角度
{
	
	float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0 , u=0 ;
	
	//fhan函数
	d = TDdata.r * TDdata.h0 * TDdata.h0;
	a0 = TDdata.h0 *(TDdata.x2);
	y = TDdata.x1-expect + a0;
	a1 = sqrt(d*(d + 8 * abs(y)));
	a2 = a0 + sign(y)*(a1 - d) / 2;
	a = (a0 + y)*fsg(y, d) + a2 * (1 - fsg(y, d));
	u = -TDdata.r * (a / d)*fsg(a, d) - TDdata.r * sign(a)*(1 - fsg(a, d));

	//跟踪微分器函数（td）
	TDdata.x1 += TDdata.h * TDdata.x2;
	TDdata.x2 += TDdata.h * u;
}

void ESO(float sysout)//sysout为飞机模型输出
{
	float e = 0.0;
	ESOdata.u1 = NLSEFdata.u;//将NLSEF的输出作为ESO的第一个输入量
	ESOdata.u2 = sysout;//将飞机模型输出作为ESO的第二个输入量
	e = ESOdata.z1 - ESOdata.u2;
	ESOdata.z1 += TDdata.h * (ESOdata.z2 - e * ESOdata.beta1);
	ESOdata.z2 += TDdata.h * (ESOdata.z3 - ESOdata.beta2*ASINH(ESOdata.gama2*e) + ESOdata.b*ESOdata.u1);
	ESOdata.z3 += TDdata.h * (-ESOdata.beta3*ASINH(ESOdata.gama3*e));
}
void NLSEF()
{	
	NLSEFdata.e1 = TDdata.x1 - ESOdata.z1;
	NLSEFdata.e2 = TDdata.x2 - ESOdata.z2;
	NLSEFdata.u0 = NLSEFdata.bet1*fal(NLSEFdata.e1,0.75, NLSEFdata.d) + NLSEFdata.bet2*fal(NLSEFdata.e2, 1.25, NLSEFdata.d);
	NLSEFdata.u = (NLSEFdata.u0 - ESOdata.z3)/ NLSEFdata.b;  //为系统的目标角速度
}
//参数的物理单位在.h文件中
//输入按照顺序来是目标角度（expect）（°），系统当前角度（sysout）（°），
//采样时间（T），快速因子（R），滤波因子（H0）
//ESO参数依次（bet01~bet03，eso_b,gam2,gam3）
//NLSEF参数依次（bet1,bet2,d,nlsef_b）
//配合无人机仿真模型一起研究
//仿真模型需要先运行calculate.m文件，然后才能运行模型
//参数全部调试结束之后，可以只保留前两个输入，用来测试自适应性能
//建议参数值为（按照顺序，从r开始）TD[30, 0.2] ESO [5, 1, 35, 15, 10, 5] NLSEF [5, 5, 0.01, 15]
float ADRC_Controller(float expect,float sysout, float T, float R, float H0, float bet01,float bet02, float bet03, float eso_b, float gam2, float gam3, float bet1, float bet2, float d, float nlsef_b)
{
	//初始化各参数还有将各函数输入输出初始化为0
	init_adrc();
	TDdata.h = T;//表示采样周期
	TDdata.h0 = H0; //为减少超调，采用h的倍数作为fhan函数控制参数
	TDdata.r = R;

	ESOdata.beta1 = bet01; ESOdata.beta2 = bet02; ESOdata.beta3 = bet03;
	ESOdata.gama2 = gam2; ESOdata.gama3 = gam3; ESOdata.b = eso_b;

	NLSEFdata.d = d; NLSEFdata.b = nlsef_b;
	NLSEFdata.bet1 = bet1; NLSEFdata.bet2 = bet2;

	/*****
	安排过度过程，输入为期望给定，
	由TD跟踪微分器得到：
	过度期望信号x1，过度期望微分信号x2
	******/
	TD(expect);
	/*****
	扩张状态观测器，得到反馈信号的扩张状态：
	1、角度信号z1；
	2、角速度信号z2；
	3、角加速度信号z3。
	其中z1、z2用于作为状态反馈与TD微分跟踪器得到的x1,x2做差后，
	经过非线性函数映射，乘以beta系数后，
	组合得到未加入状态加速度估计扰动补偿的原始控制量u
	*********/
	ESO(sysout);

	//非线性组合计算得出的NLSEFdata.u 为系统的目标角速度。
	NLSEF();

	return NLSEFdata.u;
}
