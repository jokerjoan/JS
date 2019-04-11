#pragma once
#ifndef _ADRC_H
#define _ADRC_H


typedef struct
{
	float h; //采样的频率 单位为s
			 //TD  得到好的结果之后，本部分的参数可以不再修改
	float x1;//输出1  角度 单位为°
	float x2;//输出2  跟踪的角速度  单位为°/s
	float r; //（快速因子）参数越大，跟踪速度越快，一般与h0保持固定比例      参考数值为12
	float h0;//（滤波因子）需调试的参数 确定之后若h不修改，本参数也可不修改  参考数值为0.05
}TD_data;

typedef struct
{
	//ESO  主要通过修改beta1-beta3来修改效果
	float z1;//输出1 角度跟踪估计值
	float z2;//输出2 角速度跟踪估计值
	float z3;//输出3 扰动扩张成的状态量
	float beta1;//参数beta  控制z1的输出响应 
	float beta2;//          控制z2的输出响应
	float beta3;//          控制z3的输出响应
	float b;//参数b     与扰动相关，根据系统的外部扰动值来设定，越准确，eso效果越好，该值与NLSEF中的b（nlsef_b）保持一致时，效果好
	float gama2;//参数gama，修改z2中反双曲正弦函数的输入来改变z2的输出响应   在仿真过程中，确定gama2、3之后，尽量不修改。依靠修改beta参数来改善控制效果
	float gama3;//          修改z3中反双曲正弦函数的输入来改变z3的输出响应   
	float u1;//输入1 为NLSEF的输出量 即角速度量
	float u2;//输入2 为无人机当前的角度（作为反馈输入进ESO）
}ESO_data;

typedef struct
{
	//NLSEF 
	float aa1;//固定值，确定为0.75
	float aa2;//固定值，确定为1.25
	float bet1;//非线性组合中角度误差前的比例项
	float bet2;//非线性组合中角速度误差前的比例项
	float e1;//e1=x1-z1  即为角度的误差
	float e2;//e2=x2-z2  即为角速度的误差
	float u0;//非线性组合计算出来的角速度值
	float d;//常设置为0  或者一个小值
	float b;
	float u;//最终输出信号 即为无人机的目标角速度值  为u0加上补偿的外部扰动分量之后的值
}NLSEF_data;


short int sign(float input)//td中的sign函数 即符号函数
{
	short int output = 0;
	if (input > 1e-6)
		output = 1;
	else if (input < 1e-6)
		output = -1;
	else output = 0;
	return output;
}

short int fsg(float x, float d)//td中的fsg函数
{
	short int output = 0;
	output = (sign(x + d) - sign(x - d)) / 2;
	return output;
}
float ASINH(float x)//改进后eso中的反双曲正弦函数
{
	float output = 0;
	output = log(x + sqrt(x*x + 1));
	return output;
} 
float fal(float e, float a, float d)  //NLSEF中的函数
{
	float f=0;
	if (abs(e) < d)
		f = e * pow(d, (a - 1));
	else
		f = pow(abs(e), a)*sign(e);
	return f;
}
#endif // !_TD_H
