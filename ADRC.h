#pragma once
#ifndef _ADRC_H
#define _ADRC_H


typedef struct
{
	float h; //������Ƶ�� ��λΪs
			 //TD  �õ��õĽ��֮�󣬱����ֵĲ������Բ����޸�
	float x1;//���1  �Ƕ� ��λΪ��
	float x2;//���2  ���ٵĽ��ٶ�  ��λΪ��/s
	float r; //���������ӣ�����Խ�󣬸����ٶ�Խ�죬һ����h0���̶ֹ�����      �ο���ֵΪ12
	float h0;//���˲����ӣ�����ԵĲ��� ȷ��֮����h���޸ģ�������Ҳ�ɲ��޸�  �ο���ֵΪ0.05
}TD_data;

typedef struct
{
	//ESO  ��Ҫͨ���޸�beta1-beta3���޸�Ч��
	float z1;//���1 �Ƕȸ��ٹ���ֵ
	float z2;//���2 ���ٶȸ��ٹ���ֵ
	float z3;//���3 �Ŷ����ųɵ�״̬��
	float beta1;//����beta  ����z1�������Ӧ 
	float beta2;//          ����z2�������Ӧ
	float beta3;//          ����z3�������Ӧ
	float b;//����b     ���Ŷ���أ�����ϵͳ���ⲿ�Ŷ�ֵ���趨��Խ׼ȷ��esoЧ��Խ�ã���ֵ��NLSEF�е�b��nlsef_b������һ��ʱ��Ч����
	float gama2;//����gama���޸�z2�з�˫�����Һ������������ı�z2�������Ӧ   �ڷ�������У�ȷ��gama2��3֮�󣬾������޸ġ������޸�beta���������ƿ���Ч��
	float gama3;//          �޸�z3�з�˫�����Һ������������ı�z3�������Ӧ   
	float u1;//����1 ΪNLSEF������� �����ٶ���
	float u2;//����2 Ϊ���˻���ǰ�ĽǶȣ���Ϊ���������ESO��
}ESO_data;

typedef struct
{
	//NLSEF 
	float aa1;//�̶�ֵ��ȷ��Ϊ0.75
	float aa2;//�̶�ֵ��ȷ��Ϊ1.25
	float bet1;//����������нǶ����ǰ�ı�����
	float bet2;//����������н��ٶ����ǰ�ı�����
	float e1;//e1=x1-z1  ��Ϊ�Ƕȵ����
	float e2;//e2=x2-z2  ��Ϊ���ٶȵ����
	float u0;//��������ϼ�������Ľ��ٶ�ֵ
	float d;//������Ϊ0  ����һ��Сֵ
	float b;
	float u;//��������ź� ��Ϊ���˻���Ŀ����ٶ�ֵ  Ϊu0���ϲ������ⲿ�Ŷ�����֮���ֵ
}NLSEF_data;


short int sign(float input)//td�е�sign���� �����ź���
{
	short int output = 0;
	if (input > 1e-6)
		output = 1;
	else if (input < 1e-6)
		output = -1;
	else output = 0;
	return output;
}

short int fsg(float x, float d)//td�е�fsg����
{
	short int output = 0;
	output = (sign(x + d) - sign(x - d)) / 2;
	return output;
}
float ASINH(float x)//�Ľ���eso�еķ�˫�����Һ���
{
	float output = 0;
	output = log(x + sqrt(x*x + 1));
	return output;
} 
float fal(float e, float a, float d)  //NLSEF�еĺ���
{
	float f=0;
	if (abs(e) < d)
		f = e * pow(d, (a - 1));
	else
		f = pow(abs(e), a)*sign(e);
	return f;
}
#endif // !_TD_H
