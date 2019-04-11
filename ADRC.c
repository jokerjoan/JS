#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#include"ADRC.h"

TD_data TDdata;
ESO_data ESOdata;
NLSEF_data NLSEFdata;

float init_adrc()//��ʼ�����������е��������
{

TDdata.x1 = 0.0; TDdata.x2 = 0.0;

ESOdata.z1 = 0.0; ESOdata.z2 = 0.0; ESOdata.z3 = 0.0; ESOdata.u1 = 0.0; ESOdata.u2 = 0.0;

NLSEFdata.u = 0.0; NLSEFdata.u0 = 0.0; NLSEFdata.e1 = 0.0; NLSEFdata.e2 = 0.0;
}


void TD(float expect)//expectΪĿ��Ƕ�
{
	
	float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0 , u=0 ;
	
	//fhan����
	d = TDdata.r * TDdata.h0 * TDdata.h0;
	a0 = TDdata.h0 *(TDdata.x2);
	y = TDdata.x1-expect + a0;
	a1 = sqrt(d*(d + 8 * abs(y)));
	a2 = a0 + sign(y)*(a1 - d) / 2;
	a = (a0 + y)*fsg(y, d) + a2 * (1 - fsg(y, d));
	u = -TDdata.r * (a / d)*fsg(a, d) - TDdata.r * sign(a)*(1 - fsg(a, d));

	//����΢����������td��
	TDdata.x1 += TDdata.h * TDdata.x2;
	TDdata.x2 += TDdata.h * u;
}

void ESO(float sysout)//sysoutΪ�ɻ�ģ�����
{
	float e = 0.0;
	ESOdata.u1 = NLSEFdata.u;//��NLSEF�������ΪESO�ĵ�һ��������
	ESOdata.u2 = sysout;//���ɻ�ģ�������ΪESO�ĵڶ���������
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
	NLSEFdata.u = (NLSEFdata.u0 - ESOdata.z3)/ NLSEFdata.b;  //Ϊϵͳ��Ŀ����ٶ�
}
//����������λ��.h�ļ���
//���밴��˳������Ŀ��Ƕȣ�expect�����㣩��ϵͳ��ǰ�Ƕȣ�sysout�����㣩��
//����ʱ�䣨T�����������ӣ�R�����˲����ӣ�H0��
//ESO�������Σ�bet01~bet03��eso_b,gam2,gam3��
//NLSEF�������Σ�bet1,bet2,d,nlsef_b��
//������˻�����ģ��һ���о�
//����ģ����Ҫ������calculate.m�ļ���Ȼ���������ģ��
//����ȫ�����Խ���֮�󣬿���ֻ����ǰ�������룬������������Ӧ����
//�������ֵΪ������˳�򣬴�r��ʼ��TD[30, 0.2] ESO [5, 1, 35, 15, 10, 5] NLSEF [5, 5, 0.01, 15]
float ADRC_Controller(float expect,float sysout, float T, float R, float H0, float bet01,float bet02, float bet03, float eso_b, float gam2, float gam3, float bet1, float bet2, float d, float nlsef_b)
{
	//��ʼ�����������н����������������ʼ��Ϊ0
	init_adrc();
	TDdata.h = T;//��ʾ��������
	TDdata.h0 = H0; //Ϊ���ٳ���������h�ı�����Ϊfhan�������Ʋ���
	TDdata.r = R;

	ESOdata.beta1 = bet01; ESOdata.beta2 = bet02; ESOdata.beta3 = bet03;
	ESOdata.gama2 = gam2; ESOdata.gama3 = gam3; ESOdata.b = eso_b;

	NLSEFdata.d = d; NLSEFdata.b = nlsef_b;
	NLSEFdata.bet1 = bet1; NLSEFdata.bet2 = bet2;

	/*****
	���Ź��ȹ��̣�����Ϊ����������
	��TD����΢�����õ���
	���������ź�x1����������΢���ź�x2
	******/
	TD(expect);
	/*****
	����״̬�۲������õ������źŵ�����״̬��
	1���Ƕ��ź�z1��
	2�����ٶ��ź�z2��
	3���Ǽ��ٶ��ź�z3��
	����z1��z2������Ϊ״̬������TD΢�ָ������õ���x1,x2�����
	���������Ժ���ӳ�䣬����betaϵ����
	��ϵõ�δ����״̬���ٶȹ����Ŷ�������ԭʼ������u
	*********/
	ESO(sysout);

	//��������ϼ���ó���NLSEFdata.u Ϊϵͳ��Ŀ����ٶȡ�
	NLSEF();

	return NLSEFdata.u;
}
