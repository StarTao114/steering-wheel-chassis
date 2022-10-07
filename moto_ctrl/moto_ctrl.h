#ifndef _MOTO_C_H
#define _MOTO_C_H

#include "Public.h"

#define CAR_LEN 	2	//������
#define Pi			3.141592f

//typedef struct _CAR_NUM
//{
//	float 	L,
//			
//}car_num;

enum wheel_name{LF,RF,LB,RB,	A_LF,A_RF,A_LB,A_RB,wheel_num};//Left / Right, Front / Behind
//enum steer_name{A_LF=4,A_RF,A_LB,A_RB};//���������ֱ�ţ�����ͬ��

typedef struct _MOVE_TARGET		//�˶�Ŀ��
{
	int16_t 	Vx,			//ǰ���˶��ٶȷ�������ǰΪ��
				Vy;			//�����˶��ٶȷ���������Ϊ��
//			ome;		//��ת���ٶ�
}move_target;

typedef struct _TANS_VALUE		//�����м���
{
	float 	ome,		//��ת���ٶ�
			angle;		//�����˶������
//			Vl,			//����ת�����ٶȣ�������Ҫ���ٶ���
}trans_value;

typedef struct _FINAL_OUT		//���������
{
	float 		speed[wheel_num];			//M3508Ŀ��ת�٣���PID����
	double		encoder[wheel_num];		//M2006Ŀ���е�Ƕȣ���PID˫��
}final_out;



void moto_ctrl(move_target* move_t, final_out* opu);


#endif
