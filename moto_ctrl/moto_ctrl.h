#ifndef _MOTO_C_H
#define _MOTO_C_H

#include "Public.h"

#define CAR_LEN 	2	//车身长度
#define Pi			3.141592f

//typedef struct _CAR_NUM
//{
//	float 	L,
//			
//}car_num;

enum wheel_name{LF,RF,LB,RB,	A_LF,A_RF,A_LB,A_RB,wheel_num};//Left / Right, Front / Behind
//enum steer_name{A_LF=4,A_RF,A_LB,A_RB};//各个方向轮编号，规则同上

typedef struct _MOVE_TARGET		//运动目标
{
	int16_t 	Vx,			//前后运动速度分量，向前为正
				Vy;			//左右运动速度分量，向右为正
//			ome;		//自转角速度
}move_target;

typedef struct _TANS_VALUE		//控制中间量
{
	float 	ome,		//自转角速度
			angle;		//整体运动方向角
//			Vl,			//仅旋转的线速度，好像不需要线速度了
}trans_value;

typedef struct _FINAL_OUT		//最终输出量
{
	float 		speed[wheel_num];			//M3508目标转速，进PID单环
	double		encoder[wheel_num];		//M2006目标机械角度，进PID双环
}final_out;



void moto_ctrl(move_target* move_t, final_out* opu);


#endif
