#include "moto_ctrl.h"

const u8 M2006_REDUCT_RATIO_MOTO =  36;				//2006自带减速箱的减速比例
const u8 M2006_REDUCT_RATIO_MOTO_TO_GEAR =  6;		//2006传动杆与车轮云台的减速比

const u8 M3508_REDUCT_RATIO_MOTO =  19;				//同上
const u8 M3508_REDUCT_RATIO_MOTO_TO_GEAR =  2;

//这次用矢量叠加
void moto_ctrl(move_target* move_t, final_out* opt)
{
	u8 i=0;
	
//	if(move_t->Vy < 0)								//如果试图后退，则将速度取反，电机反转
//		move_t->Vx = -move_t->Vx;
	
	for(i=0;i<4;i++)
	{
		opt->speed[i] = M3508_REDUCT_RATIO_MOTO_TO_GEAR * M3508_REDUCT_RATIO_MOTO * sqrt(move_t->Vx * move_t->Vx + move_t->Vy * move_t->Vy);
		
		if(move_t->Vy < 2 && move_t->Vy > -2)
			move_t->Vy = 1;
		
		if(move_t->Vy > 0)
			opt->encoder[i] = M2006_REDUCT_RATIO_MOTO_TO_GEAR * M2006_REDUCT_RATIO_MOTO /2.0f * 8191 * atan((float)move_t->Vx / (float)move_t->Vy) / Pi;		//整体旋转角
		if(move_t->Vy < 0)
			opt->encoder[i] = M2006_REDUCT_RATIO_MOTO_TO_GEAR * M2006_REDUCT_RATIO_MOTO /2.0f * 8191 * ( Pi - atan((float)move_t->Vx / (float)move_t->Vy) ) / Pi;		//整体旋转角

		if(move_t->Vy < 0)								//如果试图后退，则将速度取反，电机反转
			opt->speed[i]=-opt->speed[i];
	}
	
	
}


//u16 DR16_Vx;
//u16 DR16_Vy;//假装我有遥控器的返回值

//转弯半径可考虑改进为动态，(已改为变量turn_radius)
//e.g.转弯角度越大，转弯半径越小，
//当转弯半径->0时，小车自转
//void moto_ctrl(move_target* move_t, final_out* opt)
//{
//	volatile float velo=0;					//整体运动合速度
//	volatile trans_value trans={0,0};
//	volatile float Radius[4]={0};		//每个轮子的运动半径
//	volatile float turn_radius=0;				//整体转弯半径=200/trans.angle（暂定）
//	u8 i=0;
//	
//	velo=sqrt(move_t->Vx * move_t->Vx + move_t->Vy * move_t->Vy);
//	if(move_t->Vy != 0)
//		trans.angle=atan(move_t->Vx / move_t->Vy);		//整体旋转角
//	else if(move_t->Vy <= 0.2f)
//		trans.angle=3.141592f / 2.0f;
//	turn_radius=3/trans.angle;							//旋转半径参数待调整
//	trans.ome=velo / turn_radius;					//整体角速度
//	
//	Radius[LF]=pow((CAR_LEN/2 + turn_radius*(float)sin(trans.angle)),2) + pow((CAR_LEN/2 + turn_radius*(float)cos(trans.angle)),2);
//	Radius[LB]=pow((CAR_LEN/2 - turn_radius*(float)sin(trans.angle)),2) + pow((CAR_LEN/2 + turn_radius*(float)cos(trans.angle)),2);
//	Radius[RF]=pow((-CAR_LEN/2 + turn_radius*(float)cos(trans.angle)),2) + pow((CAR_LEN/2 - turn_radius*(float)sin(trans.angle)),2);
//	Radius[RB]=pow((-CAR_LEN/2 + turn_radius*(float)cos(trans.angle)),2) + pow((CAR_LEN/2 + turn_radius*(float)sin(trans.angle)),2);

//	opt->speed[LF]=Radius[LF]*trans.ome;
//	opt->speed[LB]=Radius[LB]*trans.ome;
//	opt->speed[RF]=Radius[RF]*trans.ome;
//	opt->speed[RB]=Radius[RB]*trans.ome;
//	 
//	opt->encoder[LF]=(Pi/2 - atan((turn_radius*cos(trans.angle) + CAR_LEN/2) / (turn_radius*sin(trans.angle) + CAR_LEN/2))) / 360*8191;
//	opt->encoder[LB]=(Pi/2 - atan((turn_radius*cos(trans.angle) + CAR_LEN/2) / (-turn_radius*sin(trans.angle) + CAR_LEN/2))) / 360*8191;
//	opt->encoder[RF]=(Pi/2 - atan((turn_radius*cos(trans.angle) - CAR_LEN/2) / (turn_radius*sin(trans.angle) + CAR_LEN/2))) / 360*8191;
//	opt->encoder[RB]=(Pi/2 - atan((turn_radius*cos(trans.angle) - CAR_LEN/2) / (-turn_radius*sin(trans.angle) + CAR_LEN/2))) / 360*8191;

//	for(;i<4;i++)
//	{
////		opt->speed[i]		*= 19;		//从减速箱的旋转，转换到动力杆的旋转
//		opt->encoder[i] 	*= 36;
//	}
//	
//	if(move_t->Vx < 0)								//如果试图后退，则将速度取反，电机反转
//	{
//		opt->speed[LF]=-opt->speed[LF];
//		opt->speed[LB]=-opt->speed[LB];
//		opt->speed[RF]=-opt->speed[RF];
//		opt->speed[RB]=-opt->speed[RB];
//	}
//}

//u16 AngConv_LF(u16 encoder)
//{
//	
//	encoder *= 256;//从减速箱的转角，转换到动力杆的转角
//}
