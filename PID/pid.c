#include "pid.h"

Npid pid_speed_num[wheel_num];//PID参数
Npid pid_angle_num[wheel_num];

Opid pid_speed_opu[wheel_num];//PID对输出的作用效果数量化
Opid pid_angle_opu[wheel_num];

const u8 SPEED = 0;
const u8 ANGLE = 1;

void PID_init(void)
{
	u8 i = 0;
	
	for(i=0;i < wheel_num; i++)
	{
	/***************速度环参数*****************************/	
		pid_speed_num[i].Kp = 0.25f;
		pid_speed_num[i].Ki = 0;
		pid_speed_num[i].Kd = 3;
		pid_speed_num[i].Last_E = 0;
		pid_speed_num[i].current = 0;
		pid_speed_num[i].target = 0;
		
		pid_speed_opu[i].p_out=0;
		pid_speed_opu[i].i_out=0;
		pid_speed_opu[i].d_out=0;
		pid_speed_opu[i].total_out=10;
		
	/***************角度环参数******************************/
		pid_angle_num[i].Kp = 1;
		pid_angle_num[i].Ki = 0.01f;
		pid_angle_num[i].Kd = 0;
		pid_angle_num[i].Last_E = 0;
		pid_angle_num[i].current = 0;
		
		pid_angle_num[i].target = 100;
		
		pid_angle_opu[i].p_out=0;
		pid_angle_opu[i].i_out=0;
		pid_angle_opu[i].d_out=0;
		pid_angle_opu[i].total_out=10;
	}
	
	for(i=4;i<8;i++)
	{
		pid_speed_num[i].Kp = 1;
		pid_speed_num[i].Ki = 0.0f;
		pid_speed_num[i].Kd = 0;
		pid_speed_num[i].Last_E = 0;
		pid_speed_num[i].current = 0;
		pid_speed_num[i].target = 0;
	}
//	pid_speed_num[7].Kp = 1;
	pid_speed_num[7].Kd = 5;
}


void PID_speed(Opid* pid_opu, Npid* pid_num, u8 flag)
{
	pid_num->E = pid_num->target - pid_num->current;
	
	pid_opu->p_out = pid_num->Kp * pid_num->E;
	
	if(flag == SPEED)
	{
		if(pid_num->E < E_SPEED_MAX)					//同时对I进行限幅和分离
		{
			if(pid_opu->i_out > I_SPEED_LIMIT)	pid_opu->i_out = I_SPEED_LIMIT;
			if(pid_opu->i_out < -I_SPEED_LIMIT)	pid_opu->i_out = -I_SPEED_LIMIT;
			pid_opu->i_out += pid_num->Ki * pid_num->E;
		}
		else
		{
			pid_opu->p_out *= 0.5f;
			pid_opu->i_out = 0;
		}
	}
	else if(flag == ANGLE)
	{
		if(pid_num->E < E_SPEED_MAX)					//同时对I进行限幅和分离
		{
			if(pid_opu->i_out > I_ANGLE_LIMIT)	pid_opu->i_out = I_ANGLE_LIMIT;
			if(pid_opu->i_out < -I_ANGLE_LIMIT)	pid_opu->i_out = -I_ANGLE_LIMIT;
			pid_opu->i_out += pid_num->Ki * pid_num->E;
		}
		else
		{
			pid_opu->p_out *= 0.5f;
			pid_opu->i_out = 0;
		}
	}
	
	
	pid_opu->d_out = pid_num->Kd * (pid_num->E - pid_num->Last_E);
	pid_opu->total_out = pid_opu->p_out + pid_opu->i_out + pid_opu->d_out;
	
	if(flag == SPEED)
	{
		if(pid_opu->total_out > T_SPEED_LIMIT)	pid_opu->total_out = T_SPEED_LIMIT;
		if(pid_opu->total_out < -T_SPEED_LIMIT)	pid_opu->total_out = -T_SPEED_LIMIT;
	}
	else if(flag == ANGLE)
	{
		if(pid_opu->total_out > T_ANGLE_LIMIT)	pid_opu->total_out = T_ANGLE_LIMIT;
		if(pid_opu->total_out < -T_ANGLE_LIMIT)	pid_opu->total_out = -T_ANGLE_LIMIT;

	}
	pid_num->Last_E = pid_num->E;
}

void PID_angle(Opid* pid_opu, Npid* pid_num)
{
	double new_E=0;
	
	new_E = pid_num->target - pid_num->current;
	
	pid_opu->p_out = pid_num->Kp * new_E;
	
//	if(pid_num->E < ANGLE_E_MAX)					//同时对I进行限幅和分离
//	{
		pid_opu->i_out += pid_num->Ki * (new_E + pid_num->E + pid_num->Last_E) /4.0f;
		pid_opu->d_out = pid_num->Kd * (pid_num->E - pid_num->Last_E);
	
		if(pid_opu->p_out > P_ANGLE_LIMIT)	pid_opu->p_out = P_ANGLE_LIMIT;
		if(pid_opu->p_out < -P_ANGLE_LIMIT)	pid_opu->p_out = -P_ANGLE_LIMIT;
		if(pid_opu->i_out > I_ANGLE_LIMIT)	pid_opu->i_out = I_ANGLE_LIMIT;
		if(pid_opu->i_out < -I_ANGLE_LIMIT)	pid_opu->i_out = -I_ANGLE_LIMIT;
		if(pid_opu->d_out > D_ANGLE_LIMIT)	pid_opu->d_out = D_ANGLE_LIMIT;
		if(pid_opu->d_out < -D_ANGLE_LIMIT)	pid_opu->d_out = -D_ANGLE_LIMIT;
//	}
//	else
//	{
//		pid_opu->p_out *= 0.5f;
//		pid_opu->i_out = 0;
//	}
	
	
	
	pid_opu->total_out = pid_opu->p_out + pid_opu->i_out + pid_opu->d_out;
//	if(pid_opu->total_out > T_LIMIT)	pid_opu->total_out = T_LIMIT;
//	if(pid_opu->total_out < -T_LIMIT)	pid_opu->total_out = -T_LIMIT;
	
	pid_num->Last_E = pid_num->E;
	pid_num->E = new_E;
}


/**********************************左前轮*********************************/
//左前轮速度单环PID
void PID_speed_LF(void)
{
	pid_speed_num[LF].current=M3508[LF].speed_rpm;
	PID_speed(&pid_speed_opu[LF],&pid_speed_num[LF],SPEED);
}
//左前轮转向双环PID
void PID_angle_LF(void)
{
	pid_angle_num[A_LF].current=M2006[LF].all_ecd;
	PID_angle(&pid_angle_opu[A_LF],&pid_angle_num[A_LF]);
	pid_speed_num[A_LF].target=pid_angle_opu[A_LF].total_out;
	pid_speed_num[A_LF].current=M2006[LF].speed_rpm;
	PID_speed(&pid_speed_opu[A_LF],&pid_speed_num[A_LF],ANGLE);
}

/**********************************右前轮*********************************/
//右前轮速度PID
void PID_speed_RF(void)
{
	pid_speed_num[RF].current=M3508[RF].speed_rpm;
	PID_speed(&pid_speed_opu[RF],&pid_speed_num[RF],SPEED);
}
//右前轮转向双环PID
void PID_angle_RF(void)
{
	pid_angle_num[A_RF].current=M2006[RF].all_ecd;
	PID_angle(&pid_angle_opu[A_RF],&pid_angle_num[A_RF]);
	pid_speed_num[A_RF].target=pid_angle_opu[A_RF].total_out;
	pid_speed_num[A_RF].current=M2006[RF].speed_rpm;
	PID_speed(&pid_speed_opu[A_RF],&pid_speed_num[A_RF],ANGLE);
}


/**********************************左后轮*********************************/
//左后轮速度PID
void PID_speed_LB(void)
{
	pid_speed_num[LB].current=M3508[LB].speed_rpm;
	PID_speed(&pid_speed_opu[LB],&pid_speed_num[LB],SPEED);
}
//左后轮转向双环PID
void PID_angle_LB(void)
{
	pid_angle_num[A_LB].current=M2006[LB].all_ecd;
	PID_angle(&pid_angle_opu[A_LB],&pid_angle_num[A_LB]);
	pid_speed_num[A_LB].target=pid_angle_opu[A_LB].total_out;
	pid_speed_num[A_LB].current=M2006[LB].speed_rpm;
	PID_speed(&pid_speed_opu[A_LB],&pid_speed_num[A_LB],ANGLE);
}


/**********************************右后轮*********************************/
//右后轮速度PID
void PID_speed_RB(void)
{
	pid_speed_num[RB].current=M3508[RB].speed_rpm;
	PID_speed(&pid_speed_opu[RB],&pid_speed_num[RB],SPEED);
}
//右后轮转向双环PID
void PID_angle_RB(void)
{
	pid_angle_num[A_RB].current=M2006[RB].all_ecd;
	PID_angle(&pid_angle_opu[A_RB],&pid_angle_num[A_RB]);
	pid_speed_num[A_RB].target=pid_angle_opu[A_RB].total_out;
	pid_speed_num[A_RB].current=M2006[RB].speed_rpm;
	PID_speed(&pid_speed_opu[A_RB],&pid_speed_num[A_RB],ANGLE);
}

/*************************************************************************/

//角度积累更新函数（注意位置target可以有多圈）
//angle_fbk，角度反馈feedback，从电调获取
void update_angle(motor_angle* _angle, uint16_t angle_fbk)
{
	_angle->encoder = angle_fbk;		
	if(_angle->encoder_is_init)
	{
		if(_angle->encoder - _angle->last_encoder > 4096)			//当前电机反馈角度-上次反馈角度超过半圈
		{															//由于角度值为0-8191：想获得角度的积累，要用圈计数来辅助（当前反馈-上次反馈值）才能达到目的
			_angle->round_cnt --;									//反转减一圈								
		}
		if(_angle->encoder - _angle->last_encoder < -4096)	
		{
			_angle->round_cnt ++;									//正转加一圈.
		}
	}
	else															//获取补偿量
	{
		_angle->encoder_offset = _angle->encoder;					//第一次得到的角度反馈赋给encoder_offset（零点）
		_angle->encoder_is_init = 1;
	}
	_angle->angle_offset = _angle->encoder_offset/8191.0f * 360.0f;	//机械角度值转化为角度值
	_angle->last_encoder = _angle->encoder;
	_angle->total_encoder = _angle->round_cnt*8191 + _angle->encoder - _angle->encoder_offset;	
//	_angle->angle = _angle->total_encoder/8191.0f * 360.0f;			//量纲转换
	//省略上一步，得出的是累积转过的机械角度
}

//void trans_to_moto()
//{
//		if(moto_2006)
//	{
///*************************************角度环**************************************/
//		pid_angle_num.current = (u16)msg_buff[C610_ID][0] << 8 | msg_buff[C610_ID][1];//获取电机0x201的当前角度状态
//		PID(&pid_angle_opu, &pid_angle_num);

//		pid_speed_num.target=pid_angle_opu.total_out;
//	}

///*************************************速度环**************************************/
//	if(moto_3508)
//		
//	pid_speed_num.current = (u16)msg_buff[C610_ID][2] << 8 | msg_buff[C610_ID][3];//获取电机0x201的当前转速状态
//	PID(&pid_speed_opu, &pid_speed_num);
//	
//	msg[0] = (int16_t)pid_speed_opu.total_out >> 8;			//高8
//	msg[1] = (int16_t)pid_speed_opu.total_out << 8 >> 8;
//	CAN_Sand_Msg(msg, 8, 0x200);
//}
