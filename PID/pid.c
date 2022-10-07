#include "pid.h"

Npid pid_speed_num[wheel_num];//PID����
Npid pid_angle_num[wheel_num];

Opid pid_speed_opu[wheel_num];//PID�����������Ч��������
Opid pid_angle_opu[wheel_num];

const u8 SPEED = 0;
const u8 ANGLE = 1;

void PID_init(void)
{
	u8 i = 0;
	
	for(i=0;i < wheel_num; i++)
	{
	/***************�ٶȻ�����*****************************/	
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
		
	/***************�ǶȻ�����******************************/
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
		if(pid_num->E < E_SPEED_MAX)					//ͬʱ��I�����޷��ͷ���
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
		if(pid_num->E < E_SPEED_MAX)					//ͬʱ��I�����޷��ͷ���
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
	
//	if(pid_num->E < ANGLE_E_MAX)					//ͬʱ��I�����޷��ͷ���
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


/**********************************��ǰ��*********************************/
//��ǰ���ٶȵ���PID
void PID_speed_LF(void)
{
	pid_speed_num[LF].current=M3508[LF].speed_rpm;
	PID_speed(&pid_speed_opu[LF],&pid_speed_num[LF],SPEED);
}
//��ǰ��ת��˫��PID
void PID_angle_LF(void)
{
	pid_angle_num[A_LF].current=M2006[LF].all_ecd;
	PID_angle(&pid_angle_opu[A_LF],&pid_angle_num[A_LF]);
	pid_speed_num[A_LF].target=pid_angle_opu[A_LF].total_out;
	pid_speed_num[A_LF].current=M2006[LF].speed_rpm;
	PID_speed(&pid_speed_opu[A_LF],&pid_speed_num[A_LF],ANGLE);
}

/**********************************��ǰ��*********************************/
//��ǰ���ٶ�PID
void PID_speed_RF(void)
{
	pid_speed_num[RF].current=M3508[RF].speed_rpm;
	PID_speed(&pid_speed_opu[RF],&pid_speed_num[RF],SPEED);
}
//��ǰ��ת��˫��PID
void PID_angle_RF(void)
{
	pid_angle_num[A_RF].current=M2006[RF].all_ecd;
	PID_angle(&pid_angle_opu[A_RF],&pid_angle_num[A_RF]);
	pid_speed_num[A_RF].target=pid_angle_opu[A_RF].total_out;
	pid_speed_num[A_RF].current=M2006[RF].speed_rpm;
	PID_speed(&pid_speed_opu[A_RF],&pid_speed_num[A_RF],ANGLE);
}


/**********************************�����*********************************/
//������ٶ�PID
void PID_speed_LB(void)
{
	pid_speed_num[LB].current=M3508[LB].speed_rpm;
	PID_speed(&pid_speed_opu[LB],&pid_speed_num[LB],SPEED);
}
//�����ת��˫��PID
void PID_angle_LB(void)
{
	pid_angle_num[A_LB].current=M2006[LB].all_ecd;
	PID_angle(&pid_angle_opu[A_LB],&pid_angle_num[A_LB]);
	pid_speed_num[A_LB].target=pid_angle_opu[A_LB].total_out;
	pid_speed_num[A_LB].current=M2006[LB].speed_rpm;
	PID_speed(&pid_speed_opu[A_LB],&pid_speed_num[A_LB],ANGLE);
}


/**********************************�Һ���*********************************/
//�Һ����ٶ�PID
void PID_speed_RB(void)
{
	pid_speed_num[RB].current=M3508[RB].speed_rpm;
	PID_speed(&pid_speed_opu[RB],&pid_speed_num[RB],SPEED);
}
//�Һ���ת��˫��PID
void PID_angle_RB(void)
{
	pid_angle_num[A_RB].current=M2006[RB].all_ecd;
	PID_angle(&pid_angle_opu[A_RB],&pid_angle_num[A_RB]);
	pid_speed_num[A_RB].target=pid_angle_opu[A_RB].total_out;
	pid_speed_num[A_RB].current=M2006[RB].speed_rpm;
	PID_speed(&pid_speed_opu[A_RB],&pid_speed_num[A_RB],ANGLE);
}

/*************************************************************************/

//�ǶȻ��۸��º�����ע��λ��target�����ж�Ȧ��
//angle_fbk���Ƕȷ���feedback���ӵ����ȡ
void update_angle(motor_angle* _angle, uint16_t angle_fbk)
{
	_angle->encoder = angle_fbk;		
	if(_angle->encoder_is_init)
	{
		if(_angle->encoder - _angle->last_encoder > 4096)			//��ǰ��������Ƕ�-�ϴη����Ƕȳ�����Ȧ
		{															//���ڽǶ�ֵΪ0-8191�����ýǶȵĻ��ۣ�Ҫ��Ȧ��������������ǰ����-�ϴη���ֵ�����ܴﵽĿ��
			_angle->round_cnt --;									//��ת��һȦ								
		}
		if(_angle->encoder - _angle->last_encoder < -4096)	
		{
			_angle->round_cnt ++;									//��ת��һȦ.
		}
	}
	else															//��ȡ������
	{
		_angle->encoder_offset = _angle->encoder;					//��һ�εõ��ĽǶȷ�������encoder_offset����㣩
		_angle->encoder_is_init = 1;
	}
	_angle->angle_offset = _angle->encoder_offset/8191.0f * 360.0f;	//��е�Ƕ�ֵת��Ϊ�Ƕ�ֵ
	_angle->last_encoder = _angle->encoder;
	_angle->total_encoder = _angle->round_cnt*8191 + _angle->encoder - _angle->encoder_offset;	
//	_angle->angle = _angle->total_encoder/8191.0f * 360.0f;			//����ת��
	//ʡ����һ�����ó������ۻ�ת���Ļ�е�Ƕ�
}

//void trans_to_moto()
//{
//		if(moto_2006)
//	{
///*************************************�ǶȻ�**************************************/
//		pid_angle_num.current = (u16)msg_buff[C610_ID][0] << 8 | msg_buff[C610_ID][1];//��ȡ���0x201�ĵ�ǰ�Ƕ�״̬
//		PID(&pid_angle_opu, &pid_angle_num);

//		pid_speed_num.target=pid_angle_opu.total_out;
//	}

///*************************************�ٶȻ�**************************************/
//	if(moto_3508)
//		
//	pid_speed_num.current = (u16)msg_buff[C610_ID][2] << 8 | msg_buff[C610_ID][3];//��ȡ���0x201�ĵ�ǰת��״̬
//	PID(&pid_speed_opu, &pid_speed_num);
//	
//	msg[0] = (int16_t)pid_speed_opu.total_out >> 8;			//��8
//	msg[1] = (int16_t)pid_speed_opu.total_out << 8 >> 8;
//	CAN_Sand_Msg(msg, 8, 0x200);
//}
