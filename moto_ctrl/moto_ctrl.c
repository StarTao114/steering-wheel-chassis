#include "moto_ctrl.h"

const u8 M2006_REDUCT_RATIO_MOTO =  36;				//2006�Դ�������ļ��ٱ���
const u8 M2006_REDUCT_RATIO_MOTO_TO_GEAR =  6;		//2006�������복����̨�ļ��ٱ�

const u8 M3508_REDUCT_RATIO_MOTO =  19;				//ͬ��
const u8 M3508_REDUCT_RATIO_MOTO_TO_GEAR =  2;

//�����ʸ������
void moto_ctrl(move_target* move_t, final_out* opt)
{
	u8 i=0;
	
//	if(move_t->Vy < 0)								//�����ͼ���ˣ����ٶ�ȡ���������ת
//		move_t->Vx = -move_t->Vx;
	
	for(i=0;i<4;i++)
	{
		opt->speed[i] = M3508_REDUCT_RATIO_MOTO_TO_GEAR * M3508_REDUCT_RATIO_MOTO * sqrt(move_t->Vx * move_t->Vx + move_t->Vy * move_t->Vy);
		
		if(move_t->Vy < 2 && move_t->Vy > -2)
			move_t->Vy = 1;
		
		if(move_t->Vy > 0)
			opt->encoder[i] = M2006_REDUCT_RATIO_MOTO_TO_GEAR * M2006_REDUCT_RATIO_MOTO /2.0f * 8191 * atan((float)move_t->Vx / (float)move_t->Vy) / Pi;		//������ת��
		if(move_t->Vy < 0)
			opt->encoder[i] = M2006_REDUCT_RATIO_MOTO_TO_GEAR * M2006_REDUCT_RATIO_MOTO /2.0f * 8191 * ( Pi - atan((float)move_t->Vx / (float)move_t->Vy) ) / Pi;		//������ת��

		if(move_t->Vy < 0)								//�����ͼ���ˣ����ٶ�ȡ���������ת
			opt->speed[i]=-opt->speed[i];
	}
	
	
}


//u16 DR16_Vx;
//u16 DR16_Vy;//��װ����ң�����ķ���ֵ

//ת��뾶�ɿ��ǸĽ�Ϊ��̬��(�Ѹ�Ϊ����turn_radius)
//e.g.ת��Ƕ�Խ��ת��뾶ԽС��
//��ת��뾶->0ʱ��С����ת
//void moto_ctrl(move_target* move_t, final_out* opt)
//{
//	volatile float velo=0;					//�����˶����ٶ�
//	volatile trans_value trans={0,0};
//	volatile float Radius[4]={0};		//ÿ�����ӵ��˶��뾶
//	volatile float turn_radius=0;				//����ת��뾶=200/trans.angle���ݶ���
//	u8 i=0;
//	
//	velo=sqrt(move_t->Vx * move_t->Vx + move_t->Vy * move_t->Vy);
//	if(move_t->Vy != 0)
//		trans.angle=atan(move_t->Vx / move_t->Vy);		//������ת��
//	else if(move_t->Vy <= 0.2f)
//		trans.angle=3.141592f / 2.0f;
//	turn_radius=3/trans.angle;							//��ת�뾶����������
//	trans.ome=velo / turn_radius;					//������ٶ�
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
////		opt->speed[i]		*= 19;		//�Ӽ��������ת��ת���������˵���ת
//		opt->encoder[i] 	*= 36;
//	}
//	
//	if(move_t->Vx < 0)								//�����ͼ���ˣ����ٶ�ȡ���������ת
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
//	encoder *= 256;//�Ӽ������ת�ǣ�ת���������˵�ת��
//}
