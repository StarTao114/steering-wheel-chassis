#ifndef _PID_H_
#define _PID_H_

#include "Public.h"
#include "moto_ctrl.h"
#include "can.h"

#define P_SPEED_LIMIT 1000
#define I_SPEED_LIMIT 600				//�����޷����������������ۻ�
#define D_SPEED_LIMIT 2000

#define P_ANGLE_LIMIT 4000
#define I_ANGLE_LIMIT 1500				//�����޷����������������ۻ�
#define D_ANGLE_LIMIT 5000

#define T_SPEED_LIMIT 		1500			//PID���������
#define T_ANGLE_LIMIT 		5000			//PID���������
#define E_SPEED_MAX 	400				//���ڻ��ָ��룬����ͻ�������E���ұ仯
#define E_ANGLE_MAX 	10000

typedef struct PID_OPU
{
	double 	p_out,					//��ǰ���
			i_out,					//����ۻ�
			d_out,					//��λʱ��ı仯��
			total_out;				//�ϼ����ֵ
}Opid;

typedef struct PID_NUM
{
	float 		Kp, Ki, Kd;
	double		E, Last_E;				//EΪ��ǰ���,Last_EΪ�ϴβ���ʱ�����
	double		target; 
	int			current;
}Npid;

typedef struct _motor_angle
{
	uint16_t	encoder,				//��ǰ�Ļ�е�Ƕ�
				last_encoder,			//�ϴεĻ�е�Ƕ�
				total_encoder,			//�ۻ���ת�Ļ�е�Ƕ�
				angle,					//�ۻ���ת����ʵ�Ƕ�
				round_cnt,				//�ۻ���ת��Ȧ��
				angle_offset,			//��������ʵ�Ƕ�
				encoder_offset,			//�����Ļ�е�Ƕ�
				encoder_is_init;		//��ʼ����־
}motor_angle;
//encoder_offset������������ʼλ����Ϊ��ת����㣬��Ҫ�����ͳ��ʱ��ȥ��
//��ʵ���ϸ��ݾ��ԵĻ�еλ����ͳ���ۻ���ת�Ƕȣ�
//�磺�ӻ�е�Ƕ�500��8191����һȦ�����ǳ�ʼ��500���ܱ�������ת�Ƕ���


extern Npid pid_speed_num[wheel_num];//PID����
extern Npid pid_angle_num[wheel_num];

extern Opid pid_speed_opu[wheel_num];//PID�����������Ч��������
extern Opid pid_angle_opu[wheel_num];


void PID_init(void);
void PID_speed(Opid* pid_opu, Npid* pid_num, u8 flag);
void PID_angle(Opid* pid_opu, Npid* pid_num);

void update_angle(motor_angle* _angle, uint16_t angle_fbk);

void PID_speed_LF(void);
void PID_angle_LF(void);
void PID_speed_RF(void);
void PID_angle_RF(void);
void PID_speed_LB(void);
void PID_angle_LB(void);
void PID_speed_RB(void);
void PID_angle_RB(void);

#endif
