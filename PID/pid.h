#ifndef _PID_H_
#define _PID_H_

#include "Public.h"
#include "moto_ctrl.h"
#include "can.h"

#define P_SPEED_LIMIT 1000
#define I_SPEED_LIMIT 600				//积分限幅，限制意外的误差累积
#define D_SPEED_LIMIT 2000

#define P_ANGLE_LIMIT 4000
#define I_ANGLE_LIMIT 1500				//积分限幅，限制意外的误差累积
#define D_ANGLE_LIMIT 5000

#define T_SPEED_LIMIT 		1500			//PID总输出上限
#define T_ANGLE_LIMIT 		5000			//PID总输出上限
#define E_SPEED_MAX 	400				//用于积分隔离，限制突发的误差E剧烈变化
#define E_ANGLE_MAX 	10000

typedef struct PID_OPU
{
	double 	p_out,					//当前误差
			i_out,					//误差累积
			d_out,					//单位时间的变化量
			total_out;				//合计输出值
}Opid;

typedef struct PID_NUM
{
	float 		Kp, Ki, Kd;
	double		E, Last_E;				//E为当前误差,Last_E为上次采样时的误差
	double		target; 
	int			current;
}Npid;

typedef struct _motor_angle
{
	uint16_t	encoder,				//当前的机械角度
				last_encoder,			//上次的机械角度
				total_encoder,			//累积旋转的机械角度
				angle,					//累积旋转的真实角度
				round_cnt,				//累积旋转的圈数
				angle_offset,			//补偿的真实角度
				encoder_offset,			//补偿的机械角度
				encoder_is_init;		//初始化标志
}motor_angle;
//encoder_offset：补偿，将初始位置作为旋转的零点，需要在最后统计时减去，
//（实际上根据绝对的机械位置在统计累积旋转角度）
//如：从机械角度500到8191算作一圈，但是初始的500不能被记入旋转角度中


extern Npid pid_speed_num[wheel_num];//PID参数
extern Npid pid_angle_num[wheel_num];

extern Opid pid_speed_opu[wheel_num];//PID对输出的作用效果数量化
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
