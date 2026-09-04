#ifndef __PID_H__
#define __PID_H__

#include "zf_common_headfile.h"
typedef struct
{
    float kp;
    float ki;
    float kd;
    float low_pass;     //D项低通：1=不滤波，越小越平滑

    float error_last;   //e(k-1)
    float error_prev;   //e(k-2)

    float out_p;
    float out_i;
    float out_d;

    float out;
    float out_max;
    float out_min;
    float integral_max;
} pid_t;

extern pid_t angle_steer_pid;   //方向外环（现在=视觉误差→差速）
extern pid_t angle_speed_pid;   //方向内环（陀螺仪角速度环，以后串级用）
extern pid_t speed_l_pid;       //左轮速度环
extern pid_t speed_r_pid;       //右轮速度环
void pid_init(pid_t *pid, float kp, float ki, float kd, float low_pass,float out_max, float out_min, float integral_max);
float pid_cal(pid_t *pid, float error);
float pid_cal_inc(pid_t *pid, float error);
void speed_control(void);   //速度内环（10ms，isr.c 调用）

#endif