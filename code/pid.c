#include "pid.h"
#include "imu.h"

/**
 * 方向环主力：平方P PD + 陀螺抑制
 * 和普通PD的区别：P = kp*e²/12500 + ki，误差越大P越猛（弯道自动加力）
 * D项是低通滤波后的误差本身（微分先行思想，抑制噪声）
 * 末尾 - gyro_z*kgyro：车已经在转就反向抵消一部分，防止舵机打过冲
 * 输出单位：舵机角度（度）
 * 注意12500、kgyro都是配套的标定量，只改kp/ki/kd时不要动它
 */
float quadradic_pid_solve(pid_param_t *pid, float error)
{
    pid->out_p = pid->kp * error * error / 12500.f + pid->ki;

    // 真正的微分先行 + 一阶低通（抑制噪声）
    float diff = error - pid->pre_error;
    pid->out_d = diff * pid->low_pass + pid->out_d * (1.f - pid->low_pass);

    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return MINMAX(error * pid->out_p, -pid->p_max, pid->p_max)
         + MINMAX(pid->kd * pid->out_d, -pid->d_max, pid->d_max)
         - gyro_z * pid->kgyro;
}

/**
 * 速度环：增量式 PID
 * delta = kp*(e-e1) + ki*e + kd*(e-2e1+e2)，输出累加到占空比
 * pre_output 保护：上次输出已顶到±10000还同向 → 强制归零，防电机堵转烧管
 * 输入输出单位：编码器误差 → PWM占空比
 */
float increment_pid_solve(pid_param_t *pid, float error)
{
    pid->out_d = MINMAX(pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error), -pid->d_max, pid->d_max);
    pid->out_p = MINMAX(pid->kp * (error - pid->pre_error), -pid->p_max, pid->p_max);
    pid->out_i = MINMAX(pid->ki * error, -pid->i_max, pid->i_max);

    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    pid->output = pid->out_p + pid->out_i + pid->out_d;

    if (pid->pre_output > 10000)
        if (pid->output > 0)
            pid->output = 0.;
    if (pid->pre_output < -10000)
        if (pid->output < 0)
            pid->output = 0.;

    pid->pre_output = pid->output;
    return pid->output;
}

/**
 * 备用：变积分增量式。误差大时 ki 按 S 曲线衰减（积分快退），防大偏差积饱和
 * 
 */
float changable_pid_solve(pid_param_t *pid, float error)
{
    pid->out_d = MINMAX(pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error), -pid->d_max, pid->d_max);
    pid->out_p = MINMAX(pid->kp * (error - pid->pre_error), -pid->p_max, pid->p_max);

    float ki_index = pid->ki;
    if (error + pid->pre_error > 0)
        ki_index = MAX((pid->ki) - (pid->ki) / (1.f + expf(100.f - 0.2f * fabsf(error))), 0.);

    pid->out_i = MINMAX(ki_index * error, -pid->i_max, pid->i_max);

    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    pid->output = pid->out_p + pid->out_i + pid->out_d;

    if (pid->pre_output > 10000)
        if (pid->output > 0)
            pid->output = 0.;
    if (pid->pre_output < -10000)
        if (pid->output < 0)
            pid->output = 0.;

    pid->pre_output = pid->output;
    return pid->output;
}

/**
 * 备用：BangBang。误差>8°直接全速打（15000），否则退化成普通增量式
 * 配合直道冲刺变速用，先抄了备用
 */
float bangbang_pid_solve(pid_param_t *pid, float error)
{
    float out;
    pid->error = error;

    if (error > 8 || error < -8)
    {
        out = (error > 0) ? 15000.f : -15000.f;
    }
    else
    {
        pid->out_d = pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error);
        pid->out_p = pid->kp * (error - pid->pre_error);
        pid->out_i = pid->ki * error;
        out = MINMAX(pid->out_p, -pid->p_max, pid->p_max)
            + MINMAX(pid->out_i, -pid->i_max, pid->i_max)
            + MINMAX(pid->out_d, -pid->d_max, pid->d_max);
    }
    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;
    return out;
}