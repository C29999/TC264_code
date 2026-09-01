#include "pid.h"

// 速度控制的代码
pid_t angle_steer_pid;
pid_t angle_speed_pid;
pid_t speed_l_pid;
pid_t speed_r_pid;
void speed_control(void)
{
    float expect_gyro;//外环输出。期望角速度
    int16 l_goal;
    int16 r_goal;

    if(stop_flog)
    {
        expect_gyro = 0.0f;
    }
    else
    {
        expect_gyro = pid_cal(&angle_steer_pid, image_error_filter);//外环：图像偏出输出期望角速度
    }
    dif_val=(int16)pid_cal(&angle_speed_pid, expect_gyro);//中环：期望角速度输出差速量
    if(stop_flog)
    {
        l_goal = 0;
        r_goal = 0;
    }
    else
    {
        l_goal = base_speed+dif_val;
        r_goal = base_speed-dif_val;
    }
    go_motor((int16)pid_cal_inc(&speed_l_pid, l_goal-encoder_left), (int16)pid_cal_inc(&speed_r_pid, r_goal-encoder_right));

}
void pid_init(pid_t *pid, float kp, float ki, float kd, float low_pass,float out_max, float out_min, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->low_pass = low_pass;
    pid->out_max = out_max;
    pid->out_min = out_min;
    pid->integral_max = integral_max;
}
/**
 * @brief 位置式PID计算
 * @param pid PID结构体指针
 * @param error 当前误差 = 目标值 - 实际值
 *              方向环：直接传image_error_filter（像素偏差）
 *              （目标恒为0，已隐含在 mid-94 的减法里，不用再减）
 * @return 本次输出总量（已限幅），单位由调用方决定
 *              方向环：输出dif_val差速量（速度单位，不是占空比）
 * @addtogroup out = kp*e + ki*∫e + kd*Δe
 *              积分带integral_max限幅防饱和；D项一阶低通，low_pass=1时不滤波
 * @note 输入输出单位由各环自定，参数量级不可跨环套用
 */
float pid_cal(pid_t *pid, float error)
{
    pid->out_p = pid->kp * error;

    pid->out_i += pid->ki * error;
    if (pid->out_i > pid->integral_max) pid->out_i = pid->integral_max;
    if (pid->out_i < -pid->integral_max) pid->out_i = -pid->integral_max;

    pid->out_d = pid->kd * (error - pid->error_last) * pid->low_pass+pid->out_d * (1.0f - pid->low_pass);

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    if (pid->out > pid->out_max) pid->out = pid->out_max;
    if (pid->out < pid->out_min) pid->out = pid->out_min;

    pid->error_last = error;
    return pid->out;
}
float pid_cal_inc(pid_t *pid, float error)
{
    float delta;

    delta = pid->kp * (error - pid->error_last)+pid->ki * error+pid->kd * (error - 2.0f * pid->error_last + pid->error_prev);

    pid->out += delta;

    if (pid->out > pid->out_max) pid->out = pid->out_max;
    if (pid->out < pid->out_min) pid->out = pid->out_min;

    pid->error_prev = pid->error_last;
    pid->error_last = error;
    return pid->out;
}