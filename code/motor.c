#include "motor.h"
#include "servo.h"
#include "image.h"

int16 encoder_left = 0;
int16 encoder_right = 0;

void motor_init(void)
{
    // 左电机PWM初始化
    pwm_init(MOTOR_L_PWM, MOTOR_FREQ, 0);
    gpio_init(MOTOR_L_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // 右电机PWM初始化
    pwm_init(MOTOR_R_PWM, MOTOR_FREQ, 0);
    gpio_init(MOTOR_R_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void encoder_init(void)
{
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);
    encoder_dir_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);
}

void encoder_update(void)
{
    encoder_left = encoder_get_count(TIM6_ENCODER);
    encoder_right = encoder_get_count(TIM5_ENCODER);
    encoder_clear_count(TIM6_ENCODER);
    encoder_clear_count(TIM5_ENCODER);
}
void servo_init(void)
{
    pwm_init(SMOTOR_PWM_PIN, SERVO_FREQ, SMOTOR_CENTER);
}

static float last_servo_angle = 9999.0f;

void servo_set(float angle_deg)
{
    angle_deg = MINMAX(angle_deg, -SMOTOR_LIMIT, SMOTOR_LIMIT);
    if (fabsf(angle_deg - last_servo_angle) < 0.01f)
        return;
    last_servo_angle = angle_deg;
    pwm_set_duty(SMOTOR_PWM_PIN, (uint32)SMOTOR_DUTY(angle_deg));
}

void go_motor(int16 l, int16 r)
{
    if (l > 0)
    {
        gpio_set_level(MOTOR_L_DIR, 0);
        pwm_set_duty(MOTOR_L_PWM, l + L_DEAD_ZONE);
    }
    else if (l < 0)
    {
        gpio_set_level(MOTOR_L_DIR, 1);
        pwm_set_duty(MOTOR_L_PWM, -l + L_DEAD_ZONE);
    }
    else
    {
        gpio_set_level(MOTOR_L_DIR, 0);
        pwm_set_duty(MOTOR_L_PWM, 0);
    }
    if (r > 0)
    {
        gpio_set_level(MOTOR_R_DIR, 0);
        pwm_set_duty(MOTOR_R_PWM, r + R_DEAD_ZONE);
    }
    else if (r < 0)
    {
        gpio_set_level(MOTOR_R_DIR, 1);
        pwm_set_duty(MOTOR_R_PWM, -r + R_DEAD_ZONE);
    }
    else
    {
        gpio_set_level(MOTOR_R_DIR, 0);
        pwm_set_duty(MOTOR_R_PWM, 0);
    }
}

#define LDISTANCE   (200)
#define BORDWIDTH   (150)
int16 differential_add_speed2(int16 aim, float turn)
{
    return (int16)(0.5f * aim * BORDWIDTH
         * tanf(MINMAX(fabsf(PID_ANGLE_TO_RAD(turn)), 0, PID_ANGLE_TO_RAD(SMOTOR_LIMIT)) * turn_diff)/ LDISTANCE);
}
void speed_control(void)
{
    int16 min_pts = (rpts0s_num < rpts1s_num) ? rpts1s_num : rpts0s_num;
    int16 straight_need = (int16)(1.2f / sample_dist);
    int16 dynamic_speed;
    float abs_angle = fabsf(pure_angle);

    if (abs_angle <= 1.0f && min_pts >= straight_need)
    {
        dynamic_speed = (int16)(base_speed * 1.3f);   // 直道加速
    }
    else
    {
        // 弯道按角度连续减速：角度越大，速度越低
        // pure_angle=5° → 1.1, 10° → 0.9, 15° → 0.7, 20° → 0.5
        float factor = 1.3f - abs_angle * 0.04f;
        if (factor < 0.4f) factor = 0.4f;            // 最低不低于 40%
        dynamic_speed = (int16)(base_speed * factor);
    }
    int16 l_goal = dynamic_speed;
    int16 r_goal = dynamic_speed;
    static int16 l_pwm = 0;
    static int16 r_pwm = 0;
    if (stop_flog)
    {
        base_speed = 0;
        l_goal = 0;
        r_goal = 0;
        if (encoder_left > -10 && encoder_left < 10 &&
            encoder_right > -10 && encoder_right < 10)
        {
            l_pwm = 0;
            r_pwm = 0;
            motor_pid_l.pre_error = 0;
            motor_pid_l.pre_pre_error = 0;
            motor_pid_r.pre_error = 0;
            motor_pid_r.pre_pre_error = 0;
            go_motor(0, 0);
            return;
        }
    }
    else
    {
        if (angle > 0)
            l_goal -= differential_add_speed2(dynamic_speed, MINMAX(angle, -12.0f, 12.0f));
        else
            r_goal -= differential_add_speed2(dynamic_speed, MINMAX(angle, -12.0f, 12.0f));
    }
    int16 l_delta = (int16)increment_pid_solve(&motor_pid_l, l_goal - encoder_left);
    int16 r_delta = (int16)increment_pid_solve(&motor_pid_r, r_goal - encoder_right);
    l_pwm += l_delta;
    r_pwm += r_delta;
    if (l_pwm > 9000) l_pwm = 9000;
    if (l_pwm < -9000) l_pwm = -9000;
    if (r_pwm > 9000) r_pwm = 9000;
    if (r_pwm < -9000) r_pwm = -9000;
    go_motor(l_pwm, r_pwm);
}
