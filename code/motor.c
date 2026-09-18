#include "motor.h"
#include "servo.h"
#include "image.h"

int16 encoder_left = 0;
int16 encoder_right = 0;
volatile int16 motor_goal_left = 0;
volatile int16 motor_goal_right = 0;
volatile int16 motor_pwm_left = 0;
volatile int16 motor_pwm_right = 0;
volatile int16 motor_diff = 0;

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
// 线性差速：turn 是平滑后的循迹角（度），返回带符号的内轮减速量。
// 差速辅助舵机通过急弯，最多减掉基础速度的 50%。
int16 differential_add_speed2(int16 aim, float turn)
{
    const float diff_deadband = 2.0f;
    float abs_turn = fabsf(turn);
    float ratio;
    float diff;
    float max_diff;
    if (abs_turn <= diff_deadband) return 0;
    ratio = (abs_turn - diff_deadband) / (SMOTOR_LIMIT - diff_deadband);
    if (ratio > 1.0f) ratio = 1.0f;
    diff = aim * ratio * turn_diff;
    max_diff = fabsf((float)aim) * 0.50f;
    if (diff >  max_diff) diff =  max_diff;
    if (diff < -max_diff) diff = -max_diff;
    return (int16)diff;
}
void speed_control(void)
{
    int16 min_pts = (rpts0s_num < rpts1s_num) ? rpts1s_num : rpts0s_num;
    int16 straight_need = (int16)(1.2f / sample_dist);
    int16 dynamic_speed;
    float abs_angle = fabsf(pure_angle);
    uint8 sharp_single_corner = (abs_angle > 8.0f && state_flags != 0 &&
        (state_flags & 0x03) != 0x03 && !(state_flags & 0x40));

    if (abs_angle <= 1.0f && min_pts >= straight_need)
    {
        dynamic_speed = straight_speed;   // 直道速度
    }
    else
    {
        // 弯道按角度连续减速：角度越大，速度越低（斜率 corner_speed_slope 在 data.c 改）
        // 例（slope=0.04）：5°→×1.10, 10°→×0.90, 15°→×0.70, 20°→×0.50
        float factor = 1.3f - abs_angle * corner_speed_slope;
        if (factor < 0.4f) factor = 0.4f;            // 最低不低于 40%
        dynamic_speed = (int16)(corner_speed * factor);
        // 负值代表前进，弯道目标不允许比直道目标更快。
        if (dynamic_speed < straight_speed) dynamic_speed = straight_speed;
    }
    // 边线质量下降时先减速，为重新识别和停车保护留出距离。
    if ((state_flags & 0x40) || (left_line_count < 2 && right_line_count < 2))
    {
        dynamic_speed = -100;
    }
    // 单边和直角弯不使用绝对速度上限；基础速度保持由整车速度参数统一控制。
    int16 l_goal = dynamic_speed;
    int16 r_goal = dynamic_speed;
    motor_base_goal = dynamic_speed;
    static int16 l_pwm = 0;
    static int16 r_pwm = 0;
    motor_diff = 0;
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
            motor_goal_left = 0;
            motor_goal_right = 0;
            motor_pwm_left = 0;
            motor_pwm_right = 0;
            go_motor(0, 0);
            return;
        }
    }
    else
    {
        // 差速（内减外加结构）：内轮减 diff，外轮加 diff_outer，过弯更凌厉
        int16 diff = 0;
        if (!(state_flags & 0x40) && (left_line_count >= 2 || right_line_count >= 2))
        {
            diff = differential_add_speed2(dynamic_speed, MINMAX(pure_angle, -12.0f, 12.0f));
            if (sharp_single_corner)
            {
                int16 sharp_limit = (int16)(fabsf((float)dynamic_speed) * 0.65f);
                diff = (int16)(diff * 1.3f);
                if (diff >  sharp_limit) diff =  sharp_limit;
                if (diff < -sharp_limit) diff = -sharp_limit;
            }
        }
        int16 diff_outer = (int16)(diff * turn_diff_outer);
        motor_diff = diff;
        // 图像坐标中 pure_angle < 0 表示目标在右侧，> 0 表示目标在左侧。
        if (pure_angle < 0) // 右转：右轮为内轮（减速），左轮为外轮（加速）
        {
            r_goal -= diff;
            l_goal += diff_outer;
        }
        else                // 左转：左轮为内轮（减速），右轮为外轮（加速）
        {
            l_goal -= diff;
            r_goal += diff_outer;
        }
    }
    // 电机目标速度限幅：不允许反向（前进方向为负值，差速后内轮最多减速到 0，不反转）
    if (l_goal > 0) l_goal = 0;
    if (r_goal > 0) r_goal = 0;
    motor_goal_left = l_goal;
    motor_goal_right = r_goal;
    int16 l_delta = (int16)increment_pid_solve(&motor_pid_l, l_goal - encoder_left);
    int16 r_delta = (int16)increment_pid_solve(&motor_pid_r, r_goal - encoder_right);
    l_pwm += l_delta;
    r_pwm += r_delta;
    if (l_pwm > 9000) l_pwm = 9000;
    if (l_pwm < -9000) l_pwm = -9000;
    if (r_pwm > 9000) r_pwm = 9000;
    if (r_pwm < -9000) r_pwm = -9000;
    motor_pwm_left = l_pwm;
    motor_pwm_right = r_pwm;
    go_motor(l_pwm, r_pwm);
}
