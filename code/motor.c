#include "motor.h"
#include "servo.h"
int16 encoder_left = 0;
int16 encoder_right = 0;
void servo_init(void)
{
    pwm_init(SMOTOR_PWM_PIN, SERVO_FREQ, SMOTOR_CENTER); // 上电保持中值
    
}

void servo_set(float angle_deg)
{
    angle_deg = MINMAX(angle_deg, -SMOTOR_LIMIT, SMOTOR_LIMIT);
    pwm_set_duty(SMOTOR_PWM_PIN, (uint32)SMOTOR_DUTY(angle_deg));
}
void motor_init(void)
{
    gpio_init(MOTOR_L_DIR, GPO, 1, GPO_PUSH_PULL);   // 方向脚推挽输出
    gpio_init(MOTOR_R_DIR, GPO, 1, GPO_PUSH_PULL);

    pwm_init(MOTOR_L_PWM, MOTOR_FREQ, 0);
    pwm_init(MOTOR_R_PWM, MOTOR_FREQ, 0);
}

void encoder_init(void)
{
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);  // 左
    encoder_dir_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);  // 右
}
void encoder_update(void)
{
    encoder_left  =  encoder_get_count(TIM6_ENCODER);
    encoder_right =  encoder_get_count(TIM5_ENCODER);
    encoder_clear_count(TIM5_ENCODER);
    encoder_clear_count(TIM6_ENCODER);
}
void go_motor(int16 Left_targht_speed, int16 Right_targht_speed)
{
   int16 l=Left_targht_speed;
   int16 r=Right_targht_speed;
   if(l>9000)
   {
       l=9000;
   }
   else if(l<-9000)
   {
       l=-9000;
   }
   if(r>9000)
   {
       r=9000;
   }
   else if(r<-9000)
   {
       r=-9000;
   }
   if(l>0)
   {
       gpio_set_level(MOTOR_L_DIR, 0);
       pwm_set_duty(MOTOR_L_PWM, l);
   }
   else
   {
       gpio_set_level(MOTOR_L_DIR, 1);
       pwm_set_duty(MOTOR_L_PWM, -l);
   }
   if(r>0)
   {
       gpio_set_level(MOTOR_R_DIR, 0);
       pwm_set_duty(MOTOR_R_PWM, r);
   }
   else
   {
       gpio_set_level(MOTOR_R_DIR, 1);
       pwm_set_duty(MOTOR_R_PWM, -r);
   }
}

/* ================ 阿克曼差速：舵机角 → 内外轮速差 ================ */
// 几何模型：内外轮速差 = 0.5 * 车速 * 轮距 * tan(前轮角*差速比) / 轴距
#define LDISTANCE   (200)   // 轴距(mm)：前后轮距
#define BORDWIDTH   (150)   // 轮距(mm)：左右轮距
int16 differential_add_speed2(int16 aim, float turn)
{
    return (int16)(0.5f * aim * BORDWIDTH
         * tanf(MINMAX(fabsf(PID_ANGLE_TO_RAD(turn)), 0, PID_ANGLE_TO_RAD(SMOTOR_LIMIT)) * turn_diff)
         / LDISTANCE);
}

/* ================ 速度控制：阿克曼差速分配 + 增量式速度环 ================ */
// 10ms 节拍调用：编码器反馈 → 左右轮增量式PID → go_motor 输出
void speed_control(void)
{
    int16 l_goal = base_speed;
    int16 r_goal = base_speed;
    static int16 l_pwm = 0;   // 左轮累计PWM输出
    static int16 r_pwm = 0;   // 右轮累计PWM输出

    if (stop_flog)
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
    // 舵机角>0：左轮是内轮减速；<0：右轮是内轮减速（方向反了就对调这两行）
    // 临时关闭差速排查S弯，确认后再打开
    // if (angle > 0)  l_goal -= differential_add_speed2(base_speed, angle);
    // else            r_goal -= differential_add_speed2(base_speed, angle);

    // 增量式：本次输出 = 上次输出 + 增量
    l_pwm += (int16)increment_pid_solve(&motor_pid_l, l_goal - encoder_left);
    r_pwm += (int16)increment_pid_solve(&motor_pid_r, r_goal - encoder_right);

    // 输出限幅
    if (l_pwm > 9000) l_pwm = 9000;
    if (l_pwm < -9000) l_pwm = -9000;
    if (r_pwm > 9000) r_pwm = 9000;
    if (r_pwm < -9000) r_pwm = -9000;

    go_motor(l_pwm, r_pwm);
}