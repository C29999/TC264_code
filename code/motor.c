#include "motor.h"
int16 encoder_left = 0;
int16 encoder_right = 0;
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
    encoder_left  =  encoder_get_count(TIM6_ENCODER);   // 负号按电机实际安装方向调
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