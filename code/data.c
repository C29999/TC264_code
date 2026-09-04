#include "data.h"
uint8 wifi_ok_flag;
uint8 wifi_flag = 1;    //图像发送开关：1=发图到上位机 0=不发
uint8 wifi_init_flag;
volatile uint8 wifi_stage;
volatile uint8 wifi_result;
uint8 fps;
uint8 fps_count;

int16 image_center=94;
int16 mid; //赛道中点的位置
int16 image_error=0;
int16 image_error_filter=0;



uint8 stop_flog=0;
int16 base_speed=50;     //基础速度目标(编码器计数/10ms)
int16 dif_val=0;        //方向中环输出的差速量
void data_init(void)
{
    
    pid_init(&angle_steer_pid, 0.5f, 0.0f, 1.0f, 0.5f, 100.0f, -100.0f, 0.0f);
    pid_init(&angle_speed_pid, 0.8f, 0.0f, 0.0f, 1.0f, 60.0f, -60.0f, 0.0f);
    pid_init(&speed_l_pid, 12.0f, 2.0f, 6.0f, 1.0f, (float)MOTOR_DUTY_MAX, -(float)MOTOR_DUTY_MAX, 0.0f);
    pid_init(&speed_r_pid, 12.0f, 2.0f, 6.0f, 1.0f, (float)MOTOR_DUTY_MAX, -(float)MOTOR_DUTY_MAX, 0.0f);
}
void data_debug(void)
{

}
