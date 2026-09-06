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
int16 base_speed=0;       //基础速度目标(编码器计数/10ms)
int16 dif_val=0;        //方向中环输出的差速量

pid_param_t servo_pid = PID_CREATE(0., 0.6, 0.2, 0.8, 14.5, 0, 3.0, 0.); 
pid_param_t motor_pid_l = PID_CREATE(27.0, 0.7, 0, 0, 7000, 5000, 0, 0.);    
pid_param_t motor_pid_r = PID_CREATE(27.0, 0.7, 0, 0, 7000, 5000, 0, 0.);    
pid_param_t motor_pid_l_bangbang = PID_CREATE(60.0, 3.0, 0, 0, 10000, 5000, 0, 0.); 
pid_param_t motor_pid_r_bangbang = PID_CREATE(60.0, 3.0, 0, 0, 10000, 5000, 0, 0.);

int16 straight_speed      = 400;  // 直道速度
int16 long_straight_speed = 450;  // 长直道速度

float pure_angle = 0;        // 前瞻角（度），方向环输入
float pure_rad   = 0;        // 前瞻角（弧度），图像侧算出
float aim_distance = 0.85;   // 前瞻距离（米），高速适当加大
float angle = 0;             // 舵机角输出（度），限±14.5
float turn_diff = 3.8;       // 差速比

void data_init(void)
{
}
void data_debug(void)
{

}
