#include "data.h"
uint8 wifi_ok_flag;
uint8 wifi_flag = 1;    //鍥惧儚鍙戦�佸紑鍏筹細1=鍙戝浘鍒颁笂浣嶆満 0=涓嶅彂
uint8 wifi_init_flag;
volatile uint8 wifi_stage;
volatile uint8 wifi_result;
uint8 fps;
uint8 fps_count;

int16 image_center=94;
int16 mid; //璧涢亾涓偣鐨勪綅缃�
int16 image_error=0;
int16 image_error_filter=0;



uint8 stop_flog=0;
int16 base_speed=0;       //鍩虹閫熷害鐩爣(缂栫爜鍣ㄨ鏁�/10ms)
int16 dif_val=0;        //鏂瑰悜涓幆杈撳嚭鐨勫樊閫熼噺

//                kp    kp2    ki   kd   low_pass  p_max  i_max  d_max  kgyro
pid_param_t servo_pid = PID_CREATE(1.2,  0.03,  0,  2.0,  0.3, 14.5,   0,    8.0,  -0.02);
pid_param_t motor_pid_l = PID_CREATE(20.0, 0, 0.3, 0, 0, 3000, 2000, 0, 0.);
pid_param_t motor_pid_r = PID_CREATE(20.0, 0, 0.3, 0, 0, 3000, 2000, 0, 0.);
pid_param_t motor_pid_l_bangbang = PID_CREATE(60.0, 0, 3.0, 0, 0, 10000, 5000, 0, 0.);
pid_param_t motor_pid_r_bangbang = PID_CREATE(60.0, 0, 3.0, 0, 0, 10000, 5000, 0, 0.);

int16 straight_speed      = -220;
int16 long_straight_speed = -220;
int16 corner_speed        = -180;   // 弯道基础速度（-90→-100，弯道整体加快一点）
float corner_speed_slope  = 0.01f;   // 弯道减速斜率（越大减速越快，0.04≈温和 0.06≈激进）

float pure_angle = 0;        // 鍓嶇灮瑙掞紙搴︼級锛屾柟鍚戠幆杈撳叆
float pure_rad   = 0;        // 鍓嶇灮瑙掞紙寮у害锛夛紝鍥惧儚渚х畻鍑�
float aim_distance = 0.5;    // 鍓嶇灮璺濈锛堢背锛夛紝缂╃煭鍓嶇灮鍑忓皯杩囧啿
float angle = 0;             // 鑸垫満瑙掕緭鍑猴紙搴︼級锛岄檺卤14.5
float turn_diff = 2.2f;       // 差速比（内轮减速比例 0~1，越大差速越强，0.9≈强）
float turn_diff_outer = 0;   // 外轮加速比例（相对内轮减速量：0=不加速 0.5=一半 1=同量，越大差速越强）

void data_init(void)
{
}
void data_debug(void)
{

}
