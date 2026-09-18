#include "data.h"
uint8 wifi_ok_flag;
uint8 wifi_flag = 0;    // 诊断阶段关闭图传，TCP连接只发送可解析的TRACE文本
uint8 wifi_init_flag;
volatile uint8 wifi_stage;
volatile uint8 wifi_result;
uint8 fps;
uint8 fps_count;

int16 image_center=94;
int16 mid;
int16 mid_y = 0;           // 中线前瞻点 y 像素坐标（调试十字用） //璧涢亾涓偣鐨勪綅缃�
int16 image_error=0;
int16 image_error_filter=0;



uint8 stop_flog=1;       // 上电保持停车，双击发车后才清零

/* ================ 编码器测距（10ms中断累加） ================ */
int32  total_distance = 0;        // 累计行驶距离（编码器脉冲）
int16  encoder_measure_flag = 0;  // 1=正在测距 0=停止测距
float  total_distance_m = 0;      // 累计距离（米）
float  avg_speed = 0;             // 平均速度（m/s，发车→停车）
uint32 measure_time_ms = 0;       // 测距累计时间（ms）
int16 base_speed=0;       //鍩虹閫熷害鐩爣(缂栫爜鍣ㄨ鏁�/10ms)
int16 dif_val=0;        //鏂瑰悜涓幆杈撳嚭鐨勫樊閫熼噺

//                kp    kp2    ki   kd   low_pass  p_max  i_max  d_max  kgyro
pid_param_t servo_pid = PID_CREATE(0.30, 0.05, 0,  0,  0.3, 14.5,   0,    0,  -0.035);
pid_param_t motor_pid_l = PID_CREATE(20.0, 0, 0.3, 0, 0, 3000, 2000, 0, 0.);
pid_param_t motor_pid_r = PID_CREATE(20.0, 0, 0.3, 0, 0, 3000, 2000, 0, 0.);
pid_param_t motor_pid_l_bangbang = PID_CREATE(60.0, 0, 3.0, 0, 0, 10000, 5000, 0, 0.);
pid_param_t motor_pid_r_bangbang = PID_CREATE(60.0, 0, 3.0, 0, 0, 10000, 5000, 0, 0.);

int16 straight_speed      = -100;
int16 long_straight_speed = -100;
int16 corner_speed        = -80;   // 弯道基础速度（-90→-100，弯道整体加快一点）
float corner_speed_slope  = 0.015f;  // 保持弯道基础速度；直角弯主要依靠轮速比例缩小转弯半径

float pure_angle = 0;        // 纯转角度（弧度），0=直道
float control_error = 0;     // 方向环归一化误差，范围 -50..50
float pure_rad   = 0;        // 纯转角度（弧度），0=直道
float aim_distance = 0.5;    // 目标距离（单位：米）
float angle = 0;             // 目标角度（弧度），0=直道
float servo_pid_raw = 0;     // 方向环限幅前后合成输出、舵机低通之前
float trace_raw_angle = 0;   // Pure Pursuit 原始角度、死区和低通之前
float trace_mid_raw_px = 0;  // 中线 x 在帧间低通之前（像素）
volatile uint16 image_frame_seq = 0;
volatile uint32 image_update_ms = 0;
int16 trace_l_target_pts = 0, trace_r_target_pts = 0;
int16 trace_l_min_mm = 0, trace_r_min_mm = 0;
uint8 track_stop_count = 0, track_invalid_count = 0;
int16 motor_base_goal = 0;
float turn_diff = 0.8f;       // 差速增益；motor.c 会把内轮最大减速限制为基础速度的 50%
float turn_diff_outer = 0;   // 外轮加速比例（相对内轮减速量：0=不加速 0.5=一半 1=同量，越大差速越强）
float corner_cut_px = 0.0f;   // 弯道内切偏移量（像素），0=不内切
float corner_cut_th  = 0.15f;   // 弯道判定阈值（弧度），直道转角小于此不内切
float corner_turn    = 0.0f;   // 前瞻点局部转角（调试显示，正=右弯 负=左弯）
float corner_mismatch_th = 0.35f;   // 两侧边线形态差阈值（弧度≈20°），超过判为闭合/串线，降级单边
float mx_rate_limit = 0.03f;   // 中线单帧最大变化（米/帧≈2px），防前瞻点突变导致舵机抽搐
float corner_buz_th = 0.30f;   // 大弯道蜂鸣阈值（弧度≈17度），超过触发蜂鸣器响一下
int16 buzzer_tick = 0;         // 蜂鸣器剩余响帧数（>0 时响，每帧递减）
uint16 state_flags = 0;         // 取线分支状态位：bit0=L bit1=R bit2=双边 bit3=降级 bit4=单左 bit5=单右 bit6=全丢 bit7=内切
void data_init(void)
{
}
void data_debug(void)
{

}
