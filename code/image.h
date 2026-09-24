#ifndef IMAGE_H
#define IMAGE_H

#include "zf_common_headfile.h"
#define IPTS_MAX (200) //边线点的数组容量

// 半赛道宽：车居中时110行处(右线x-左线x)的一半
// 校准：挡住右线,车摆正,看屏幕mid显示M,改成(188-M)
// 占位恒等表阶段沿用透视校准值94；标定换真鸟瞰表后必须改为(0.225f*pixel_per_meter)
#define TRACK_HALF_W  (16)

/* ================ 鸟瞰图（逆透视） ================ */
#define PERS_W  MT9V03X_W    // 鸟瞰图宽 188，与屏幕窗口/原图同构
#define PERS_H  MT9V03X_H    // 鸟瞰图高 120
#define EDGE_WHITE_THRESHOLD (128)   // 二值图白色像素判定阈值（element.c 扫描共用）

// 轻量图像描述符（与国一 image_t 一致，方便逐行移植国一函数）
typedef struct image
{
    uint8 *data;
    int16 width;
    int16 height;
    int16 step;
} image_t;
#define AT_IMAGE(img, x, y) ((img)->data[(y) * (img)->step + (x)])

extern uint8 img_pers_data[PERS_H][PERS_W];     // 鸟瞰灰度图（逆透视输出）
extern int16 touch_boundary0;   // 左巡线碰到图像边界（十字/环岛判据）
extern int16 touch_boundary1;   // 右巡线碰到图像边界
extern int16 maze_start_y;   // 迷宫法实际起始行（显示调试用）
extern int16 maze_start_left_x, maze_start_right_x;
extern int16 lookahead_lx, lookahead_rx, lookahead_y;   // 前瞻行左右边界点(原图像素,-1=无)

extern uint8 image_binary[MT9V03X_H][MT9V03X_W];
extern uint8 wifi_scratch_buf[];   /* WiFi 1-bit二值图打包缓冲 */
extern int16 left_line_points[IPTS_MAX][2];
extern int16 right_line_points[IPTS_MAX][2];
extern uint16 left_line_count;
extern uint16 right_line_count;
static uint8 otsu_local_threshold(const uint8 image[MT9V03X_H][MT9V03X_W],uint16 x_left_yes, uint16 x_right_no,uint16 y_top_yes, uint16 y_bottom_no);
void image_threshold(const uint8 image[MT9V03X_H][MT9V03X_W]);
void image_threshold_block(const uint8 image[MT9V03X_H][MT9V03X_W]);
void image_display_otsu_thresholds(void);
void anti_perspective_fast(void);
void find_edges_binary(void);
void process_edge_points(void);
void calculation_error(void);
void track_protection(void);


#define POINTS_MAX_LEN 200//点集的最大容量
#define sample_dist 0.02f //重采样的间距
#define angle_dist 0.05f //局部角度参考距离
#define pixel_per_meter 70.0f

extern float rpts0[POINTS_MAX_LEN][2];   // L0 原始浮点
extern float rpts1[POINTS_MAX_LEN][2];
extern int16  rpts0_num, rpts1_num;

extern float rpts0b[POINTS_MAX_LEN][2];  // L1 平滑
extern float rpts1b[POINTS_MAX_LEN][2];
extern int16  rpts0b_num, rpts1b_num;

extern float rpts0s[POINTS_MAX_LEN][2]; // L2 重采样(核心)
extern float rpts1s[POINTS_MAX_LEN][2];
extern int16  rpts0s_num, rpts1s_num;

extern float rpts0a[POINTS_MAX_LEN];    // L3 局部角度
extern float rpts1a[POINTS_MAX_LEN];
extern int16  rpts0a_num, rpts1a_num;

extern float rpts0an[POINTS_MAX_LEN];   // L4 NMS后角度
extern float rpts1an[POINTS_MAX_LEN];
extern int16  rpts0an_num, rpts1an_num;

/* ================ 角点结果 ================ */
extern int16 Lpt0_rpts0s_id, Lpt1_rpts1s_id;
extern int16 N_Lpt0_rpts0s_id, N_Lpt1_rpts1s_id;
extern int16 Lpt0_found, Lpt1_found;
extern int16 N_Lpt0_found, N_Lpt1_found;

/* ================ 远端角点 ================ */
extern int16 far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;
extern int16 far_Lpt0_found, far_Lpt1_found;
extern float far_rpts0s[POINTS_MAX_LEN][2];
extern float far_rpts1s[POINTS_MAX_LEN][2];
extern float far_orig0[POINTS_MAX_LEN][2];
extern float far_orig1[POINTS_MAX_LEN][2];
extern int16  far_rpts0s_num, far_rpts1s_num;

/* ================ 直线度 + 置信度 ================ */
extern int16 is_straight0, is_straight1;
extern float conf1, conf2, conf1_max, conf2_max;

/* ================ 十字状态机（移植 STC32 例程：折角角点+路口中心导航） ================ */
/* 十字识别与十字转向分开控制：CROSS_ENABLE 管检测/显示/状态机，
 * CROSS_CONTROL_ENABLE 仅在 CR_ENTER/CR_OUT 时接管转向。 */
#define CROSS_ENABLE 0
#define CROSS_CONTROL_ENABLE 0
#define CROSS_SCAN_TOP_Y 90
/* 十字状态：element.c 定义 cross_flag，image.c/wifi_spi.c 共同消费 */
typedef enum { CR_NONE = 0, CR_START = 1, CR_ENTER = 2, CR_OUT = 3 } cross_state_t;
extern uint8 cross_flag;          // 当前十字状态（0无 1发现近端角点 2十字中 3驶出）
extern uint8 cross_candidate_frames; // 当前连续十字候选帧数（上位机调试用）
extern uint8 cross_near_pair_valid;  // 左右近端角点的几何配对是否有效
extern uint8 cross_geometry_valid;   // 近角配对与补线几何是否同时有效
extern uint8 cross_exit_lane_valid;  // 当前帧是否已恢复为出口窄竖道
extern uint8 cross_exit_confirm_frames; // 出口窄竖道连续确认帧数
extern float cross_phase_distance;   // 当前十字阶段累计距离（米）
extern int16 cross_max_x;         // 对面路口中心 x（原图像素，enter/out 导航目标）
extern int16 cross_far_y;         // 对面路口所在行（原图像素，-1=无效）
extern int16 cross_edge_l, cross_edge_r;  // 对面路口左右边缘（原图像素，显示用）

extern uint8 cross_line_active;   // 1=十字状态机激活中（蜂鸣同源触发 / 调试）
// 补线锁存角点（原图像素），cross_line_active 期间有效，上位机持续显示用
extern int16 cross_hold_nl_x, cross_hold_nl_y;
extern int16 cross_hold_nr_x, cross_hold_nr_y;
extern int16 cross_hold_fl_x, cross_hold_fl_y;
extern int16 cross_hold_fr_x, cross_hold_fr_y;

/* ================ 四角点虚拟补线（屏幕/WiFi 显示 + 十字区域巡线导航） ================ */
/* cross_vline_valid=1 时四角点像素坐标有效：
 *   虚拟左边线 NL-FL、虚拟右边线 NR-FR（补出十字横路段断开的边线）；
 *   虚拟中线 M0→M1（M0=近端两角点中点，M1=远端两角点中点）。 */
extern uint8  cross_vline_valid;
extern int16 cv_nl_x, cv_nl_y, cv_nr_x, cv_nr_y;   // 近端左/右角点（像素）
extern int16 cv_fl_x, cv_fl_y, cv_fr_x, cv_fr_y;   // 远端左/右角点（像素）
extern int16 cv_m0_x, cv_m0_y, cv_m1_x, cv_m1_y;   // 虚拟中线近端/远端点（像素）

#define IMG_ANGLE_TO_RAD(deg) (deg)*3.1415926f/180.0f

//点云处理函数

int16 clip(int16 x, int16 low, int16 up);//整数裁剪
float fclip(float x, float low, float up);//浮点数裁剪
void  blur_points(float pts_in[][2], int16 num, float pts_out[][2], int16 kernel);//平滑
void  resample_points(float pts_in[][2], int16 num1, float pts_out[][2], int16 *num2, float dist);//重采样
void  local_angle_points(float pts_in[][2], int16 num, float angle_out[], int16 dist);//计算局部角度
void  nms_angle(float angle_in[], int16 num, float angle_out[], int16 kernel);//非极大值抑制
void find_corners(void);//查找角点
void find_far_corners_crawl(void);//近端角点向上爬黑白边界找远端外角点 FL/FR
void cross_build_vlines(void);//四角点齐全：生成虚拟边线 NL-FL/NR-FR 与虚拟中线 M0-M1（显示/WiFi）
void cross_apply_integer_edges(void);//十字确认后按逐行整数坐标写回边线数组
uint8 crawl_trace_pixel(int16 x, int16 y);//爬线轨迹位图查询（display.c 青色绘制）
extern int16 crawl_show_ymax;//本帧爬线上界（-1=未爬，屏幕不画轨迹）
void  find_cross_center(void);//十字中列+行扫描对面路口中心（非角点检测）
void  cross_line_completion(void);//十字补线：四角点有效时把断开的边界线补成连续虚拟线
void  beeper_poll(void);//蜂鸣器统一输出：大弯道 + 十字（四角点齐哔哔哔）
void findline_lefthand_binary(const uint8 binary[PERS_H][PERS_W], int16 x, int16 y, int16 points[][2], uint16 *point_count);
void findline_righthand_binary(const uint8 binary[PERS_H][PERS_W], int16 x, int16 y, int16 points[][2], uint16 *point_count);
void  find_farline_l(void);//查找远端左边界
void  find_farline_r(void);//查找远端右边界
#endif
