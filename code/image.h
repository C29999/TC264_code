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

extern uint8 image_binary[MT9V03X_H][MT9V03X_W];
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

#define IMG_ANGLE_TO_RAD(deg) (deg)*3.1415926f/180.0f

//点云处理函数

int16 clip(int16 x, int16 low, int16 up);//整数裁剪
float fclip(float x, float low, float up);//浮点数裁剪
void  blur_points(float pts_in[][2], int16 num, float pts_out[][2], int16 kernel);//平滑
void  resample_points(float pts_in[][2], int16 num1, float pts_out[][2], int16 *num2, float dist);//重采样
void  local_angle_points(float pts_in[][2], int16 num, float angle_out[], int16 dist);//计算局部角度
void  nms_angle(float angle_in[], int16 num, float angle_out[], int16 kernel);//非极大值抑制
void  find_corners(void);//查找角点
void  find_far_corners(void);//查找远端角点
void  find_farline_l(void);//查找远端左边界
void  find_farline_r(void);//查找远端右边界
#endif
