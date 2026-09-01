#ifndef DATA_H
#define DATA_H
#include "zf_common_headfile.h"
extern uint8 wifi_ok_flag;
extern uint8 wifi_flag;
extern uint8 wifi_init_flag;
extern volatile uint8 wifi_stage;   /* WIFI 初始化当前阶段：0=未开始 1=SPI 2=WIFI连接 3=TCP 4=完成 255=失败 */
extern volatile uint8 wifi_result;  /* 当前阶段结果：0=进行中 1=成功 2=失败 */
extern uint8 fps;
extern uint8 fps_count;

void data_debug(void);
extern int16 image_center;//图像中心位置
extern int16 image_error;//图像误差
extern int16 mid; //赛道中点的位置
extern int16 image_error_filter;//图像误差滤波

extern uint8 stop_flog;
extern int16 base_speed;    //基础速度目标(编码器计数/10ms)
extern int16 dif_val;       //方向中环输出的差速量

void data_init(void);
void data_debug(void);
#endif
