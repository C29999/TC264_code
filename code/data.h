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
#endif
