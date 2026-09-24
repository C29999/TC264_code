#ifndef WIFI_SPI_H
#define WIFI_SPI_H

#include "zf_common_headfile.h"

#define WIFI_SEND_FLAG  1       //图传开关：1=发图到上位机 0=只跑算法不发图

void my_wifi_spi_init(void);
void wifi_debug(void);
void wifi_debug_data(void);
void wifi_crossline_send(void);  // 十字中线文本 $CLINE sx,sy,ex,ey
int Sin(void);

// WiFi 接收解析
extern int16 wifi_cmd_speed;    // $SPEED 后的第一个数字
extern int16 wifi_cmd_param2;   // $SPEED 后的第二个数字
extern uint8 wifi_cmd_flag;     // 收到新指令时置1，处理完清0
extern volatile uint8 wifi_go_flag; // 收到 $GO 后置1，主循环处理后清0
extern volatile uint8 wifi_stop_flag; // 收到 $STOP 后置1，主循环安全停车

extern int16 rx_speed;
extern int16 rx_change_flag;
void wifi_task(void);
uint8 wifi_remote_ready(void);
void rx_change(void);

#endif
