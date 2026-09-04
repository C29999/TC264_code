#ifndef WIFI_SPI_H
#define WIFI_SPI_H

#include "zf_common_headfile.h"

#define WIFI_SEND_FLAG  1       //图传开关：1=发图到上位机 0=只跑算法不发图

void my_wifi_spi_init(void);
void wifi_image_send(void);
void wifi_debug(void);
int Sin(void);
#endif
