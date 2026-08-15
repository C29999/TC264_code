#ifndef IMAGE_H
#define IMAGE_H

#include "zf_common_headfile.h"

extern uint8 image_binary[MT9V03X_H][MT9V03X_W];
static uint8 otsu_local_threshold(const uint8 image[MT9V03X_H][MT9V03X_W],uint16 x_left_yes, uint16 x_right_no,uint16 y_top_yes, uint16 y_bottom_no);
void image_threshold(const uint8 image[MT9V03X_H][MT9V03X_W]);
void image_display_otsu_thresholds(void);
void findline_lefthand_binary(const uint8 binary[MT9V03X_H][MT9V03X_W], int16 start_x, int16 start_y, int16 points[][2], uint16 *point_count);
void findline_righthand_binary(const uint8 binary[MT9V03X_H][MT9V03X_W], int16 start_x, int16 start_y, int16 points[][2], uint16 *point_count);

#endif
