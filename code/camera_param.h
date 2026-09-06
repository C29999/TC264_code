#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

#include "zf_common_headfile.h"

extern const int16 invx[MT9V03X_H][MT9V03X_W];
// 鸟瞰图→原图：y 坐标查找表
extern const int16 invy[MT9V03X_H][MT9V03X_W];

// 查找表为 const 静态数据，无需运行时填充
void camera_param_init(void);

#endif
