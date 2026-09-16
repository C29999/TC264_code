#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

#include "zf_common_headfile.h"

extern const int16 invx[MT9V03X_H][MT9V03X_W];
extern const int16 invy[MT9V03X_H][MT9V03X_W];

void camera_param_init(void);

#endif
