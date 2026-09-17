#ifndef __BEEP_H__
#define __BEEP_H__

#include "zf_common_headfile.h"

#define BEEP_PIN (P11_11)   // 蜂鸣器引脚

void beep_init(void);   // 初始化蜂鸣器引脚
void beep_on(void);     // 蜂鸣器响
void beep_off(void);    // 蜂鸣器停

#endif
