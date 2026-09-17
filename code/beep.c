#include "beep.h"

void beep_init(void)
{
    gpio_init(BEEP_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);   // 默认关
}

void beep_on(void)
{
    gpio_set_level(BEEP_PIN, GPIO_HIGH);   // 蜂鸣器响
}

void beep_off(void)
{
    gpio_set_level(BEEP_PIN, GPIO_LOW);    // 蜂鸣器停
}
