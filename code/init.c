#include "init.h"

/* mt9v03x 摄像头驱动导出的全局状态 */
extern uint8 mt9v03x_init(void);          /* 返回 0=成功 1=失败 */
extern uint8 mt9v03x_lost_flag;           /* 0=摄像头在线 1=丢帧/未接 */

void system_0_init(void)
{
    lcd_init();
    key_init(10);
    imu_init();
    system_delay_ms(300);
    data_init();
    encoder_init();
    motor_init();
    pit_ms_init(CCU60_CH0, 10);         // 10ms 定时器，用于按键扫描
    pit_ms_init(CCU60_CH1, 1000);       // 1s 定时器，用于帧率计算
    pit_ms_init(CCU61_CH0, 2);
    pit_ms_init(CCU61_CH1, 10);
}
void system_1_init(void)
{
    system_delay_ms(200);
    if (mt9v03x_init() != 0)
    {
        /* 摄像头 I2C 通讯失败（排线没插/模块供电异常） */
        wifi_result = 2;
       // wifi_stage  = 255;
       show_center("MT9V03X Error");
       system_delay_ms(800);
    }
    else 
    {
        show_center("MT9V03X Success");
    }
    my_wifi_spi_init();
}