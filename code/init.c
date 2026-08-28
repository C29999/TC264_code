#include "init.h"

/* mt9v03x 摄像头驱动导出的全局状态 */
extern uint8 mt9v03x_init(void);          /* 返回 0=成功 1=失败 */
extern uint8 mt9v03x_lost_flag;           /* 0=摄像头在线 1=丢帧/未接 */

void system_init(void)
{
    lcd_init();
    key_init(10);
    lvgl_init();                        // LVGL 核心 + 屏幕 + 100ms 定时器
    pit_ms_init(CCU60_CH0, 10);         // 必须在 WIFI 初始化前启用：CCU60 中断里调 lv_tick_inc(10)，LVGL tick 才会走
    pit_ms_init(CCU60_CH1, 1000);
    g_ui.page = PAGE_INIT;              // 设当前页面为"初始化页"
    lvgl_demo();                        // 创建初始化页控件
    my_wifi_spi_init();                 // 跑 WIFI 初始化 1~4 阶段

    /* ---- 第 5 阶段：摄像头 MT9V03X 初始化 ---- */
    wifi_stage  = 5;
    wifi_result = 0;
    delay_with_refresh_ms(200);
    if (mt9v03x_init() != 0)
    {
        /* 摄像头 I2C 通讯失败（排线没插/模块供电异常） */
        wifi_result = 2;
        wifi_stage  = 255;
        delay_with_refresh_ms(800);
    }
    else if (mt9v03x_lost_flag != 0)
    {
        /* 初始化返回成功但丢帧标志还 1（I2C 配成功但帧 DMA 没拿到） */
        wifi_result = 2;
        wifi_stage  = 255;
        delay_with_refresh_ms(800);
    }
    else
    {
        wifi_result = 1;
        delay_with_refresh_ms(200);

        /* ---- 第 6 阶段：全部初始化完成 ---- */
        wifi_stage  = 6;
        wifi_result = 0;
        delay_with_refresh_ms(200);
        wifi_result = 1;
        delay_with_refresh_ms(200);
    }

    delay_with_refresh_ms(600);         // 给用户看最终结果（CAM OK / ALL READY / FAIL），再切主界面
    ui_switch_page(PAGE_DEMO);          // 切到主界面（2张图 + 标题 + 菜单）
}
