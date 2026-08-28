/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 在 GPL 第3版（即 GPL3.0）下，可以选择适用的任何版本，重新发布和/或修改此库
*
* TC264 开源库 的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或特定用途的保证
* 更多的细节请参见 GPL
*
* 您应该在收到 TC264 开源库 的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上为许可协议的中文参考翻译
* 许可协议的英文原文在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即 LICENSE 文件
* 欢迎各位使用并传播此程序 但修改内容时必须保留逐飞科技的版权声明
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹下 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "lvgl_demo.h"
#include "wifi_spi.h"
#include "image.h"

#pragma section all "cpu0_dsram"
// 本文件内（#pragma section all restore）之间的所有变量将被分配到 CPU0 的 RAM 中

// 以下区域是可扩展区域  完成用户所需的代码逻辑或测试功能等函数
// 以下区域是可扩展区域  完成用户所需的代码逻辑或测试功能等函数
// 以下区域是可扩展区域  完成用户所需的代码逻辑或测试功能等函数

/* mt9v03x 摄像头驱动全局状态 */

// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();
    debug_init();
    system_init();                     // 内部已做：LVGL初始化 + WIFI初始化 + 摄像头初始化 + 切到主界面

    cpu_wait_event_ready();
    while (TRUE)
    {
        if (mt9v03x_finish_flag)
        {
            image_threshold(mt9v03x_image); 
        }
        display_draw();
        if (g_ui.power_on == POWER_ON) {
            lv_task_handler();
            system_delay_ms(5);
        } else {
            system_delay_ms(1);          /* 关屏状态：最多 delay 1ms，CPU 全力跑业务 */
        }
    }

}

#pragma section all restore

// **************************** 代码区域 ****************************
