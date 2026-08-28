#include "wifi_spi.h"
#include "lvgl_demo.h"
#include "math.h"
#include <stdio.h>
#define PAI 3.1415926f
#define AMPLITUDE 100
#define STEP 0.1f

static float sin_phase = 0.0f;
static char tx_buf[48] __attribute__((unused));

static int SinWithPhase(float phase_offset)
{
    return (int)(AMPLITUDE * sin(sin_phase + phase_offset));
}

int Sin(void)
{
    int value;

    value = SinWithPhase(0.0f);

    sin_phase += STEP;
    if (sin_phase >= 2.0f * PAI)
    {
        sin_phase -= 2.0f * PAI;
    }

    return value;
}

void my_wifi_spi_init(void)
{
    uint8 retry_count;

    wifi_ok_flag = 0;
    wifi_flag = 0;
    wifi_init_flag = 0;
    wifi_stage = 0;
    wifi_result = 0;

    /* ---- 第 1 阶段：SPI 初始化 ---- */
    wifi_stage = 1;
    wifi_result = 0;
    delay_with_refresh_ms(200);
    if (wifi_spi_init(NULL, NULL) != 0)
    {
        wifi_result = 2;
        wifi_stage = 255;
        delay_with_refresh_ms(500);
        return;
    }
    wifi_result = 1;
    delay_with_refresh_ms(200);

    /* ---- 第 2 阶段：WIFI 连接（最多重试 3 次）---- */
    wifi_stage = 2;
    wifi_result = 0;
    delay_with_refresh_ms(200);
    for (retry_count = 0; retry_count < 3; retry_count++)
    {
        if (wifi_spi_wifi_connect("chun", "12345678") == 0)
        {
            break;
        }
        delay_with_refresh_ms(500); // 重试等待也要刷新
    }

    if (retry_count >= 3)
    {
        wifi_result = 2;
        wifi_stage = 255;
        delay_with_refresh_ms(500);
        return;
    }
    wifi_result = 1;
    delay_with_refresh_ms(200);

    /* ---- 第 3 阶段：TCP 连接 ---- */
    wifi_stage = 3;
    wifi_result = 0;
    delay_with_refresh_ms(200);
    if (wifi_spi_socket_connect("TCP", "192.168.108.248", "8080", "6060") != 0)
    {
        wifi_result = 2;
        wifi_stage = 255;
        delay_with_refresh_ms(500);
        return;
    }
    else
    {
        wifi_ok_flag = 1;
    }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    wifi_flag = 1;
    wifi_result = 1;
    delay_with_refresh_ms(200);

    /* ---- 第 4 阶段：完成 ---- */
    wifi_stage = 4;
    wifi_init_flag = 1;
    delay_with_refresh_ms(200 + 500);
}
void wifi_image_send(void)
{
    static seekfree_assistant_camera_struct camera_obj;
    static uint8 camera_configured = 0;

    if (!mt9v03x_finish_flag)
    {
        return;
    }

    if (!wifi_flag)
    {
        mt9v03x_finish_flag = 0;
        return;
    }

    if (!camera_configured)
    {
        seekfree_assistant_camera_config(&camera_obj, SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X, MT9V03X_W, MT9V03X_H, mt9v03x_image);
        camera_configured = 1;
    }

    seekfree_assistant_camera_send(&camera_obj);
    mt9v03x_finish_flag = 0;
}
void wifi_debug(void)
{
}
