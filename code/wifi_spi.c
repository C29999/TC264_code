#include "wifi_spi.h"

#include "math.h"
#include <stdio.h>
#define PAI 3.1415926f
#define AMPLITUDE 100
#define STEP 0.1f

static float sin_phase = 0.0f;
static char tx_buf[48] __attribute__((unused));
static uint8 wifi_send_ready = 0;  
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
    if (wifi_spi_init(NULL, NULL) != 0)
    {
        show_center("WIFI SPI Error");
        system_delay_ms(500);
        return;
    }
    else
    {
        show_center("WIFI SPI Success");
        system_delay_ms(500);
    }
    if (wifi_spi_wifi_connect("chun", "12345678") == 0)
        {
            show_center("WIFI Connected Success");
            system_delay_ms(500); 
        }
        else
        {
            show_center("WIFI Connecting fail");
            system_delay_ms(500);
            return;
        }
    system_delay_ms(200);
    if (wifi_spi_socket_connect("TCP", "192.168.243.248", "8080", "6060") != 0)
    {
        show_center("TCP Connecting fail");
        system_delay_ms(500);
        return;
    }
    else
    {
        show_center("TCP Connecting success");
        system_delay_ms(500);
    }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    wifi_send_ready = 1;    //全部初始化成功，允许发数据
}
void wifi_image_send(void)
{
    static seekfree_assistant_camera_struct camera_obj;
    static uint8 camera_configured = 0;
    if (!wifi_send_ready)
    {
        return;
    }
    /* 全图：尺寸和缓冲固定，只需配置一次 */
    if (!camera_configured)
    {
        seekfree_assistant_camera_config(&camera_obj,
            SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X, MT9V03X_W, MT9V03X_H, mt9v03x_image);
        camera_configured = 1;
    }
    seekfree_assistant_camera_send(&camera_obj);
}
void wifi_boundary_send(void)
{
    static seekfree_assistant_camera_boundary_struct boundary_l_obj;
    static seekfree_assistant_camera_boundary_struct boundary_r_obj;
    if (!wifi_send_ready)
    {
        return;
    }
    /* 左边线：红色 */
    if (left_line_count > 0)
    {
        seekfree_assistant_camera_boundary_config(&boundary_l_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT16, 0xF800,
            (uint16)left_line_count, left_line_points);
        seekfree_assistant_camera_boundary_send(&boundary_l_obj);
    }
    /* 右边线：蓝色 */
    if (right_line_count > 0)
    {
        seekfree_assistant_camera_boundary_config(&boundary_r_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT16, 0x001F,
            (uint16)right_line_count, right_line_points);
        seekfree_assistant_camera_boundary_send(&boundary_r_obj);
    }
}
void wifi_debug(void)
{
 //  wifi_image_send();
    wifi_boundary_send();
}
