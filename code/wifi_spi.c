#include "wifi_spi.h"
#include "math.h"
#include <stdio.h>
#define PAI 3.1415926f
#define AMPLITUDE 100
#define STEP      0.1f

static float sin_phase = 0.0f;
static char tx_buf[48];

static int SinWithPhase(float phase_offset)
{
    return (int)(AMPLITUDE * sin(sin_phase + phase_offset));
}

int Sin(void)
{
    int value;

    value = SinWithPhase(0.0f);

    sin_phase += STEP;
    if(sin_phase >= 2.0f * PAI)
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

    // Initialize the module only. This separates SPI hardware errors from
    // WiFi authentication errors.
    if (wifi_spi_init(NULL, NULL) != 0)
    {
        show_center("WIFI INIT FAIL");
        system_delay_ms(500);
        ips200_clear();
        return;
    }

    show_center("WIFI INIT OK");
    system_delay_ms(500);
    ips200_clear();

    // Connect to the 2.4 GHz hotspot. Retry several times in case the
    // module is still reconnecting to the access point.
    for (retry_count = 0; retry_count < 3; retry_count++)
    {
        if (wifi_spi_wifi_connect("chun", "12345678") == 0)
        {
            break;
        }

        show_center("WIFI RETRY");
        system_delay_ms(1000);
        ips200_clear();
    }

    if (retry_count >= 3)
    {
        show_center("WIFI FAIL");
        system_delay_ms(500);
        ips200_clear();
        return;
    }

    show_center("WIFI OK");
    system_delay_ms(500);
    ips200_clear();

    // Try once only. Do not block forever before the PC connects.
    if (wifi_spi_socket_connect("TCP", "192.168.179.248","8080","6060")!= 0)
    {
        show_center("TCP FAIL");
        system_delay_ms(500);
        ips200_clear();
        return;
    }
    else 
    {
        wifi_ok_flag = 1;
    }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    wifi_flag = 1;

    show_center("TCP OK");
    system_delay_ms(500);
    ips200_clear();
}
void wifi_image_send(void)
{
    static seekfree_assistant_camera_struct camera_obj;
    static uint8 camera_configured = 0;

    // Consume each completed camera frame once. The display can read the
    // frame before this function clears the flag in the main loop.
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
        seekfree_assistant_camera_config(
            &camera_obj,
            SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
            MT9V03X_W,
            MT9V03X_H,
            mt9v03x_image);
        camera_configured = 1;
    }

    seekfree_assistant_camera_send(&camera_obj);
    mt9v03x_finish_flag = 0;
}
void wifi_debug(void)
{
    // The display task calls wifi_image_send() after drawing the frame.
}
