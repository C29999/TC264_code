/**
 * IPS200 display port for LVGL
 */

#define MY_DISP_HOR_RES    240
#define MY_DISP_VER_RES    320

#include "lv_port_disp.h"
#include "zf_device_ips200.h"
#include <stdbool.h>

static void disp_init(void);
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

void lv_port_disp_init(void)
{
    disp_init();

    static lv_disp_draw_buf_t draw_buf_dsc;
    static lv_color_t buf_1[MY_DISP_HOR_RES * MY_DISP_VER_RES / 10];
    lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, NULL, MY_DISP_HOR_RES * MY_DISP_VER_RES / 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = ips200_width_max;
    disp_drv.ver_res = ips200_height_max;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf_dsc;
    lv_disp_drv_register(&disp_drv);
}

static void disp_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_color(0x0000, 0xFFFF);
    ips200_clear();
}

volatile bool disp_flush_enabled = true;

void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(disp_flush_enabled) {
        uint16 width  = (uint16)(area->x2 - area->x1 + 1);
        uint16 height = (uint16)(area->y2 - area->y1 + 1);
        ips200_show_rgb565_image((uint16)area->x1, (uint16)area->y1,
                                 (const uint16 *)color_p,
                                 width, height, width, height, 0);
    }
    lv_disp_flush_ready(disp_drv);
}
