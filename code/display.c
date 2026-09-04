#include "display.h"
#include "pid.h"
#include "data.h"

//前向声明：这些static函数定义在display_draw()后面，ctc编译器必须先声明才能调用
static void display_main_drow(void);
static void tuning_draw_menu(void);
static void tuning_draw_edit(void);

page_t current_page = PAGE_MAIN;
uint8 main_select = 0;       // 0：常用 data，1：调参
uint8 page_changed = 1;
char show_buf[21];
#define BOLD_TEXT_MAX_LENGTH (20)
#define BOLD_TEXT_BUFFER_WIDTH (BOLD_TEXT_MAX_LENGTH * 8 + 1)
static uint16 bold_text_buffer[BOLD_TEXT_BUFFER_WIDTH * 16];
static page_t last_display_page = PAGE_MAIN;

#define TUNING_MENU_NUM     (2)     //调参一级目录数量：0=PID, 1=SPEED
#define PID_PARAM_NUM       (10)    //PID编辑页参数数
#define SPEED_PARAM_NUM     (3)     //SPEED编辑页参数数
static uint8 tuning_level  = 0;    //0=目录态，1=参数编辑态
static uint8 tuning_select = 0;    //目录态：当前目录项
static uint8 param_cursor = 0;     //编辑态：当前参数项

//==================== 水墨开机动画 ====================
#define INK_CH_MAX      (9)         // 单句最大字数
#define INK_CHAR_H      (16)        // 字模高度（像素）
#define INK_LEVELS      (7)         // 灰度级数：0=纸白(看不见) 6=墨黑
#define INK_STEP_MS     (40)        // 每级灰度停留时间
#define INK_HOLD_MS     (800)       // 墨色最深时的停留时间

// 灰度渐变表：纸白 -> 墨黑（RGB565）
static const uint16 ink_level[INK_LEVELS] =
{
    0xFFFF, 0xD69A, 0xAE75, 0x8631, 0x5ECD, 0x3689, 0x18C3
};

// sentence1: 二十二届智能车竞赛
static const uint8 ink_font1[9][32] =
{
    {/*二*/ 0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00},
    {/*十*/ 0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0xFF,0xFE,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00},
    {/*二*/ 0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00},
    {/*届*/ 0x1F,0xFC,0x10,0x04,0x10,0x04,0x1F,0xFC,0x10,0x40,0x10,0x40,0x10,0x40,0x17,0xFC,0x14,0x44,0x14,0x44,0x17,0xFC,0x24,0x44,0x24,0x44,0x47,0xFC,0x04,0x04,0x00,0x00},
    {/*智*/ 0x10,0x00,0x1F,0x3E,0x24,0x22,0x04,0x22,0x7F,0xA2,0x0A,0x22,0x11,0x3E,0x20,0x00,0x0F,0xF8,0x08,0x08,0x08,0x08,0x0F,0xF8,0x08,0x08,0x08,0x08,0x0F,0xF8,0x08,0x08},
    {/*能*/ 0x10,0x40,0x24,0x44,0x42,0x48,0xFF,0x70,0x01,0x40,0x00,0x42,0x7E,0x42,0x42,0x3E,0x42,0x00,0x7E,0x44,0x42,0x48,0x42,0x70,0x7E,0x40,0x42,0x42,0x4A,0x42,0x44,0x3E},
    {/*车*/ 0x02,0x00,0x02,0x00,0x02,0x00,0x7F,0xFC,0x04,0x00,0x09,0x00,0x11,0x00,0x21,0x00,0x3F,0xF8,0x01,0x00,0x01,0x00,0xFF,0xFE,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00},
    {/*竞*/ 0x02,0x00,0x01,0x00,0x3F,0xF8,0x08,0x20,0x04,0x40,0xFF,0xFE,0x00,0x00,0x1F,0xF0,0x10,0x10,0x10,0x10,0x1F,0xF0,0x04,0x40,0x04,0x40,0x08,0x42,0x30,0x42,0xC0,0x3E},
    {/*赛*/ 0x01,0x00,0x7F,0xFE,0x44,0x42,0x9F,0xF4,0x04,0x40,0x3F,0xF8,0x04,0x40,0xFF,0xFE,0x08,0x20,0x1F,0xF0,0x29,0x28,0xC9,0x26,0x09,0x20,0x0A,0xA0,0x04,0x40,0x18,0x20},
};

// sentence2: 全力以赴，不留遗憾
static const uint8 ink_font2[9][32] =
{
    {/*全*/ 0x01,0x00,0x01,0x00,0x02,0x80,0x04,0x40,0x08,0x20,0x10,0x10,0x2F,0xE8,0xC1,0x06,0x01,0x00,0x01,0x00,0x1F,0xF0,0x01,0x00,0x01,0x00,0x01,0x00,0x7F,0xFC,0x00,0x00},
    {/*力*/ 0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x3F,0xFC,0x01,0x04,0x01,0x04,0x01,0x04,0x01,0x04,0x02,0x04,0x02,0x04,0x04,0x04,0x04,0x04,0x08,0x44,0x10,0x28,0x20,0x10},
    {/*以*/ 0x00,0x20,0x08,0x20,0x44,0x20,0x42,0x20,0x42,0x20,0x40,0x20,0x40,0x20,0x40,0x40,0x40,0x40,0x40,0x40,0x48,0x80,0x50,0xA0,0x61,0x10,0x42,0x08,0x04,0x04,0x08,0x04},
    {/*赴*/ 0x04,0x20,0x04,0x20,0x04,0x20,0x3F,0x20,0x04,0x30,0x04,0x28,0x7F,0xA4,0x04,0x24,0x04,0x20,0x24,0x20,0x27,0xA0,0x24,0x20,0x24,0x20,0x54,0x00,0x4F,0xFE,0x80,0x00},
    {/*，*/ 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x80,0x01,0x80,0x00,0x80,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {/*不*/ 0x3F,0xFE,0x00,0x40,0x00,0x40,0x00,0x80,0x00,0x80,0x01,0xA0,0x02,0x90,0x04,0x88,0x08,0x84,0x10,0x82,0x20,0x82,0x40,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x00},
    {/*留*/ 0x06,0x00,0x78,0xFC,0x40,0x44,0x48,0x44,0x44,0x44,0x5A,0x94,0x61,0x08,0x00,0x00,0x3F,0xF8,0x21,0x08,0x21,0x08,0x3F,0xF8,0x21,0x08,0x21,0x08,0x3F,0xF8,0x20,0x08},
    {/*遗*/ 0x00,0x40,0x23,0xF8,0x12,0x48,0x13,0xF8,0x00,0x40,0x0F,0xFE,0xF0,0x00,0x13,0xF8,0x12,0x08,0x12,0x48,0x12,0x48,0x10,0xA0,0x11,0x10,0x2A,0x08,0x47,0xFE,0x00,0x00},
    {/*憾*/ 0x20,0x14,0x20,0x12,0x27,0xFE,0x24,0x10,0x35,0xD0,0xAC,0x14,0xA5,0xD4,0xA5,0x58,0xA5,0xCA,0x24,0x16,0x24,0x42,0x28,0x24,0x22,0xA2,0x22,0x8A,0x24,0x78,0x20,0x00},
};

// sentence3: 剑指国一
static const uint8 ink_font3[4][32] =
{
    {/*剑*/ 0x04,0x02,0x04,0x02,0x0A,0x02,0x09,0x12,0x10,0x92,0x20,0x52,0x5F,0x12,0x00,0x12,0x08,0x92,0x04,0x92,0x24,0x92,0x11,0x12,0x11,0x02,0x03,0xC2,0x3C,0x0A,0x10,0x04},
    {/*指*/ 0x11,0x00,0x11,0x04,0x11,0x38,0x11,0xC0,0xFD,0x02,0x11,0x02,0x10,0xFE,0x14,0x00,0x19,0xFC,0x31,0x04,0xD1,0x04,0x11,0xFC,0x11,0x04,0x11,0x04,0x51,0xFC,0x21,0x04},
    {/*国*/ 0x7F,0xFC,0x40,0x04,0x40,0x04,0x5F,0xF4,0x41,0x04,0x41,0x04,0x4F,0xE4,0x41,0x04,0x41,0x44,0x41,0x24,0x5F,0xF4,0x40,0x04,0x40,0x04,0x7F,0xFC,0x40,0x04,0x00,0x00},
    {/*一*/ 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};

// 整帧显存：每字16px宽+1px间隔，共16行
static uint16 ink_buf[INK_CH_MAX * 17 * 16];

// 将一句字模按指定"墨色"整帧重绘到屏幕中央（阴码逐行式，MSB在左）
static void ink_draw_frame(const uint8 font[][32], uint16 ch_num, uint16 ink_w, uint16 ink_color)
{
    uint16 x0;
    uint16 y0;
    uint16 ci;
    uint16 row;
    uint8  bit;
    uint8  byte_h;
    uint8  byte_l;
    uint16 idx = 0;

    x0 = (ips200_width_max  > ink_w) ? (uint16)((ips200_width_max  - ink_w) / 2) : 0;
    y0 = (ips200_height_max > INK_CHAR_H) ? (uint16)((ips200_height_max - INK_CHAR_H) / 2) : 0;

    for (row = 0; row < INK_CHAR_H; row++)              // 逐行拼出整帧RGB565显存
    {
        for (ci = 0; ci < ch_num; ci++)
        {
            byte_h = font[ci][row * 2];                 // 该行左半字节
            byte_l = font[ci][row * 2 + 1];             // 该行右半字节
            for (bit = 0; bit < 8; bit++)               // MSB对应最左像素
            {
                ink_buf[idx++] = (byte_h & (uint8)(0x80 >> bit)) ? ink_color : RGB565_WHITE;
            }
            for (bit = 0; bit < 8; bit++)
            {
                ink_buf[idx++] = (byte_l & (uint8)(0x80 >> bit)) ? ink_color : RGB565_WHITE;
            }
            ink_buf[idx++] = RGB565_WHITE;              // 字间1px留白
        }
    }
    ips200_show_rgb565_image(x0, y0, ink_buf, ink_w, INK_CHAR_H, ink_w, INK_CHAR_H, 0);
}

// 水墨浅入浅出动画：整句由纸白渐显为墨色，停留后再渐隐回纸白
static void show_ink_ani(const uint8 font[][32], uint16 ch_num)
{
    uint16 ink_w = (uint16)(ch_num * 17);               // 16px字宽+1px间隔
    uint8  lv;

    for (lv = 0; lv < INK_LEVELS; lv++)                 // 浅入：纸白 -> 墨黑
    {
        ink_draw_frame(font, ch_num, ink_w, ink_level[lv]);
        system_delay_ms(INK_STEP_MS);
    }
    system_delay_ms(INK_HOLD_MS);                       // 最深时停留
    for (lv = INK_LEVELS - 1; lv > 0; lv--)             // 浅出：墨黑 -> 纸白
    {
        ink_draw_frame(font, ch_num, ink_w, ink_level[lv - 1]);
        system_delay_ms(INK_STEP_MS);
    }
}

void lcd_init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_clear();
}

// 开机水墨动画：在所有初始化（摄像头、WIFI 等）结束后调用
void show_boot_ani(void)
{
    show_ink_ani(ink_font1, 9);
    show_ink_ani(ink_font2, 9);
    show_ink_ani(ink_font3, 4);
    ips200_clear();
}

static void show_red_bold(uint16 x, uint16 y, const char *text, uint16 color)
{
    uint16 text_length;
    uint16 text_width;
    uint16 char_index;
    uint8 row;
    uint8 col;
    uint8 font_top;
    uint8 font_bottom;
    uint16 pixel_x;

    text_length = (uint16)strlen(text);
    if (text_length > BOLD_TEXT_MAX_LENGTH)
    {
        text_length = BOLD_TEXT_MAX_LENGTH;
    }

    text_width = (uint16)(text_length * 8 + 1);
    for (row = 0; row < 16; row++)
    {
        for (col = 0; col < text_width; col++)
        {
            bold_text_buffer[row * text_width + col] = RGB565_WHITE;
        }
    }

    for (char_index = 0; char_index < text_length; char_index++)
    {
        if ((text[char_index] < 32) || (text[char_index] > 126))
        {
            continue;
        }
        for (col = 0; col < 8; col++)
        {
            pixel_x = (uint16)(char_index * 8 + col);
            font_top = ascii_font_8x16[(uint8)text[char_index] - 32][col];
            font_bottom = ascii_font_8x16[(uint8)text[char_index] - 32][col + 8];

            for (row = 0; row < 8; row++)
            {
                if (font_top & (1 << row))
                {
                    bold_text_buffer[row * text_width + pixel_x] = color;
                    bold_text_buffer[row * text_width + pixel_x + 1] = color;
                }

                if (font_bottom & (1 << row))
                {
                    bold_text_buffer[(row + 8) * text_width + pixel_x] = color;
                    bold_text_buffer[(row + 8) * text_width + pixel_x + 1] = color;
                }
            }
        }
    }
    ips200_show_rgb565_image(x, y, bold_text_buffer,text_width, 16, text_width, 16, 0);
}
void show_center(const char *text)
{
    uint16 text_width;
    uint16 x;
    uint16 y;

    // The current font is IPS200_8X16_FONT.
    text_width = (uint16)(strlen(text) * 8);
    x = (ips200_width_max > text_width) ?
        (uint16)((ips200_width_max - text_width) / 2) : 0;
    y = (ips200_height_max > 16) ?
        (uint16)((ips200_height_max - 16) / 2) : 0;

    show_red_bold(x, y, text, RGB565_PINK);
}
//边线坐标→缩略图屏幕坐标映射（带截断，防单边补线时越界画花屏幕）
static uint16 edge_map_x(int16 x)
{
    if (x < 0)   x = 0;
    if (x > 187) x = 187;
    return 127 + (uint16)(x * 110 / 188);
}
static uint16 edge_map_y(int16 y)
{
    if (y < 0)   y = 0;
    if (y > 119) y = 119;
    return (uint16)(y * 80 / 120);
}
void show_draw_edges(void)
{
    const uint16 by = 0;     // 二值图显示区域原点 y
    uint16 i;

    // 图像中心竖线：黄色，画在二值图上（无论有没有边线都显示）
    ips200_draw_line(edge_map_x(image_center), by,
                     edge_map_x(image_center), by + 80,
                     RGB565_YELLOW);

    if (left_line_count == 0 && right_line_count == 0)
    {
        show_red_bold(146, 32, "NO POINTS", RGB565_PINK);
        return;
    }

    // 左边线：蓝色
    for(i=1;i<left_line_count;i++)
    {
        ips200_draw_line(edge_map_x(left_line_points[i-1][0]), edge_map_y(left_line_points[i-1][1]),
                         edge_map_x(left_line_points[i][0]),   edge_map_y(left_line_points[i][1]),
                         RGB565_BLUE);
    }

    // 右边线：红色
    for(i=1;i<right_line_count;i++)
    {
        ips200_draw_line(edge_map_x(right_line_points[i-1][0]), edge_map_y(right_line_points[i-1][1]),
                         edge_map_x(right_line_points[i][0]),   edge_map_y(right_line_points[i][1]),
                         RGB565_RED);
    }

    // 中线：绿色（逐行对齐版：按y行取左右线x的中点，单边行用±TRACK_HALF_W虚拟）
    {
        int16 lx[MT9V03X_H], rx[MT9V03X_H];
        int16 y2, mx;
        int16 prev_x = -1, prev_y = -1;

        for (y2 = 0; y2 < MT9V03X_H; y2++) { lx[y2] = -1; rx[y2] = -1; }
        // 把边线点按行号展开(一行取第一次出现的x)
        for (i = 0; i < left_line_count; i++)
            if (left_line_points[i][1] >= 0 && left_line_points[i][1] < MT9V03X_H
                && lx[left_line_points[i][1]] < 0)
                lx[left_line_points[i][1]] = left_line_points[i][0];
        for (i = 0; i < right_line_count; i++)
            if (right_line_points[i][1] >= 0 && right_line_points[i][1] < MT9V03X_H
                && rx[right_line_points[i][1]] < 0)
                rx[right_line_points[i][1]] = right_line_points[i][0];

        // 从近端(底部)往远端逐行连线
        for (y2 = MT9V03X_H - 1; y2 >= 0; y2--)
        {
            if (lx[y2] >= 0 && rx[y2] >= 0)     mx = (lx[y2] + rx[y2]) / 2;   //双边:真中点
            else if (lx[y2] >= 0)               mx = lx[y2] + TRACK_HALF_W;   //仅左:虚拟
            else if (rx[y2] >= 0)               mx = rx[y2] - TRACK_HALF_W;   //仅右:虚拟
            else { prev_x = -1; continue; }     //该行无线:断开,下段重新起笔

            if (prev_x >= 0)
                ips200_draw_line(edge_map_x(prev_x), edge_map_y(prev_y),
                                 edge_map_x(mx), edge_map_y(y2), RGB565_GREEN);
            prev_x = mx;
            prev_y = y2;
        }
    }
}
void display_draw(void)
{
    diplay_key_control();
    if (mt9v03x_finish_flag)
    {
        ips200_show_gray_image(0,0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, 126, 80, 0);
        ips200_show_gray_image(127,0, (const uint8 *)image_binary[0], MT9V03X_W, MT9V03X_H, 110, 80, 0);
        show_draw_edges();   // 图和边线同帧：刷完图立刻画线，每帧只画一次
    }
    if(current_page == PAGE_MAIN)
    {
        display_main_drow();
        sprintf(show_buf,"fps:%d",fps);
        show_red_bold(108,108, show_buf, RGB565_PINK);
        sprintf(show_buf,"center:%03d",image_center);
        show_red_bold(108,128, show_buf, RGB565_PINK);
        sprintf(show_buf,"mid:%03d",mid);
        show_red_bold(108,148, show_buf, RGB565_PINK);
        sprintf(show_buf,"error:%03d",image_error_filter);
        show_red_bold(108,168, show_buf, RGB565_PINK);
    }
    else if(current_page == PAGE_DATA)
    {
        if (last_display_page != PAGE_DATA)
        {
            show_red_bold(106, 88, "data", RGB565_RED);
        }
        sprintf(show_buf,"encoder:%03d",encoder_left);
        show_red_bold(0,108, show_buf, RGB565_GREEN);
        sprintf(show_buf,"encoder:%03d",encoder_right);
        show_red_bold(0,128, show_buf, RGB565_GREEN);
        sprintf(show_buf,"speed:%03d",base_speed);
        show_red_bold(0,148, show_buf, RGB565_GREEN);
        sprintf(show_buf,"dif_val:%03d",dif_val);
        show_red_bold(0,168, show_buf, RGB565_GREEN);
        sprintf(show_buf,"l_goal:%03d",base_speed+dif_val);
        show_red_bold(0,188, show_buf, RGB565_GREEN);
        sprintf(show_buf,"r_goal:%03d",base_speed-dif_val);
        show_red_bold(0,208, show_buf, RGB565_GREEN);
        sprintf(show_buf,"l_out:%03d",(int)speed_l_pid.out);
        show_red_bold(0,228, show_buf, RGB565_GREEN);
        sprintf(show_buf,"r_out:%03d",(int)speed_r_pid.out);
        show_red_bold(0,248, show_buf, RGB565_GREEN);
        sprintf(show_buf,"error:%03d",image_error_filter);
        show_red_bold(0,268, show_buf, RGB565_GREEN);
        sprintf(show_buf,"pts:%03d/%03d",rpts0s_num,rpts1s_num);
        show_red_bold(108,108, show_buf, RGB565_CYAN);
        sprintf(show_buf,"ang:%.0f/%.0f",conf1_max*57.3f,conf2_max*57.3f);
        show_red_bold(108,128, show_buf, RGB565_YELLOW);
        sprintf(show_buf,"cL:%d cR:%d",Lpt0_found,Lpt1_found);
        show_red_bold(108,148, show_buf, RGB565_RED);
    }
    else if(current_page == PAGE_TUNING)
    {
        if (tuning_level == 0)  tuning_draw_menu();
        else                    tuning_draw_edit();
    }

    last_display_page = current_page;
}
void display_main_drow(void)
{
    //ips200_show_string(16, 20, "data");
    //ips200_show_string(16, 40, "tuning");

    show_red_bold(106, 88, "main", RGB565_RED);

    if(main_select == 0)
    {
        show_red_bold(0, 108, "-->data", RGB565_BLUE);
        show_red_bold(0, 128, "tuning", RGB565_BLUE);
}
    else
    {
        show_red_bold(0, 108, "data", RGB565_BLUE);
        show_red_bold(0, 128, "-->tuning", RGB565_BLUE);
    }
}

//==================== 调参页：参数加减辅助 ====================
//
static float pid_step(uint8 i)
{
    if (i == 1 || i == 4 || i == 7) return 0.05f;   //ki
    if (i == 3 || i == 6 || i == 9) return 0.05f;   //low_pass
    return 0.10f;                                    //kp/kd
}

//根据编辑页和项号，修改对应参数
static void param_apply_delta(int8 sign)
{
    float step;

    if (tuning_select == 0)        //PID页
    {
        step = pid_step(param_cursor) * (float)sign;
        switch (param_cursor)
        {
        case 0: angle_steer_pid.kp += step;           break;
        case 1: angle_steer_pid.ki += step;           break;
        case 2: angle_steer_pid.kd += step;           break;
        case 3: angle_steer_pid.low_pass += step;     break;
        case 4: angle_speed_pid.kp += step;           break;
        case 5: angle_speed_pid.kd += step;           break;
        case 6: speed_l_pid.kp += step;               break;
        case 7: speed_l_pid.ki += step;               break;
        case 8: speed_l_pid.kd += step;               break;
        case 9: speed_l_pid.low_pass += step;         break;
        }
        //同步速度环左右参数（保证两轮一致）
        if (param_cursor >= 6 && param_cursor <= 9)
        {
            switch (param_cursor)
            {
            case 6: speed_r_pid.kp = speed_l_pid.kp; break;
            case 7: speed_r_pid.ki = speed_l_pid.ki; break;
            case 8: speed_r_pid.kd = speed_l_pid.kd; break;
            case 9: speed_r_pid.low_pass = speed_l_pid.low_pass; break;
            }
        }
    }
    else if (tuning_select == 1)   //SPEED页
    {
        switch (param_cursor)
        {
        case 0:
            base_speed = (int16)((int32)base_speed + (int32)sign * 10);
            if (base_speed <   0) base_speed =   0;
            if (base_speed > 500) base_speed = 500;
            break;
        case 1:
            angle_steer_pid.out_max += (float)sign * 10.0f;
            if (angle_steer_pid.out_max < 10.0f) angle_steer_pid.out_max = 10.0f;
            angle_steer_pid.out_min = -angle_steer_pid.out_max;
            break;
        case 2:
            angle_speed_pid.out_max += (float)sign * 5.0f;
            if (angle_speed_pid.out_max < 5.0f) angle_speed_pid.out_max = 5.0f;
            angle_speed_pid.out_min = -angle_speed_pid.out_max;
            break;
        }
    }
    page_changed = 1;
}

//==================== 调参页按键处理 ====================
static void tuning_key(void)
{
    if (tuning_level == 0)        //目录态
    {
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS)   //KEY1：下一项(循环)
        {
            tuning_select++;
            if (tuning_select >= TUNING_MENU_NUM) tuning_select = 0;
            page_changed = 1;
            key_clear_state(KEY_1);
        }
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS)   //KEY2：进编辑态
        {
            tuning_level = 1;
            param_cursor = 0;
            page_changed = 1;
            key_clear_state(KEY_2);
        }
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS)   //KEY4：回主菜单
        {
            current_page = PAGE_MAIN;
            tuning_level = 0;
            page_changed = 1;
            key_clear_state(KEY_4);
        }
    }
    else                          //编辑态
    {
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS)   //KEY1：加
        {
            param_apply_delta(1);
            key_clear_state(KEY_1);
        }
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS)   //KEY2：减
        {
            param_apply_delta(-1);
            key_clear_state(KEY_2);
        }
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS)   //KEY3：下一项参数
        {
            uint8 max_n = (tuning_select == 0) ? PID_PARAM_NUM : SPEED_PARAM_NUM;
            param_cursor++;
            if (param_cursor >= max_n) param_cursor = 0;
            page_changed = 1;
            key_clear_state(KEY_3);
        }
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS)   //KEY4：退目录
        {
            tuning_level = 0;
            page_changed = 1;
            key_clear_state(KEY_4);
        }
    }
}

//==================== 总按键入口（对外唯一） ====================
void diplay_key_control(void)
{
    if (page_changed)
    {
        ips200_clear();
        page_changed = 0;
    }

    switch (current_page)
    {
    case PAGE_MAIN:
        //KEY1：切 data/tuning
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            main_select++;
            if (main_select > 1) main_select = 0;
            page_changed = 1;
            key_clear_state(KEY_1);
        }
        //KEY2：确认进入
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
        {
            current_page = (main_select == 0) ? PAGE_DATA : PAGE_TUNING;
            tuning_level = 0;
            tuning_select = 0;
            page_changed = 1;
            key_clear_state(KEY_2);
        }
        break;

    case PAGE_DATA:
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS)   //KEY4：回主菜单
        {
            current_page = PAGE_MAIN;
            page_changed = 1;
            key_clear_state(KEY_4);
        }
        break;

    case PAGE_TUNING:
        tuning_key();
        break;
    }
}

static void tuning_draw_menu(void)
{
    show_red_bold(106, 88, "tuning", RGB565_RED);
    if (tuning_select == 0)
    {
        show_red_bold(0, 108, "-->PID",         RGB565_PURPLE);
        show_red_bold(0, 128, "   SPEED LIMIT", RGB565_BLUE);
    }
    else
    {
        show_red_bold(0, 108, "   PID",         RGB565_BLUE);
        show_red_bold(0, 128, "-->SPEED LIMIT", RGB565_PURPLE);
    }
}

static void tuning_draw_pid(void)
{
    const uint8 y0 = 108;
    const uint8 dy = 20;
    uint8 hi;
    const char *name[PID_PARAM_NUM] = {
        "ags.kp:","ags.ki:","ags.Kd","ags:lp","asp:kp","asp:kd","motL:kp", "motL:ki", "motL:kd", "motL:lp"
    };
    float val[PID_PARAM_NUM] = {angle_steer_pid.kp, angle_steer_pid.ki, angle_steer_pid.kd, angle_steer_pid.low_pass,angle_speed_pid.kp, angle_speed_pid.kd,speed_l_pid.kp, speed_l_pid.ki, speed_l_pid.kd, speed_l_pid.low_pass};
    show_red_bold(106, 88, "PID TUNING", RGB565_RED);
    for (hi = 0; hi < PID_PARAM_NUM; hi++)
    {
        sprintf(show_buf, "%s %5.2f", name[hi], val[hi]);
        if (hi == param_cursor)
            show_red_bold(0, (uint16)(y0 + (uint16)hi * dy), show_buf, RGB565_GREEN);
        else
            show_red_bold(0, (uint16)(y0 + (uint16)hi * dy), show_buf, RGB565_PURPLE);
    }
}

static void tuning_draw_speed(void)
{
    show_red_bold(106, 88, "SPEED LIMIT", RGB565_RED);

    sprintf(show_buf, "base_speed %03d", base_speed);
    if (param_cursor == 0) show_red_bold(0, 108, show_buf, RGB565_GREEN);
    else                   show_red_bold(0, 108, show_buf, RGB565_PURPLE);
    sprintf(show_buf, "max_gyro  %4.0f", angle_steer_pid.out_max);
    if (param_cursor == 1) show_red_bold(0, 128, show_buf, RGB565_GREEN);
    else                   show_red_bold(0, 128, show_buf, RGB565_PURPLE);

    sprintf(show_buf, "max_dif   %4.0f", angle_speed_pid.out_max);
    if (param_cursor == 2) show_red_bold(0, 148, show_buf, RGB565_GREEN);
    else                   show_red_bold(0, 148, show_buf, RGB565_PURPLE);
}

static void tuning_draw_edit(void)
{
    if (tuning_select == 0) tuning_draw_pid();
    else                    tuning_draw_speed();
}
