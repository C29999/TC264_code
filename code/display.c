#include "display.h"

page_t current_page = PAGE_MAIN;
uint8 main_select = 0;       // 0：常用 data，1：调参
uint8 page_changed = 1;

#define BOLD_TEXT_MAX_LENGTH (20)
#define BOLD_TEXT_BUFFER_WIDTH (BOLD_TEXT_MAX_LENGTH * 8 + 1)
static uint16 bold_text_buffer[BOLD_TEXT_BUFFER_WIDTH * 16];
static page_t last_display_page = PAGE_MAIN;

void lcd_init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
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

    /* The buffer is packed to text_width because the RGB565 API has no stride. */
    ips200_show_rgb565_image(x, y, bold_text_buffer,
                             text_width, 16, text_width, 16, 0);
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

void display_drow(void)
{
    diplay_main_key();
    if(current_page == PAGE_MAIN)
    {
        display_main_drow();
        if (mt9v03x_finish_flag)
        {
            ips200_show_gray_image(0,0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, 126, 80, 0);
            ips200_show_gray_image(127,0, (const uint8 *)image_binary[0], MT9V03X_W, MT9V03X_H, 110, 80, 0);

            if(wifi_ok_flag) show_red_bold(88, 108, "wifi ok", RGB565_GREEN);
            else show_red_bold(88, 108, "wifi fail", RGB565_RED);
           // mt9v03x_finish_flag = 0;
        }
    }
    else if(current_page == PAGE_DATA)
    {
        if (last_display_page != PAGE_DATA)
        {
            show_red_bold(106, 88, "data", RGB565_RED);
        }

        if (mt9v03x_finish_flag)
        {
            ips200_show_gray_image(0,0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, 126, 80, 0);
            ips200_show_gray_image(127,0, (const uint8 *)image_binary[0], MT9V03X_W, MT9V03X_H, 110, 80, 0);
          //  mt9v03x_finish_flag = 0;
        }
    }
    else if(current_page == PAGE_TUNING)
    {
        show_red_bold(106, 88, "tuning", RGB565_RED);
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

void diplay_main_key(void)
{
    if (current_page == PAGE_MAIN)
    {
        // KEY1: select data or tuning.
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            main_select++;
            if (main_select > 1)
            {
                main_select = 0;
            }

            page_changed = 1;
            key_clear_state(KEY_1);
        }

        // KEY2: confirm the current selection.
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
        {
            if (main_select == 0)
            {
                current_page = PAGE_DATA;
            }
            else
            {
                current_page = PAGE_TUNING;
            }

            page_changed = 1;
            key_clear_state(KEY_2);
        }
    }
    else if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        // KEY4: return to the main page.
        current_page = PAGE_MAIN;
        page_changed = 1;
        key_clear_state(KEY_4);
    }

    if (page_changed)
    {
        ips200_clear();
        page_changed = 0;
    }
}
