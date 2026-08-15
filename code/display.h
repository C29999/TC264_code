#ifndef DISPLAY_H
#define DISPLAY_H
#include "zf_common_headfile.h"
typedef enum
{
    PAGE_MAIN,      //
    PAGE_DATA,      // 
    PAGE_TUNING      // 
} page_t;
extern page_t current_page;
extern uint8 page_changed;
void lcd_init(void);
void show_center(const char *text);
void display_drow(void);
void display_main_drow(void);
void diplay_main_key(void);
#endif
