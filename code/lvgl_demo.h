#ifndef CODE_LVGL_DEMO_H_
#define CODE_LVGL_DEMO_H_

// Tell LVGL where to find lv_conf.h
#include "lvgl/lvgl.h"
#include "lv_port_disp.h"

/* 单模块内部的进度：比如 WIFI 内部 6 小步 / IMU 内部 3 小步 */
typedef struct
{
    char text[64];   /* 模块当前显示的文字，如 "WIFI INIT OK" */
    int  step;       /* 模块内第几步：1~total */
    int  total;      /* 模块一共几步 */
    int  ok;         /* 1=本模块成功(显示绿) 0=失败(显示红) -1=进行中(粉) */
} ui_init_module_t;

/* 单个模块槽位（用于"模块结果列表"显示：✓/✗/○）*/
typedef struct
{
    char name[24];   /* 模块名："WIFI" "IMU" "MOTOR" ... */
    int  done;       /* 0=未开始(○) 1=成功(✓) -1=失败(✗) */
} ui_init_module_slot_t;

/* ========== 通用"初始化总界面"状态（支持 N 个模块） ==========
 * 用法：开机启动时调 ui_init_begin(N) 告诉系统一共要做 N 个模块初始化；
 * 然后每个模块开始时 ui_init_set_module("WIFI", 6)，
 * 模块内部每一小步 ui_init_set_step("WIFI SPI OK", 1)，
 * 模块结束时 ui_init_set_result(1)  1=成功 0=失败。
 * 自动推进"模块大进度条"百分比，并在模块列表里打勾/打叉。
 * ============================================================ */
#define UI_INIT_MAX_MODULES 8    /* 最多 8 个模块：WIFI / IMU / MOTOR / ENCODER / ... */
typedef struct
{
    /* 正在跑的"当前模块" */
    char cur_module[32];           /* 当前模块名，显示在标题下方 "Init Module : WIFI" */
    ui_init_module_t cur;          /* 当前模块的内部进度 / 文字 / 成功与否 */

    /* 模块级"大进度" */
    int total_modules;             /* 总共要做几个模块（开机已知）*/
    int done_modules;              /* 已做完成败都算 +1 */

    /* 每个模块的最终结果（模块列表显示用）*/
    ui_init_module_slot_t modules[UI_INIT_MAX_MODULES];  /* "WIFI"/"IMU"... + done 状态 */
} ui_init_page_t;

/* ========== 主演示页（PAGE_DEMO：图像+菜单+状态）========== */
#define DEMO_SUB_MAIN   0
#define DEMO_SUB_DATA   1
#define DEMO_SUB_TUNING 2
typedef struct
{
    int  sub_page;        /* DEMO_SUB_MAIN / DEMO_SUB_DATA / DEMO_SUB_TUNING */
    int  main_select;     /* MAIN 子页菜单：0=data / 1=tuning */
    int  wifi_ok;         /* 1=wifi ok 0=wifi fail */
    int  fps_val;         /* 帧率 */
    int  img_ready;       /* 新的一帧灰度+二值图是否就绪 */
    int  sub_page_changed;/* 子页刚切换（只在切换那一帧清屏用的内部标志）*/
} ui_demo_page_t;

typedef struct
{
    int page;//显示页面
    int power_on;//屏幕开关 1=开 0=关(省算力)
    #define PAGE_NONE        0//黑屏
    #define PAGE_WIFI_INIT  1 //
    #define PAGE_INIT       1 //初始化总界面（N 个模块，兼容 WIFI_INIT）
    #define PAGE_DEMO       2 //演示界面：图像+菜单+状态
    #define POWER_ON 1
    #define POWER_OFF 0

    //每个页面的状态
    ui_init_page_t init;
    ui_demo_page_t demo;
}ui_state_t;

extern ui_state_t g_ui;//全局唯一变量
void delay_with_refresh_ms(int ms);
void lvgl_init(void);
void lvgl_demo(void);

void  ui_set_power(int on);//屏幕开关
void  ui_switch_page(int new_page);//切页面：清屏+改page+建新控件

/* ========== 通用"初始化总界面"对外接口 ==========
 * 业务文件（wifi_spi.c、未来的 imu.c/motor.c）只调这些，不碰 LVGL。 */
void ui_init_begin(int total_modules);                               /* 开机调一次，告诉系统共几个模块 */
void ui_init_set_module(const char *module_name, int module_steps);  /* 每个模块开始时调，如 ("WIFI", 6) */
void ui_init_set_step (const char *step_text, int step_index);       /* 模块内每一步调一次 */
void ui_init_set_result(int success);                                /* 模块结束时调：1=ok 0=fail → 自动推进大进度条 */


/* ========== 主演示页对外接口（业务文件只调这些，不碰 LVGL） ========== */
void ui_demo_set_sub_page     (int sub_page);   /* DEMO_SUB_MAIN / DEMO_SUB_DATA / DEMO_SUB_TUNING */
void ui_demo_set_main_select  (int sel);        /* MAIN 子页里的选中项：0=data / 1=tuning */
void ui_demo_set_wifi_ok      (int ok);         /* 1=ok 0=fail */
void ui_demo_set_fps          (int fps);        /* 帧率 */
void ui_demo_set_img_ready    (int ready);      /* 1=新帧就绪 0=已消费(刷新后内部置 0) */

#endif
