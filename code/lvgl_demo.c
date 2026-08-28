#include "lvgl_demo.h"
#include "lv_port_disp.h"
#include "data.h"          /* 读 wifi_stage / wifi_init_flag 等标志位 */
#include "image.h"         /* image_binary[H][W] */

/* 注意：本文件所有全局变量 / 静态对象默认进 .data/.bss，会落在 TC264 CPU0 DSARAM(0x60000000~6001E000)
 * 这块只有 120KB，因此 LV_MEM_SIZE 要小，不用的 LV_USE_* 控件必须在 lv_conf.h 里关 0。 */

/* ========================================================================
 * Data-Driven UI 核心层
 * 业务文件只改 g_ui，本文件 100ms 定时器自动读 g_ui → 刷屏幕
 * ======================================================================== */

/* -------- ① 全局共享状态实例化（.h 里 extern 的那个 g_ui 在这里定义） -------- */
ui_state_t g_ui = {
    .page     = PAGE_NONE,
    .power_on = POWER_ON,
    .init     = {
        .cur_module    = "",
        .cur           = { .text = "", .step = 0, .total = 0, .ok = -1 },
        .total_modules = 0,
        .done_modules  = 0,
        .modules       = {0},   /* 8 个模块槽位全部清零(name="", done=0) */
    },
    .demo     = {
        .sub_page         = DEMO_SUB_MAIN,
        .main_select      = 0,
        .wifi_ok          = 0,
        .fps_val          = 0,
        .img_ready        = 0,
        .sub_page_changed = 1,
    },
};

static lv_obj_t *s_wifi_title;       /* 顶部大标题 */
static lv_obj_t *s_wifi_status;      /* 中间粉色大字 */
static lv_obj_t *s_wifi_step_lbl;    /* 模块内步骤小字 */
static lv_obj_t *s_wifi_bar;         /* 模块内小进度条 */
static lv_timer_t *s_ui_refresh_tmr; /* 100ms 自动刷新引擎 */

/* -------- PAGE_DEMO 控件（主演示页：图像+菜单+状态） -------- */
static lv_obj_t *s_demo_box_gray;    /* 左图容器（蓝色边框画在它上面） */
static lv_obj_t *s_demo_box_binary;  /* 右图容器 */
static lv_obj_t *s_demo_img_gray;    /* 灰度图 img（容器子对象） */
static lv_obj_t *s_demo_img_binary;  /* 二值图 img（容器子对象） */
static lv_obj_t *s_demo_title_gray;   /* 左图上方标题 "gray" */
static lv_obj_t *s_demo_title_binary; /* 右图上方标题 "binary" */

/* 灰度 / 二值图的 LVGL 图像描述符（LV_IMG_CF_ALPHA_8BIT：复用 uint8 原数组，不转 RGB565） */
static lv_img_dsc_t s_demo_gray_dsc;
static lv_img_dsc_t s_demo_binary_dsc;
/* 前向声明：ui_refresh_cb 在 lvgl_init 里被引用，但定义在后面 */
static void ui_refresh_cb(lv_timer_t *tmr);

/* 前向声明：页面创建/刷新函数（lvgl_demo 和 ui_refresh_cb 里引用，定义在文件末尾）*/
static void page_init_create(void);    /* 初始化页：创建控件（下一轮填实现）*/
static void page_init_refresh(void);   /* 初始化页：读 g_ui 刷新控件（下一轮填实现）*/
static void page_demo_create(void);    /* 演示页：创建图像+标题+菜单+状态标签 */
static void page_demo_refresh(void);   /* 演示页：读 g_ui.demo 刷新图像/菜单/状态 */

/* lvgl_init：LVGL 核心 + 屏幕 + 启动 100ms 刷新引擎 */
void lvgl_init(void)
{
    lv_init();
    lv_port_disp_init();
    /* 启动 Data-Driven UI 的 100ms 自动刷新引擎 */
    s_ui_refresh_tmr = lv_timer_create(ui_refresh_cb, 100, NULL);
}

/* demo_timer_cb 已删除（旧 page_demo 已不再有 label/slider）。*/

/* -------- ⑤ 100ms 自动刷新引擎：根据 g_ui.page 决定刷新哪个页面 -------- */
static void ui_refresh_cb(lv_timer_t *tmr)
{
    (void)tmr;

    /* 关屏状态：直接 return，省算力 */
    if (g_ui.power_on == POWER_OFF) return;

    /* 根据当前页面调对应的刷新函数 */
    switch (g_ui.page) {
        case PAGE_INIT:  page_init_refresh();  break;  /* 初始化页：读 g_ui.init 刷控件 */
        case PAGE_DEMO:  page_demo_refresh(); break;  /* 演示页：读 g_ui.demo 刷图像+菜单+状态 */
        default: break;
    }
}


/* ========================================================================
 * 对外接口 1：ui_init_begin —— 开机调一次，告诉系统共几个模块
 * ======================================================================== */
void ui_init_begin(int total_modules)
{
    if(total_modules<1) total_modules=1;                            // 至少 1 个模块，防止 0 或负数
    if(total_modules>UI_INIT_MAX_MODULES) total_modules=UI_INIT_MAX_MODULES;  // 不超过最大容量(8)，防数组越界

    /* ---------- 把"模块总数"和"已完成数"写进全局状态 ---------- */
    g_ui.init.total_modules=total_modules;   // 记录模块总数：用于算大进度条百分比 done/total
    g_ui.init.done_modules=0;                // 已完成的模块数：每调一次 ui_init_set_result 自动 +1

    /* ---------- 全部模块结果清空初始化 ---------- */
    for(int i=0;i<total_modules;i++)
    {
        g_ui.init.modules[i].done=0;         // 第 i 个模块：0=未开始(显示 ○)，1=成功(✓)，-1=失败(✗)
        g_ui.init.modules[i].name[0]='\0';   // 模块名清空
    }

    /* ---------- 当前模块字段清零 ---------- */
    g_ui.init.cur_module[0] = '\0';          // 当前模块名字符串清空(空字符串)
    g_ui.init.cur.text[0]   = '\0';          // 当前模块正在显示的文字清空(如 "WIFI SPI OK")
    g_ui.init.cur.step      = 0;             // 模块内部当前步骤号清零(还没开始)
    g_ui.init.cur.total     = 0;             // 模块内部总步骤数清零(还没设置)
    g_ui.init.cur.ok        = -1;            // 当前模块结果：-1=进行中 1=成功 0=失败(开机默认进行中)
}

/* ========================================================================
 * 对外接口 2：ui_init_set_module —— 每个模块开始时调
 *   例：ui_init_set_module("WIFI", 6)  → 开始做 WIFI 模块，内部共 6 小步
 * ======================================================================== */
void ui_init_set_module(const char *module_name, int module_steps)
{
    int slot = g_ui.init.done_modules;   /* 当前该写第几个槽位 */

    /* 把模块名记到 modules[slot].name，状态记成 -1(进行中) */
    if (slot >= 0 && slot < UI_INIT_MAX_MODULES) {
        if (module_name) {
            snprintf(g_ui.init.modules[slot].name,
                     sizeof(g_ui.init.modules[slot].name), "%s", module_name);
        }
        g_ui.init.modules[slot].done = -1;   /* -1 = 进行中(显示 ⏳) */
    }

    /* 当前模块名（顶部 "Running: WIFI" 显示用）*/
    snprintf(g_ui.init.cur_module, sizeof(g_ui.init.cur_module),
             "%s", module_name ? module_name : "");

    /* 重置模块内部进度 */
    g_ui.init.cur.total = (module_steps < 1) ? 1 : module_steps;
    g_ui.init.cur.step  = 0;
    g_ui.init.cur.ok    = -1;
    g_ui.init.cur.text[0] = '\0';
}

/* ========================================================================
 * 对外接口 3：ui_init_set_step —— 模块内部每走一步调一次
 *   例：ui_init_set_step("WIFI SPI OK", 1)  → 模块内第 1 步完成，显示 "WIFI SPI OK"
 * ======================================================================== */
void ui_init_set_step(const char *step_text, int step_index)
{
    if(step_text) {
        snprintf(g_ui.init.cur.text, sizeof(g_ui.init.cur.text), "%s", step_text);
    }
    if (step_index < 1) step_index = 1;
    if (step_index > g_ui.init.cur.total) step_index = g_ui.init.cur.total;
    g_ui.init.cur.step = step_index;
    g_ui.init.cur.ok   = -1;   /* 进行中（粉色） */
}

/* ========================================================================
 * 对外接口 4：ui_init_set_result —— 模块结束时调
 *   success=1 成功(绿勾) 0 失败(红叉)，自动推进大进度条 done_modules++
 * ======================================================================== */
void ui_init_set_result(int success)
{
    int ok = success ? 1 : 0;
    g_ui.init.cur.ok = ok;

    /* 记到模块结果列表的当前槽（和 set_module 时同一个 slot）*/
    int slot = g_ui.init.done_modules;
    if (slot >= 0 && slot < UI_INIT_MAX_MODULES) {
        g_ui.init.modules[slot].done = ok;   /* 1=✓ 0=✗ */
    }

    /* 自动推进大进度：done_modules++（不管成败都算做完了）*/
    if (g_ui.init.done_modules < g_ui.init.total_modules) {
        g_ui.init.done_modules++;
    }
}

/* ========================================================================
 * 对外接口 5：ui_set_power —— 发车关屏开关
 *   on=1 开屏(正常刷新)  on=0 关屏(省算力：停 LCD flush + 主循环跳过 lv_task_handler)
 * ======================================================================== */
void ui_set_power(int on)
{
    g_ui.power_on = (on) ? POWER_ON : POWER_OFF;
    if (g_ui.power_on == POWER_ON) disp_enable_update();
    else                           disp_disable_update();
}

/* ========================================================================
 * 对外接口 7~11：主演示页（PAGE_DEMO）状态设置
 *   业务文件（display.c/image.c 等）只调这些，不碰 LVGL 控件
 * ======================================================================== */
void ui_demo_set_sub_page(int sub_page)
{
    if (sub_page < 0) sub_page = 0;
    if (sub_page > 2) sub_page = 2;
    if (g_ui.demo.sub_page != sub_page) {
        g_ui.demo.sub_page         = sub_page;
        g_ui.demo.sub_page_changed = 1;
    }
}
void ui_demo_set_main_select(int sel)
{
    if (sel < 0) sel = 0;
    if (sel > 1) sel = 1;
    g_ui.demo.main_select = sel;
}
void ui_demo_set_wifi_ok(int ok)
{
    g_ui.demo.wifi_ok = ok ? 1 : 0;
}
void ui_demo_set_fps(int fps)
{
    if (fps < 0)   fps = 0;
    if (fps > 255) fps = 255;
    g_ui.demo.fps_val = fps;
}
void ui_demo_set_img_ready(int ready)
{
    g_ui.demo.img_ready = ready ? 1 : 0;
}

/* ========================================================================
 * 对外接口 6：delay_with_refresh_ms —— LVGL 友好的阻塞 delay
 *   每 10ms 调一次 lv_task_handler 保持画面响应（WIFI 重试 1 秒等待不卡屏）
 *   关屏状态直接死等，不刷
 * ======================================================================== */
void delay_with_refresh_ms(int ms)
{
    if (g_ui.power_on == POWER_ON) {
        int step = 10;
        int left = ms;
        while (left > 0) {
            int sleep_ms = (left > step) ? step : left;
            lv_task_handler();
            system_delay_ms(sleep_ms);
            left -= sleep_ms;
        }
        lv_task_handler();   /* 最后补刷一次，确保最新文字真的显示出来 */
    } else {
        system_delay_ms(ms);  /* 关屏直接死等，不刷 */
    }
}
void ui_switch_page(int new_page)
{
    lv_obj_clean(lv_scr_act());   /* 清掉当前屏幕所有控件（LVGL 会自动释放内存）*/
    g_ui.page = new_page;         /* 改全局状态 */
    lvgl_demo();                  /* 根据 new_page 创建新页面控件 */
}
void lvgl_demo(void)
{
    switch (g_ui.page) {
        case PAGE_INIT:  page_init_create();  break;  /* 创建初始化页控件 */
        case PAGE_DEMO:  page_demo_create();  break;  /* 创建主界面控件（原 label/button/slider） */
        default: break;
    }
}

/* ========================================================================
 * 页面函数：page_init_create —— 创建初始化页的所有 LVGL 控件
 *   标题 / 当前模块名 / 步骤文字 / 小进度条
 * ======================================================================== */
static void page_init_create(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);

    /* 顶部大标题 */
    s_wifi_title = lv_label_create(scr);
    lv_label_set_text(s_wifi_title, "System Init");
    lv_obj_align(s_wifi_title, LV_ALIGN_TOP_MID, 0, 20);

    /* 中间粉色大字（状态文字，如 "WIFI SPI Init..." / "WIFI SPI OK"）*/
    s_wifi_status = lv_label_create(scr);
    lv_obj_set_style_text_font(s_wifi_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_wifi_status, lv_color_hex(0xFF66AA), 0);  /* PINK */
    lv_obj_align(s_wifi_status, LV_ALIGN_CENTER, 0, -20);
    lv_label_set_text(s_wifi_status, "Starting...");

    /* 步骤小字（如 "Step 2 / 4"）—— 用 lv_obj_align 直接定位，不用 align_to */
    s_wifi_step_lbl = lv_label_create(scr);
    lv_obj_set_style_text_color(s_wifi_step_lbl, lv_color_hex(0xCCCCCC), 0);  /* 浅灰色（比 999999 更亮）*/
    lv_obj_align(s_wifi_step_lbl, LV_ALIGN_CENTER, 0, 20);   /* 屏幕中心往下 20 像素 */
    lv_label_set_text(s_wifi_step_lbl, "Step 0 / 4");

    /* 底部小进度条（模块内进度 0~100%）*/
    s_wifi_bar = lv_bar_create(scr);
    lv_obj_set_size(s_wifi_bar, 200, 16);
    lv_obj_align(s_wifi_bar, LV_ALIGN_BOTTOM_MID, 0, -50);
    lv_bar_set_range(s_wifi_bar, 0, 100);
    lv_bar_set_value(s_wifi_bar, 0, LV_ANIM_OFF);
}

/* ========================================================================
 * 页面函数：page_init_refresh —— 读 data.h 标志位 → 刷新初始化页控件
 *   每 100ms 被 ui_refresh_cb 调一次
 * ======================================================================== */
static void page_init_refresh(void)
{
    const char *text;
    int step;
    int total = 6;   /* 初始化共 6 个阶段：1=SPI 2=WIFI 3=TCP 4=WIFI Ready 5=CAM 6=ALL Ready */
    char tmp[32];
    int pct;

    /* 根据 wifi_stage + wifi_result 决定显示什么文字 */
    /* wifi_result: 0=进行中 1=成功 2=失败 */
    switch (wifi_stage) {
        case 0:
            text = "Starting...";
            step = 0;
            break;
        case 1:   /* SPI 初始化 */
            step = 1;
            if      (wifi_result == 0) text = "WIFI SPI Init...";
            else if (wifi_result == 1) text = "WIFI SPI OK";
            else                       text = "WIFI SPI FAIL";
            break;
        case 2:   /* WIFI 连接 */
            step = 2;
            if      (wifi_result == 0) text = "WIFI Connect...";
            else if (wifi_result == 1) text = "WIFI Connect OK";
            else                       text = "WIFI Connect FAIL";
            break;
        case 3:   /* TCP 连接 */
            step = 3;
            if      (wifi_result == 0) text = "TCP Connect...";
            else if (wifi_result == 1) text = "TCP Connect OK";
            else                       text = "TCP Connect FAIL";
            break;
        case 4:   /* WIFI 模块结束 */
            step = 4;
            if      (wifi_result == 0) text = "WIFI Module...";
            else                       text = "WIFI Ready!";
            break;
        case 5:   /* 摄像头 MT9V03X 初始化 */
            step = 5;
            if      (wifi_result == 0) text = "CAM Init...";
            else if (wifi_result == 1) text = "CAM OK";
            else                       text = "CAM FAIL";
            break;
        case 6:   /* 全部完成 */
            text = "ALL READY!";
            step = 6;
            break;
        case 255: /* 失败 */
            text = "INIT FAIL!";
            step = 6;
            break;
        default:
            text = "Unknown";
            step = 0;
            break;
    }

    /* 刷中间粉色大字 */
    lv_label_set_text(s_wifi_status, text);

    /* 刷步骤小字 + 小进度条百分比 */
    snprintf(tmp, sizeof(tmp), "Step %d / %d", step, total);
    lv_label_set_text(s_wifi_step_lbl, tmp);
    pct = (step * 100) / total;
    lv_bar_set_value(s_wifi_bar, pct, LV_ANIM_ON);
}

/* ========================================================================
 * 页面函数：page_demo_create —— 主演示页
 *
 *  屏幕 240×320（竖屏，IPS200）：
 *    ┌────────────────────────────┐ Y=4
 *    │                            │ ← 标题（白色，各居中在自己图像上方）
 *    │ ┌──────────┐   ┌──────────┐│ Y=24
 *    │ │          │   │          ││ ← 2px 蓝色边框（画在容器 lv_obj 上）
 *    │ │   img    │   │   img    ││ ← lv_img 是容器子对象
 *    │ │          │   │          ││
 *    │ └──────────┘   └──────────┘│ Y=100 左右
 *    │      gray         binary   │
 *    │           main             │ ← 子页标题 14 号红字居中
 *    │  -->data        wifi ok    │ ← MAIN 页：菜单 + wifi 状态
 *    │  tuning                    │
 *    │  fps:30                    │ ← DATA 页：帧率（紫色）
 *    └────────────────────────────┘
 *
 * ======================================================================== */
static void page_demo_create(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
    const int img_w = MT9V03X_W;   /* 188 */
    const int img_h = MT9V03X_H;   /* 120 */
    const int gap = 6;             /* 两图之间空隙 */
    const int margin = 4;          /* 左右边距 */

    /* ---- 1) 图像描述符：LV_IMG_CF_ALPHA_8BIT，data 直接从原数组复用，不拷贝 ---- */
    s_demo_gray_dsc.header.always_zero = 0;
    s_demo_gray_dsc.header.w           = img_w;
    s_demo_gray_dsc.header.h           = img_h;
    s_demo_gray_dsc.header.cf          = LV_IMG_CF_ALPHA_8BIT;
    s_demo_gray_dsc.data_size          = (uint32)img_w * (uint32)img_h;
    s_demo_gray_dsc.data               = (const uint8 *)mt9v03x_image[0];

    s_demo_binary_dsc.header.always_zero = 0;
    s_demo_binary_dsc.header.w           = img_w;
    s_demo_binary_dsc.header.h           = img_h;
    s_demo_binary_dsc.header.cf          = LV_IMG_CF_ALPHA_8BIT;
    s_demo_binary_dsc.data_size          = (uint32)img_w * (uint32)img_h;
    s_demo_binary_dsc.data               = (const uint8 *)image_binary[0];

    /* 左右图容器 */
    s_demo_box_gray = lv_obj_create(scr);
    lv_obj_set_size(s_demo_box_gray, img_w, img_h);
    lv_obj_set_pos(s_demo_box_gray, margin, 24);
    lv_obj_set_style_bg_opa(s_demo_box_gray, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_demo_box_gray, 0, 0);
    lv_obj_set_style_pad_all(s_demo_box_gray, 0, 0);


    s_demo_box_binary = lv_obj_create(scr);
    lv_obj_set_size(s_demo_box_binary, img_w, img_h);
    lv_obj_set_pos(s_demo_box_binary, margin + img_w + gap, 24);
    lv_obj_set_style_bg_opa(s_demo_box_binary, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_demo_box_binary, 0, 0);
    lv_obj_set_style_pad_all(s_demo_box_binary, 0, 0);

    /* 图像 */
    s_demo_img_gray = lv_img_create(s_demo_box_gray);
    lv_img_set_src(s_demo_img_gray, &s_demo_gray_dsc);
    lv_obj_align(s_demo_img_gray, LV_ALIGN_CENTER, 0, 0);

    s_demo_img_binary = lv_img_create(s_demo_box_binary);
    lv_img_set_src(s_demo_img_binary, &s_demo_binary_dsc);
    lv_obj_align(s_demo_img_binary, LV_ALIGN_CENTER, 0, 0);

    /* 底部小标题 */
    s_demo_title_gray = lv_label_create(scr);
    lv_label_set_text(s_demo_title_gray, "gray");
    lv_obj_set_style_text_font(s_demo_title_gray, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_demo_title_gray, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_color(s_demo_title_gray, lv_color_hex(0x3A3A3A), 0);
    lv_obj_set_style_bg_opa(s_demo_title_gray, LV_OPA_70, 0);
    lv_obj_set_style_radius(s_demo_title_gray, 6, 0);
    lv_obj_set_style_pad_left(s_demo_title_gray, 8, 0);
    lv_obj_set_style_pad_right(s_demo_title_gray, 8, 0);
    lv_obj_set_style_pad_top(s_demo_title_gray, 2, 0);
    lv_obj_set_style_pad_bottom(s_demo_title_gray, 2, 0);
    lv_obj_align_to(s_demo_title_gray, s_demo_box_gray, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

    s_demo_title_binary = lv_label_create(scr);
    lv_label_set_text(s_demo_title_binary, "binary");
    lv_obj_set_style_text_font(s_demo_title_binary, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_demo_title_binary, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_color(s_demo_title_binary, lv_color_hex(0x3A3A3A), 0);
    lv_obj_set_style_bg_opa(s_demo_title_binary, LV_OPA_70, 0);
    lv_obj_set_style_radius(s_demo_title_binary, 6, 0);
    lv_obj_set_style_pad_left(s_demo_title_binary, 8, 0);
    lv_obj_set_style_pad_right(s_demo_title_binary, 8, 0);
    lv_obj_set_style_pad_top(s_demo_title_binary, 2, 0);
    lv_obj_set_style_pad_bottom(s_demo_title_binary, 2, 0);
    lv_obj_align_to(s_demo_title_binary, s_demo_box_binary, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
    g_ui.demo.sub_page_changed = 0;
}
/* ========================================================================
 * 页面函数：page_demo_refresh —— 读 g_ui.demo → 刷主演示页控件
 *   每 100ms 被 ui_refresh_cb 调用一次
 * ======================================================================== */
static void page_demo_refresh(void)
{
    /* ---- 3) 图像：只要有新帧就绪就刷新一次 img src 指针（触发脏矩形重绘） ---- */
    if (g_ui.demo.img_ready || mt9v03x_finish_flag) {
        /* s_demo_gray_dsc / binary_dsc 本身 data 永远指向全局数组，
         * 这里 set_src 再赋值让 LVGL 知道"内容变了"需要重绘 */
        lv_img_set_src(s_demo_img_gray,   &s_demo_gray_dsc);
        lv_img_set_src(s_demo_img_binary, &s_demo_binary_dsc);
        /* 消费掉 ready 标志 */
        g_ui.demo.img_ready = 0;
        /* mt9v03x_finish_flag 在这里不归 0 — 留给 image_processing/wifi_image_send 用 */
    }
}
 