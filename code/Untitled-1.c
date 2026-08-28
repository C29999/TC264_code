/* ---------- LVGL版 WIFI 初始化 UI + 发车关屏封装 ---------- */
void wifi_ui_init(void);                                 /* 创建"WIFI Init 页"的 LVGL 控件（标题/状态/进度条/重试） */
void wifi_ui_show(const char *text, int step, int total);/* 更新状态文字：显示 text + 进度 step/total (进度条自动走百分比) */
void wifi_ui_power(int on);                              /* 1=开屏正常显示  0=发车关屏(停止LCD刷新+跳主循环lv_task) */
int  wifi_ui_active(void);                               /* 主循环用：返回0就别调 lv_task_handler/display_draw，省算力 */