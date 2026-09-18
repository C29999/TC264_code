#ifndef ELEMENT_H
#define ELEMENT_H

#include "zf_common_headfile.h"

/* ================ 元素主类型 ================ */
typedef enum {
    ELEM_NONE = 0,        // 无元素 / 直道
    ELEM_STRAIGHT,
    ELEM_CROSS,           // 十字
    ELEM_ISLAND_LEFT,     // 左环岛（暂不实现）
    ELEM_ISLAND_RIGHT,    // 右环岛（暂不实现）
    ELEM_UNKNOWN
} element_type_t;

typedef enum {
    CROSS_NONE = 0,       // 非十字模式
    CROSS_HALF_LEFT,      // 左半十字
    CROSS_HALF_RIGHT,     // 右半十字
    CROSS_OUT             // 退出过渡
} cross_sub_t;

extern element_type_t current_elem_type;   // 当前元素主类型
extern cross_sub_t    current_cross_sub;   // 十字内部子状态

extern int16 cross_pre_in_length;          // 角点最近端距离阈值（点数）
extern int16 cross_pre_in_another_length;  // 另一侧角点距离阈值
extern int16 cross_in_length;             // 进入十字后判断"无近边线"的阈值
extern int32 cross_run_distance;          // 跑十字内部的目标距离（编码器脉冲）
extern int16 cross_out_length;             // 退出时近边线恢复长度阈值
extern int32 cross_out_distance;           // 退出过渡距离

/* ================ 角点结果（定义在 element.c） ================ */
extern int16 Lpt0_rpts0s_id, Lpt1_rpts1s_id;
extern int16 N_Lpt0_rpts0s_id, N_Lpt1_rpts1s_id;
extern int16 Lpt0_found, Lpt1_found;
extern int16 N_Lpt0_found, N_Lpt1_found;

/* ================ 远端角点 ================ */
extern int16 far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;
extern int16 far_Lpt0_found, far_Lpt1_found;
extern float far_rpts0s[][2];   // 长度不写在声明里，避免依赖 image.h（image.h 与 zf_common 循环 include）
extern float far_rpts1s[][2];
extern int16  far_rpts0s_num, far_rpts1s_num;

/* ================ 直线度 + 置信度 ================ */
extern int16 is_straight0, is_straight1;
extern float conf1, conf2, conf1_max, conf2_max;

extern int32 total_distance;               // 累计行驶距离（编码器脉冲）
extern int16 encoder_measure_flag;         // 1=正在测距 0=停止测距
extern uint16 g_elem_debug;                // 1=显示调试信息

/* ================ 函数声明 ================ */
void check_cross(void);                    // 十字判别（基于角点+远角点+直线度）
void run_cross(void);                      // 十字状态机执行
void element_state_machine(void);           // 元素总入口（cpu1主循环调用）

#endif