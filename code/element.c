#include "element.h"


int16 left_cont;

/* ================ 角点结果（元素模块） ================ */
int16 Lpt0_rpts0s_id = 0, Lpt1_rpts1s_id = 0;     // 左/右 L 角点索引
int16 N_Lpt0_rpts0s_id = 0, N_Lpt1_rpts1s_id = 0; // 左/右 反L角点索引（负峰）
int16 Lpt0_found = 0, Lpt1_found = 0;             // 1=找到 L 角点
int16 N_Lpt0_found = 0, N_Lpt1_found = 0;         // 1=找到 反L 角点

/* 远端 L 角点（十字判据用：两个入口角点之间的垂直距离校验） */
int16 far_Lpt0_rpts0s_id = 0, far_Lpt1_rpts1s_id = 0; // 远端角点索引
int16 far_Lpt0_found = 0, far_Lpt1_found = 0;     // 1=找到远端角点

/* 远端边线段（十字内部导航时切到远端边线算偏差） */
float far_rpts0s[POINTS_MAX_LEN][2]; float far_rpts1s[POINTS_MAX_LEN][2];
int16 far_rpts0s_num = 0, far_rpts1s_num = 0;     // 远端边线实际点数

/* 直线度 + 置信度（用于区分直道/弯道/十字） */
int16 is_straight0 = 0, is_straight1 = 0;         // 1=该边线近似直线
float conf1 = 0, conf2 = 0;                       // 当前帧角度峰值
float conf1_max = 0, conf2_max = 0;               // 最近端最大角度峰值

void find_corner_point(void)
{
    
    
}

// 找十字"下面两个交点"：左右边线上 52°~120° 的 90°折点（国一 find_corners 移植）
void find_corners(void)
{
    // 直道判定初始化：边线够长(>1.2m)才可能是直线
    is_straight0 = (rpts0s_num > (int16)(1.2f / sample_dist));
    is_straight1 = (rpts1s_num > (int16)(1.2f / sample_dist));
    Lpt0_found = Lpt1_found = N_Lpt0_found = N_Lpt1_found = 0;
    Lpt0_rpts0s_id = Lpt1_rpts1s_id = -1;
    N_Lpt0_rpts0s_id = N_Lpt1_rpts1s_id = -1;
    conf1_max = conf2_max = 0;

    int16 dist_pts = (int16)roundf(angle_dist / sample_dist);   // 0.05/0.02=2.5→3
    if (dist_pts < 1) dist_pts = 1;

    // ---- 左边线：找 Lpt0 ----
    for (int16 i = dist_pts; i < (rpts0s_num < 150 ? rpts0s_num : 150); i++)
    {
        if (rpts0an[i] == 0) continue;              // NMS 过滤
        int16 im1 = clip(i - dist_pts, 0, rpts0s_num - 1);
        int16 ip1 = clip(i + dist_pts, 0, rpts0s_num - 1);
        // 角点置信度：当前角度 - 前后均值（折点在角度上呈尖峰）
        float conf1 = fabsf(rpts0a[i]) - (fabsf(rpts0a[im1]) + fabsf(rpts0a[ip1])) * 0.5f;
        // 90°折点：52°~120° 且 在 1.6m 内
        if (!Lpt0_found && conf1 > 0.9076f && conf1 < 2.0944f && i < (int16)(1.6f / sample_dist))
        {
            Lpt0_rpts0s_id = i;
            Lpt0_found = 1;
            if (rpts0a[i] > 0)                      // 正=向右拐→内L点（十字外侧拐角，排除）
            {
                N_Lpt0_rpts0s_id = i;
                N_Lpt0_found = 1;
                Lpt0_found = 0;
            }
        }
        // 非直线判定：0.36~1.92m 内出现 >8° 折角
        if (conf1 > 0.1396f && i > (int16)(0.36f / sample_dist) && i < (int16)(1.92f / sample_dist))
            is_straight0 = 0;
        if (conf1 > conf1_max) conf1_max = conf1;   // 调试用
    }

    // ---- 右边线：找 Lpt1（对称）----
    for (int16 i = dist_pts; i < (rpts1s_num < 150 ? rpts1s_num : 150); i++)
    {
        if (rpts1an[i] == 0) continue;
        int16 im1 = clip(i - dist_pts, 0, rpts1s_num - 1);
        int16 ip1 = clip(i + dist_pts, 0, rpts1s_num - 1);
        float conf2 = fabsf(rpts1a[i]) - (fabsf(rpts1a[im1]) + fabsf(rpts1a[ip1])) * 0.5f;
        if (!Lpt1_found && conf2 > 0.9076f && conf2 < 2.0944f && i < (int16)(1.6f / sample_dist))
        {
            Lpt1_rpts1s_id = i;
            Lpt1_found = 1;
            if (rpts1a[i] < 0)                      // 负=向左拐→内L点
            {
                N_Lpt1_rpts1s_id = i;
                N_Lpt1_found = 1;
                Lpt1_found = 0;
            }
        }
        if (conf2 > 0.1396f && i > (int16)(0.36f / sample_dist) && i < (int16)(1.92f / sample_dist))
            is_straight1 = 0;
        if (conf2 > conf2_max) conf2_max = conf2;
    }
}
