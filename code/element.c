#include "element.h"
#include "image.h"
#include "beep.h"

/* ================= 远端角点 v3：原图"水平路右带"边缘延伸 =================
 * 旧算法在鸟瞰图(img_pers_data)上沿白色边界逐行外推 + far_corner_intersection
 * 双段直线求交，结果被墙边窄带带到图像边缘（FL/FR 贴边），与真实十字角点对不上。
 * 新算法回到原图二值图(image_binary)：
 *   从右近端角点上方逐行扫描，找"近端右边界之外的第二条宽带"（= 水平路右带），
 *   取该带左缘向下延伸为 FL、右缘向下延伸为 FR。
 * 结果写入 far_rpts0s[0]/far_rpts1s[0]（鸟瞰米制，display/wifi 语义不变），
 * 原图像素坐标写入 far_orig0[0]/far_orig1[0]。
 */
#define FAR_WHITE_THRESHOLD (128)   /* 与 image.c EDGE_WHITE_THRESHOLD 一致 */
#define FAR_BAND_MIN_WIDTH  (8)     /* 与 image.c BINARY_START_MIN_WIDTH 一致 */
#define FAR_BAND_ROWS_MAX   (3)     /* 水平路右带最多累积行数（含首行，与调试页一致） */

/* 十字补线保持期：四角点齐后，即使近端/远端角点暂时失效（如车驶过近端角点、
 * 远端角点被车头遮挡），仍用锁存角点继续补线，直到十字驶出图像。单位：帧
 * （60fps 下 60 帧 ≈ 1s；可调，太长会在驶出后把线补向车后方）。 */
#define CROSS_HOLD_FRAMES   (60)

/* 十字蜂鸣：四个角点全有效时边沿触发，连续响 cross_buzz_times 次。 */
#define CROSS_BUZZ_ON_FRAMES  (8)   /* 每次鸣响时长（帧），与大弯道一致（约 70~130ms） */
#define CROSS_BUZZ_GAP_FRAMES (6)   /* 两声之间停顿（帧） */

/* 原图坐标 → 鸟瞰坐标：在逆透视表(invx/invy，鸟瞰→原图)上反向查找；
 * 找不到时在半径 2 邻域内找最近有效点；返回 -1 表示不可映射。 */
static int16 far_orig_to_bird_id(int16 fx, int16 fy)
{
    int16 v, u, r, dy, dx;
    for (v = 0; v < PERS_H; v++)
    {
        for (u = 0; u < PERS_W; u++)
        {
            if (invx[v][u] == fx && invy[v][u] == fy)
                return (int16)(v * PERS_W + u);
        }
    }
    for (r = 1; r <= 2; r++)
    {
        for (dy = -r; dy <= r; dy++)
        {
            for (dx = -r; dx <= r; dx++)
            {
                int16 nx, ny;
                if (abs(dx) + abs(dy) != r) continue;
                nx = fx + dx; ny = fy + dy;
                if (nx < 0 || nx >= PERS_W || ny < 0 || ny >= PERS_H) continue;
                for (v = 0; v < PERS_H; v++)
                {
                    for (u = 0; u < PERS_W; u++)
                    {
                        if (invx[v][u] == nx && invy[v][u] == ny)
                            return (int16)(v * PERS_W + u);
                    }
                }
            }
        }
    }
    return -1;
}


/* 角点变量定义统一在 image.c（含 far_ 远线系列），本文件只写函数 */

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
        /* 车体横向移动时 NMS 可能使单帧角度点被抑制，保留原始角度峰值判断。 */
        int16 im1 = clip(i - dist_pts, 0, rpts0s_num - 1);
        int16 ip1 = clip(i + dist_pts, 0, rpts0s_num - 1);
        // 角点置信度：当前角度 - 前后均值（折点在角度上呈尖峰）
        float conf1 = fabsf(rpts0a[i]) - (fabsf(rpts0a[im1]) + fabsf(rpts0a[ip1])) * 0.5f;
        // 90°折点：52°~120° 且 在 1.6m 内
        if (!Lpt0_found && conf1 > 0.60f && conf1 < 2.35f && i < (int16)(2.0f / sample_dist))
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
        int16 im1 = clip(i - dist_pts, 0, rpts1s_num - 1);
        int16 ip1 = clip(i + dist_pts, 0, rpts1s_num - 1);
        float conf2 = fabsf(rpts1a[i]) - (fabsf(rpts1a[im1]) + fabsf(rpts1a[ip1])) * 0.5f;
        if (!Lpt1_found && conf2 > 0.60f && conf2 < 2.35f && i < (int16)(2.0f / sample_dist))
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
/* 远端角点 v3：原图"水平路右带"边缘延伸（替代鸟瞰外推 + 双段求交）。 */
void find_far_corners(void)
{
    static float prev_lx = 0.0f, prev_ly = 0.0f, prev_rx = 0.0f, prev_ry = 0.0f;
    static uint8 stable_l = 0, stable_r = 0;
    int16 nr_x, nr_y;
    int16 y, band_y0 = -1;
    int16 xL = -1, xR = -1, pxL = -1, pxR = -1;
    uint8 cnt = 0;
    int16 fl_x, fl_y, fr_x, fr_y;
    int16 id;

    far_Lpt0_found = 0;
    far_Lpt1_found = 0;
    far_Lpt0_rpts0s_id = -1;
    far_Lpt1_rpts1s_id = -1;
    far_rpts0s_num = 0;
    far_rpts1s_num = 0;

    /* 前提：右近端角点有效（水平路右带从右近端角点上方开始找）。 */
    if (!Lpt1_found || Lpt1_rpts1s_id < 0 || Lpt1_rpts1s_id >= rpts1s_num)
    {
        stable_l = 0;
        stable_r = 0;
        return;
    }
    /* rpts1s 是原图坐标米制（L0 由 left/right_line_points 转换而来） */
    nr_x = (int16)(rpts1s[Lpt1_rpts1s_id][0] * pixel_per_meter);
    nr_y = (int16)(rpts1s[Lpt1_rpts1s_id][1] * pixel_per_meter);
    if (nr_y < 8 || nr_y >= PERS_H - 2)
    {
        stable_l = 0;
        stable_r = 0;
        return;
    }

    /* 从右近端角点上方逐行扫原图二值：枚举白色段，取第 2 条宽度足够的段；
     * 若该段右端不含近端右边界，即为"水平路右带"；累积最多 4 行取均值平滑。 */
    for (y = nr_y - 2; y >= 2; y--)
    {
        int16 x = 1;
        int16 wide_cnt = 0;
        int16 rb_left = -1, rb_right = -1;
        while (x < PERS_W - 1)
        {
            int16 rl, rr;
            while (x < PERS_W - 1 && image_binary[y][x] < FAR_WHITE_THRESHOLD) x++;
            if (x >= PERS_W - 1) break;
            rl = x;
            while (x < PERS_W - 1 && image_binary[y][x] >= FAR_WHITE_THRESHOLD) x++;
            rr = x - 1;
            if (rr - rl + 1 >= FAR_BAND_MIN_WIDTH)
            {
                wide_cnt++;
                if (wide_cnt == 2) { rb_left = rl; rb_right = rr; break; }
            }
        }
        if (wide_cnt >= 2 && rb_left >= 0 && rb_right < nr_x - 10)
        {
            if (band_y0 < 0)
            {
                band_y0 = y;
                xL = rb_left;
                xR = rb_right;
                cnt = 1;
                continue;
            }
            if (cnt < FAR_BAND_ROWS_MAX)
            {
                pxL = xL; pxR = xR;
                xL = rb_left;
                xR = rb_right;
                cnt++;
                continue;
            }
            break;
        }
        if (band_y0 >= 0) break;   /* 已进入水平带后出现其他结构，终止 */
    }
    if (band_y0 < 0)
    {
        stable_l = 0;
        stable_r = 0;
        return;
    }

    fl_x = (xL + (pxL >= 0 ? pxL : xL)) / 2;   /* FL：右带左缘向下延伸 */
    fr_x = (xR + (pxR >= 0 ? pxR : xR)) / 2;   /* FR：右带右缘向下延伸 */
    fl_y = band_y0 + 2;
    fr_y = band_y0 + 4;

    /* ---- FL：原图点 → 鸟瞰点，写入 far_rpts0s[0]（鸟瞰米制） ---- */
    id = far_orig_to_bird_id(fl_x, fl_y);
    if (id >= 0)
    {
        float ddx, ddy;
        far_rpts0s[0][0] = (float)(id % PERS_W) / pixel_per_meter;
        far_rpts0s[0][1] = (float)(id / PERS_W) / pixel_per_meter;
        far_orig0[0][0] = (float)fl_x;
        far_orig0[0][1] = (float)fl_y;
        far_rpts0s_num = 1;
        ddx = far_rpts0s[0][0] - prev_lx;
        ddy = far_rpts0s[0][1] - prev_ly;
        /* 远端候选必须连续稳定，且单帧跳变不能超过约 6 cm。 */
        if (stable_l == 0 || sqrtf(ddx * ddx + ddy * ddy) < 0.06f) stable_l++;
        else stable_l = 1;
        prev_lx = far_rpts0s[0][0];
        prev_ly = far_rpts0s[0][1];
        if (stable_l >= 4) { far_Lpt0_rpts0s_id = 0; far_Lpt0_found = 1; }
    }
    else stable_l = 0;

    /* ---- FR：对称处理 ---- */
    id = far_orig_to_bird_id(fr_x, fr_y);
    if (id >= 0)
    {
        float ddx, ddy;
        far_rpts1s[0][0] = (float)(id % PERS_W) / pixel_per_meter;
        far_rpts1s[0][1] = (float)(id / PERS_W) / pixel_per_meter;
        far_orig1[0][0] = (float)fr_x;
        far_orig1[0][1] = (float)fr_y;
        far_rpts1s_num = 1;
        ddx = far_rpts1s[0][0] - prev_rx;
        ddy = far_rpts1s[0][1] - prev_ry;
        if (stable_r == 0 || sqrtf(ddx * ddx + ddy * ddy) < 0.06f) stable_r++;
        else stable_r = 1;
        prev_rx = far_rpts1s[0][0];
        prev_ry = far_rpts1s[0][1];
        if (stable_r >= 4) { far_Lpt1_rpts1s_id = 0; far_Lpt1_found = 1; }
    }
    else stable_r = 0;
}

/* ================= 十字补线：用四个角点把断开的边界线补成连续虚拟线 =================
 * 触发条件：近端两角点(NL/NR) + 远端两角点(FL/FR) 同时有效（远端自带 4 帧稳定滤波）。
 * 做法：左右边线在近端角点处截断，从近端角点向远端角点逐行追踪原图二值边界
 *       （贴合十字实际边缘：垂直路左/右缘 → 水平带边缘的台阶/拐角），不是直线。
 *       追踪点同步写入 left_line_points/right_line_points（像素）与
 *       rpts0s/rpts1s（米制），使显示/上传/巡线在十字区连续。
 * 保持：四角点齐后进入保持期（CROSS_HOLD_FRAMES），车驶过近端角点后继续用锁存
 *       角点补线，直到十字驶出图像；cross_line_active 供蜂鸣器同源触发。
 */
uint8 cross_line_active = 0;    /* 1=十字补线正在生效（蜂鸣/调试用） */

#define CROSS_TRACE_WINDOW (24)  /* 每行边界追踪搜索窗口半径（px） */
#define CROSS_TRACE_MIN_W   (4)  /* 参与候选的白色段最小宽度（px，滤噪声） */

/* 逐行边界追踪：从起点 (nx,ny) 向远端 (fy) 每行找与上一行最连续的白色段边界
 * （left_side=1 取段左缘，0 取段右缘）。只过滤"目标边缘贴边"的段：
 * 左线要求左缘 > 1（右缘可贴边，因为十字区主带右缘贴边 187 是常态）；
 * 右线要求右缘 < PERS_W-2。找不到段时沿用上一行位置（线不裂开）。
 * 返回点数（含起点），点序 y 递减；容量上限由调用方保证。 */
static int16 cross_trace_line(const uint8 binary[PERS_H][PERS_W], int16 pts[][2], int16 cap,
                              int16 nx, int16 ny, int16 fy, uint8 left_side)
{
    int16 n = 0;
    int16 y, x, lo, hi;
    int16 bpx = nx;
    if (fy >= ny) return 0;                       /* 远端必须在近端上方 */
    if (ny - fy > PERS_H - 8) return 0;           /* 补线段过长视为异常 */
    pts[n][0] = nx; pts[n][1] = ny; n++;          /* 起点 = 近端角点行 */
    for (y = ny - 1; y >= fy; y--)
    {
        int16 best_x = -1, best_d = 32767;
        lo = bpx - CROSS_TRACE_WINDOW; if (lo < 1) lo = 1;
        hi = bpx + CROSS_TRACE_WINDOW; if (hi > PERS_W - 2) hi = PERS_W - 2;
        x = lo;
        while (x <= hi)
        {
            int16 rl, rr, cand, d;
            while (x <= hi && binary[y][x] < FAR_WHITE_THRESHOLD) x++;
            if (x > hi) break;
            rl = x;
            while (x <= hi && binary[y][x] >= FAR_WHITE_THRESHOLD) x++;
            rr = x - 1;
            if (rr - rl + 1 < CROSS_TRACE_MIN_W) continue;
            cand = left_side ? rl : rr;
            /* 只过滤目标边缘贴边；十字区主带另一侧贴边是常态，允许 */
            if (left_side && cand <= 1) continue;
            if (!left_side && cand >= PERS_W - 2) continue;
            d = abs(cand - bpx);
            if (d < best_d) { best_d = d; best_x = cand; }
        }
        if (best_x < 0) best_x = bpx;             /* 该行无合适边界：沿用上一行，保持线连续 */
        pts[n][0] = best_x;
        pts[n][1] = y;
        n++;
        if (n >= cap) break;
        bpx = best_x;
    }
    return n;
}

void cross_line_completion(void)
{
    /* 锁存角点：四角点齐时刷新，车驶过近端角点后继续用旧值补线（保持期） */
    static int16 h_nl_x = 0, h_nl_y = 0, h_nr_x = 0, h_nr_y = 0;
    static int16 h_fl_x = 0, h_fl_y = 0, h_fr_x = 0, h_fr_y = 0;
    static int16 trace_l[IPTS_MAX][2], trace_r[IPTS_MAX][2];   /* 追踪点缓冲 */
    static uint8 line_hold = 0;      /* 补线保持标志 */
    static int16 hold_cnt = 0;       /* 保持期剩余帧 */
    uint8 full;
    int16 nx, ny, rx, ry, fl_x, fl_y, fr_x, fr_y;
    int16 lk = -1, rk = -1, lk_m = -1, rk_m = -1, i;
    int16 n_l, n_r;

    /* 四角点全有效：刷新锁存角点并重置保持期（远端自带 4 帧稳定滤波） */
    full = (Lpt0_found && Lpt1_found && far_Lpt0_found && far_Lpt1_found) ? 1 : 0;
    if (full)
    {
        if (Lpt0_rpts0s_id < 0 || Lpt0_rpts0s_id >= rpts0s_num) return;
        if (Lpt1_rpts1s_id < 0 || Lpt1_rpts1s_id >= rpts1s_num) return;
        h_nl_x = (int16)(rpts0s[Lpt0_rpts0s_id][0] * pixel_per_meter);
        h_nl_y = (int16)(rpts0s[Lpt0_rpts0s_id][1] * pixel_per_meter);
        h_nr_x = (int16)(rpts1s[Lpt1_rpts1s_id][0] * pixel_per_meter);
        h_nr_y = (int16)(rpts1s[Lpt1_rpts1s_id][1] * pixel_per_meter);
        h_fl_x = (int16)far_orig0[0][0];
        h_fl_y = (int16)far_orig0[0][1];
        h_fr_x = (int16)far_orig1[0][0];
        h_fr_y = (int16)far_orig1[0][1];
        line_hold = 1;
        hold_cnt = CROSS_HOLD_FRAMES;
    }
    else if (line_hold)
    {
        if (hold_cnt > 0) hold_cnt--;
        if (hold_cnt == 0) line_hold = 0;   /* 保持期结束（十字应已驶过） */
    }
    if (!line_hold)
    {
        cross_line_active = 0;
        return;
    }
    cross_line_active = 1;

    /* 用锁存的角点坐标（车驶过近端角点后仍保持补线） */
    nx = h_nl_x; ny = h_nl_y;
    rx = h_nr_x; ry = h_nr_y;
    fl_x = h_fl_x; fl_y = h_fl_y;
    fr_x = h_fr_x; fr_y = h_fr_y;

    /* 合理性：远端在近端上方（y 更小）且距离在图像范围内 */
    if (fl_y >= ny || fr_y >= ry)
    {
        cross_line_active = 0;
        return;
    }
    if (ny - fl_y > PERS_H - 8 || ry - fr_y > PERS_H - 8)
    {
        cross_line_active = 0;
        return;
    }

    /* 截断点定位：巡线从底部(y大)向远端(y小)存点，取 y>=锁存行的最接近行。
     * 车前进后锁存行可能已被新的入口带覆盖，此时自动退到下方最近行，保证连续。 */
    for (i = 0; i < left_line_count; i++)
        if (left_line_points[i][1] >= ny) lk = i;
    for (i = 0; i < right_line_count; i++)
        if (right_line_points[i][1] >= ry) rk = i;
    if (lk < 0 || rk < 0)
    {
        cross_line_active = 0;
        return;
    }
    /* 起点取巡线实际点（与巡线末端无缝衔接） */
    nx = left_line_points[lk][0];
    ny = left_line_points[lk][1];
    rx = right_line_points[rk][0];
    ry = right_line_points[rk][1];

    /* 逐行追踪左右边界（弯，贴合十字实际边缘：主带左缘渐进左扩/右缘台阶） */
    n_l = cross_trace_line(image_binary, trace_l, IPTS_MAX, nx, ny, fl_y, 1);
    n_r = cross_trace_line(image_binary, trace_r, IPTS_MAX, rx, ry, fr_y, 0);
    if (n_l < 2 || n_r < 2)
    {
        cross_line_active = 0;
        return;
    }

    /* 像素空间：截断在 lk，起点即巡线点，之后补追踪点（显示/上传直接可见） */
    if (lk + n_l <= IPTS_MAX)
    {
        for (i = 1; i < n_l; i++)
        {
            left_line_points[lk + i][0] = trace_l[i][0];
            left_line_points[lk + i][1] = trace_l[i][1];
        }
        left_line_count = (uint16)(lk + n_l);
    }
    if (rk + n_r <= IPTS_MAX)
    {
        for (i = 1; i < n_r; i++)
        {
            right_line_points[rk + i][0] = trace_r[i][0];
            right_line_points[rk + i][1] = trace_r[i][1];
        }
        right_line_count = (uint16)(rk + n_r);
    }

    /* 米制空间同步补线（calculation_error 巡线用）：起点行与像素侧一致 */
    for (i = 0; i < rpts0s_num; i++)
        if (rpts0s[i][1] * pixel_per_meter >= ny) lk_m = i;
    for (i = 0; i < rpts1s_num; i++)
        if (rpts1s[i][1] * pixel_per_meter >= ry) rk_m = i;
    if (lk_m < 0 || rk_m < 0) return;
    if (lk_m + n_l <= POINTS_MAX_LEN)
    {
        for (i = 1; i < n_l; i++)
        {
            rpts0s[lk_m + i][0] = (float)trace_l[i][0] / pixel_per_meter;
            rpts0s[lk_m + i][1] = (float)trace_l[i][1] / pixel_per_meter;
        }
        rpts0s_num = (int16)(lk_m + n_l);
    }
    if (rk_m + n_r <= POINTS_MAX_LEN)
    {
        for (i = 1; i < n_r; i++)
        {
            rpts1s[rk_m + i][0] = (float)trace_r[i][0] / pixel_per_meter;
            rpts1s[rk_m + i][1] = (float)trace_r[i][1] / pixel_per_meter;
        }
        rpts1s_num = (int16)(rk_m + n_r);
    }
}

/* ================= 蜂鸣器统一输出：大弯道 + 十字提示 =================
 * 每帧在 process_edge_points 末尾调用一次。
 * - 大弯道：buzzer_tick>0 时响（calculation_error 只做边沿触发设置，此处统一递减输出）
 * - 十字：补线激活（四角点确认，cross_line_active 上升沿）触发，
 *         与补线完全同源——只要屏幕上补线出现，蜂鸣必然响 cross_buzz_times 次
 */
void beeper_poll(void)
{
    static uint8 cb_prev = 0;       /* 上一帧补线激活标志 */
    static int16 cb_remain = 0;     /* 剩余鸣响次数 */
    static int16 cb_on = 0;         /* 当前这声响剩余帧数 */
    static int16 cb_off = 0;        /* 当前停顿剩余帧数 */
    uint8 cb_now;
    uint8 beep = 0;

    /* 十字：补线激活（四角点确认）边沿触发一次哔哔哔 */
    cb_now = cross_line_active ? 1 : 0;
    if (cb_now && !cb_prev)
    {
        cb_remain = (cross_buzz_times > 1) ? (int16)(cross_buzz_times - 1) : 0;
        cb_on = CROSS_BUZZ_ON_FRAMES;
        cb_off = 0;
    }
    cb_prev = cb_now;

    if (cb_on > 0)
    {
        cb_on--;
        beep = 1;
        if (cb_on == 0 && cb_remain > 0) cb_off = CROSS_BUZZ_GAP_FRAMES;
    }
    else if (cb_off > 0)
    {
        cb_off--;
        if (cb_off == 0 && cb_remain > 0)
        {
            cb_remain--;
            cb_on = CROSS_BUZZ_ON_FRAMES;
        }
    }

    /* 统一输出：大弯道或十字任一在响都保持 beep_on */
    if (buzzer_tick > 0)
    {
        buzzer_tick--;
        beep = 1;
    }
    if (beep) beep_on(); else beep_off();
}
