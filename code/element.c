#include "element.h"
#include "image.h"
#include "beep.h"

/* data.c 定义（data.h 同声明）：累计行驶距离（米），十字状态机时序用 */

/* ============================================================================
 * 十字路口检测（移植自逐飞 STC32 开源例程当前启用写法）
 *
 * 思路（与旧"找四个角点+几何补线"完全不同）：
 *   1) 近端角点 = 边线上"前后方向向量真实夹角 ≥75°"的第一个折点
 *      （等距采样点上取前后各 14 点的入/出向量，atan2 求夹角；
 *        比"x 跳变 N 像素"抗噪，弯道渐变不会误触发）。
 *   2) 识别 = 左右巡线同时触界/丢失，并且 NL/NR/FL/FR 四角点齐全。
 *   3) 状态机 none → start → enter → out，全部用编码器里程推进/超时撤销。
 *   4) enter（车在十字中）不找远端角点：
 *      列扫描找"最长白列"（看得最远的列）= 对面竖路中心，
 *      再行扫描确认对面路口宽度稳定，得路口中心 cross_max_x；
 *      导航目标直接钉在路口中心直穿（calculation_error 消费）。
 *
 * 对外接口保持不变（display.c / wifi_spi.c 依赖）：
 *   Lpt0_found/Lpt1_found + Lpt0_rpts0s_id/Lpt1_rpts1s_id  近端折角角点
 *   far_Lpt0_found/far_Lpt1_found + far_rpts0s[0]/far_orig0[0]  对面路口左右边缘
 *   cross_hold_*（$CORNERS 文本协议 + 上位机角点 boundary）
 *   cross_line_active（蜂鸣同源：none→start 上升沿响 cross_buzz_times 次）
 * ==========================================================================*/

/* ---- 折角检测参数 ---- */
#define CR_STEP          14       /* 入/出向量窗口（等距点数，sample_dist=0.02m → 0.28m） */
#define CR_ANGLE_TH      1.309f   /* 折角阈值 75°（弧度） */
#define CR_ID_DUAL       80       /* 双角点：角点序号上限 */
#define CR_SIDE_MIN_X    20       /* 左角点 x 下限（仅排除最外侧噪声） */
#define CR_SIDE_MAX_X    172      /* 右角点 x 上限（给逆透视后的外扩留余量） */
#define CR_FOLD_SIDE_PX  8        /* 折后必须向外侧展开的最小位移（像素） */
#define CR_CONFIRM_FRAMES 3       /* 连续三帧满足几何判据后才确认十字 */
#define CR_NEAR_SPAN_MIN 45       /* 两近端角点横向间距下限，滤单侧弯道误检 */
#define CR_NEAR_Y_GAP_MAX 24      /* 两近端角点纵向差上限，十字入口角点应近似同高 */
#define CR_PATCH_K_MIN   0.35f    /* 十字补线斜率下限（像素/像素，待实车标定） */
#define CR_PATCH_K_MAX   8.00f    /* 十字补线斜率上限（像素/像素，待实车标定） */

/* ---- start → enter 参数 ---- */
#define CR_ENTRY_W_PX    48       /* 两角点/入口横向张开 > 48px 视为进入横路 */
#define CR_ENTRY_Y_PX    90       /* 两角点 y 都 > 90（进入近端）视为进入 */

/* ---- enter 列/行扫描参数（二值图版，替代原局部大津） ---- */
#define CR_SCAN_TOP_Y    CROSS_SCAN_TOP_Y /* 列扫描起始行（车头附近，原 car_head_y-10） */
#define CR_SCAN_X_HALF   20       /* 列扫描 x 范围 ±20 */
#define CR_ROW_START_Y   98       /* 行扫描起始行 */
#define CR_ROW_EDGE_XL   30       /* 路口左边缘必须 > 30（否则还在横路段） */
#define CR_ROW_EDGE_XR   158      /* 路口右边缘必须 < 158 */
#define CR_ROW_W_MAX     120      /* 行宽 ≥120 视为仍在横路，继续上扫 */
#define CR_ROW_W_MIN     10       /* 行宽下限（滤噪） */
#define CR_ROW_STABLE    5        /* 连续三行边界差 ≤5px 视为路口稳定 */

/* ---- enter → out / out → none：只使用原图边线，不依赖逆透视 ---- */
#define CR_OUT_START_Y      93
#define CR_OUT_START_W     125
#define CR_EXIT_W_LO        40
#define CR_EXIT_W_HI        48
#define CR_EXIT_MIN_PTS     25
#define CR_EXIT_STABLE       5       /* 出口竖道连续确认，防止双十字中段短暂变窄 */
#define CR_M_START_TO      0.9f
#define CR_M_ENTER_MIN    0.33f   /* 进入十字后至少保持的车体距离 */
#define CR_M_ENTER_TO      1.60f  /* 未识别出口时的故障保护上限，不作为正常退出条件 */
#define CR_M_OUT_TO       0.17f

/* ---- 蜂鸣（与旧版一致） ---- */
#define CROSS_BUZZ_ON_FRAMES  8
#define CROSS_BUZZ_GAP_FRAMES 6

/* ===================== 十字状态机状态 ===================== */
/* CR_NONE/CR_START/CR_ENTER/CR_OUT 定义见 image.h */
uint8  cross_flag = CR_NONE;     /* 当前十字状态（calculation_error 消费） */
uint8  cross_candidate_frames = 0;
uint8  cross_near_pair_valid = 0;
uint8  cross_geometry_valid = 0;
uint8  cross_exit_lane_valid = 0;
uint8  cross_exit_confirm_frames = 0;
float cross_phase_distance = 0.0f;
static float cross_enc_base = 0.0f;
static float cross_out_enc_base = 0.0f;
int16  cross_max_x = MT9V03X_W / 2; /* 对面路口中心 x（像素，导航目标） */
int16  cross_far_y = -1;            /* 对面路口所在行（像素） */
int16  cross_edge_l = -1;           /* 对面路口左边缘（像素，显示用） */
int16  cross_edge_r = -1;           /* 对面路口右边缘（像素，显示用） */
static int16 s_aim_x = MT9V03X_W / 2;   /* 列扫描目标保持（对应 cross_max_x） */
static int16 s_mid_x = MT9V03X_W / 2;   /* 行扫描成功的路口中心保持 */

/* cross_line_active / cross_hold 定义（WiFi $CORNERS 与蜂鸣依赖） */
uint8 cross_line_active = 0;
int16 cross_hold_nl_x = -1, cross_hold_nl_y = -1;
int16 cross_hold_nr_x = -1, cross_hold_nr_y = -1;
int16 cross_hold_fl_x = -1, cross_hold_fl_y = -1;
int16 cross_hold_fr_x = -1, cross_hold_fr_y = -1;

/* ===================== 四角点虚拟补线与导航线 =====================
 * 四个角点 NL/NR（近端折角）与 FL/FR（远端爬线）同帧全部有效时：
 *   虚拟左边线 = NL-FL 直线，虚拟右边线 = NR-FR 直线（补出横路段断边）；
 *   虚拟中线   = M0((NL+NR)/2) → M1((FL+FR)/2)（补边线的中点连线）。
 * 不覆盖普通边界数组；输出给 display.c / wifi_spi.c，并在 ENTER/OUT 由
 * calculation_error() 作为十字导航线使用。 */
uint8 cross_vline_valid = 0;
int16 cv_nl_x = -1, cv_nl_y = -1, cv_nr_x = -1, cv_nr_y = -1;
int16 cv_fl_x = -1, cv_fl_y = -1, cv_fr_x = -1, cv_fr_y = -1;
int16 cv_m0_x = -1, cv_m0_y = -1, cv_m1_x = -1, cv_m1_y = -1;

/* 参考 extendline_l/r：远端角点缺失时，沿近端角点之前的边界切线向上延长。 */
static uint8 cross_extend_far(float pts[][2], int16 num, int16 corner_id,
                              int16 *far_x, int16 *far_y)
{
    int16 i0 = corner_id - 7;
    int16 i1 = corner_id - 2;
    float x0, y0, x1, y1, dy, scale;
    const int16 target_y = 10;

    if (corner_id < 8 || corner_id >= num || i0 < 0 || i1 <= i0) return 0;
    x0 = pts[i0][0] * pixel_per_meter;
    y0 = pts[i0][1] * pixel_per_meter;
    x1 = pts[i1][0] * pixel_per_meter;
    y1 = pts[i1][1] * pixel_per_meter;
    dy = y1 - y0;
    if (fabsf(dy) < 1.0f) return 0;
    if (fabsf((x1 - x0) / dy) < CR_PATCH_K_MIN
        || fabsf((x1 - x0) / dy) > CR_PATCH_K_MAX) return 0;
    scale = ((float)target_y - y1) / dy;
    *far_x = clip((int16)(x1 + (x1 - x0) * scale), 2, MT9V03X_W - 3);
    *far_y = target_y;
    return 1;
}

/* 将补线按每个实际图像行写回边线数组，并同步等距点。
 * 每个 y 只写一个整数 x，避免浮点直线取整后多个点重合造成截断。 */
static void cross_apply_integer_edge(int16 pts[][2], uint16 *count,
                                     float rpts[][2], int16 rnum,
                                     int16 near_x, int16 near_y,
                                     int16 far_x, int16 far_y)
{
    uint16 i;
    float dy = (float)far_y - near_y;
    float k;
    if (!pts || !count || dy >= -1.0f || rnum <= 0) return;
    k = ((float)far_x - near_x) / dy;
    if (fabsf(k) < CR_PATCH_K_MIN || fabsf(k) > CR_PATCH_K_MAX) return;
    for (i = 0; i < *count; i++) {
        int16 y = pts[i][1];
        if (y <= near_y && y >= far_y)
            pts[i][0] = clip((int16)(near_x + k * ((float)y - near_y) + 0.5f), 1, MT9V03X_W - 2);
    }
    for (i = 0; i < (uint16)rnum; i++) {
        int16 y = (int16)(rpts[i][1] * pixel_per_meter + 0.5f);
        if (y <= near_y && y >= far_y)
            rpts[i][0] = (float)clip((int16)(near_x + k * ((float)y - near_y) + 0.5f), 1, MT9V03X_W - 2) / pixel_per_meter;
    }
}

void cross_apply_integer_edges(void)
{
    if (cross_flag < CR_START || !cross_vline_valid) return;
    cross_apply_integer_edge(left_line_points, &left_line_count, rpts0s, rpts0s_num,
                             cv_nl_x, cv_nl_y, cv_fl_x, cv_fl_y);
    cross_apply_integer_edge(right_line_points, &right_line_count, rpts1s, rpts1s_num,
                             cv_nr_x, cv_nr_y, cv_fr_x, cv_fr_y);
}

/* 角点变量定义统一在 image.c（含 far_ 远线系列），本文件只写函数 */

/* ----------------------------------------------------------------------------
 * 近端角点：等距采样边线上的真实折角
 * 点序 rpts0s[0] 在近端（图像底部），索引增大向远端。
 * 夹角 = atan2(|叉积|, 点积)；方向用"折后向图像外侧展开"判定，
 * 不依赖坐标系符号，左右线对称。
 * --------------------------------------------------------------------------*/
static float corner_turn_angle(float pts[][2], int16 i)
{
    float dx_in = pts[i - CR_STEP][0] - pts[i][0];
    float dy_in = pts[i - CR_STEP][1] - pts[i][1];
    float dx_out = pts[i + CR_STEP][0] - pts[i][0];
    float dy_out = pts[i + CR_STEP][1] - pts[i][1];
    float cross = dx_in * dy_out - dy_in * dx_out;
    float dot = dx_in * dx_out + dy_in * dy_out;
    return fabsf(atan2f(cross, dot));
}

void find_corners(void)
{
    int16 i;
    int16 best_l = -1, best_r = -1;
    float best_ang_l = 0.0f, best_ang_r = 0.0f;
    float max_ang_l = 0.0f, max_ang_r = 0.0f;

    is_straight0 = (rpts0s_num > (int16)(1.2f / sample_dist));
    is_straight1 = (rpts1s_num > (int16)(1.2f / sample_dist));
    Lpt0_found = Lpt1_found = N_Lpt0_found = N_Lpt1_found = 0;
    Lpt0_rpts0s_id = Lpt1_rpts1s_id = -1;
    N_Lpt0_rpts0s_id = N_Lpt1_rpts1s_id = -1;
    conf1_max = conf2_max = 0;

    /* ---- 左边线：第一个"向左外展开"的强折角 = 左近端角点 ---- */
    for (i = CR_STEP; i < rpts0s_num - CR_STEP; i++)
    {
        float ang = corner_turn_angle(rpts0s, i);

        if (ang > max_ang_l) max_ang_l = ang;
        /* 非直线：0.36~1.92m 内出现 >8° 折角 */
        if (ang > 0.1396f
            && i > (int16)(0.36f / sample_dist)
            && i < (int16)(1.92f / sample_dist))
        {
            is_straight0 = 0;
        }
    }
    /* 在有效距离内选择最强的外扩局部峰值。
     * 原先采用“第一个超过阈值的峰值”，边线有毛刺时会把噪声折角锁成近端角点。 */
    for (i = CR_STEP + 1; i < rpts0s_num - CR_STEP - 1; i++)
    {
        float ang = corner_turn_angle(rpts0s, i);
        float prev_ang = corner_turn_angle(rpts0s, i - 1);
        float next_ang = corner_turn_angle(rpts0s, i + 1);
        int16 px = (int16)(rpts0s[i][0] * pixel_per_meter);
        int16 px_far = (int16)(rpts0s[i + CR_STEP][0] * pixel_per_meter);
        if (ang >= CR_ANGLE_TH && ang >= prev_ang && ang > next_ang
            && px > CR_SIDE_MIN_X
            && px_far < px - CR_FOLD_SIDE_PX)
        {
            if (ang > best_ang_l)
            {
                best_ang_l = ang;
                best_l = i;
            }
        }
    }
    if (best_l >= 0)
    {
        Lpt0_rpts0s_id = best_l;
        Lpt0_found = 1;
    }
    conf1_max = max_ang_l;   /* 显示：ang*57.3 = 度 */

    /* ---- 右边线：对称 ---- */
    for (i = CR_STEP; i < rpts1s_num - CR_STEP; i++)
    {
        float ang = corner_turn_angle(rpts1s, i);

        if (ang > max_ang_r) max_ang_r = ang;
        if (ang > 0.1396f
            && i > (int16)(0.36f / sample_dist)
            && i < (int16)(1.92f / sample_dist))
        {
            is_straight1 = 0;
        }
    }
    for (i = CR_STEP + 1; i < rpts1s_num - CR_STEP - 1; i++)
    {
        float ang = corner_turn_angle(rpts1s, i);
        float prev_ang = corner_turn_angle(rpts1s, i - 1);
        float next_ang = corner_turn_angle(rpts1s, i + 1);
        int16 px = (int16)(rpts1s[i][0] * pixel_per_meter);
        int16 px_far = (int16)(rpts1s[i + CR_STEP][0] * pixel_per_meter);
        if (ang >= CR_ANGLE_TH && ang >= prev_ang && ang > next_ang
            && px < CR_SIDE_MAX_X
            && px_far > px + CR_FOLD_SIDE_PX)
        {
            if (ang > best_ang_r)
            {
                best_ang_r = ang;
                best_r = i;
            }
        }
    }
    if (best_r >= 0)
    {
        Lpt1_rpts1s_id = best_r;
        Lpt1_found = 1;
    }
    conf2_max = max_ang_r;
}

/* ============================================================================
 * 远端外角点 FL/FR：从近端外角点向上"爬黑白边界"
 *
 * 拓扑依据：十字白区是连通块，近端角点 NL/NR 与远端外角点 FL/FR 同处其
 * 黑白边界上——沿边界爬，角点必然在轨迹上（不依赖宽度/角度峰值等启发式）。
 *
 * 两个关键细节（标准二值图实测，缺一不可）：
 *   1) 必须跟踪【全部轮廓】而不只是外轮廓：对面竖路与侧支路夹出的黑块四面
 *      被白包围时是"洞"，FL 落在洞边界上，只爬外轮廓会漏掉一侧角点。
 *   2) 角点 = 路口中心两侧窗口内"近竖直缘段（对面竖路左右缘）"最靠车头的
 *      端点；横路边是水平段，用局部割线方向 |dy|>=0.8|dx| 排除，竖段末端
 *      再沿轮廓吃 2 个非水平过渡点，精确落在竖缘与横路边的转折处。
 *
 * 标准图验证：FL=(102,47)（手动真值 104,48），FR=(155,50)（真值 156,50）。
 * 仅检测+显示+上报（与 find_corners 同级常开），不截断边线、不接管转向。
 * ==========================================================================*/
#define CRAWL_CONTOUR_MAX   700    /* 单条轮廓最大点数（实测外轮廓 475），存于 wifi_scratch_buf */
#define CRAWL_HALF          3      /* 局部方向割线半窗（像素） */
#define CRAWL_MIN_DY        7      /* 竖缘段最小 y 跨度 */
#define CRAWL_SLOPE_NUM     8      /* 竖直判据：|dy|*10 >= 8*|dx|（0.8） */
#define CRAWL_SLOPE_DEN     10
#define CRAWL_WIN_IN        5      /* 竖缘窗口距路口中心内边 */
#define CRAWL_WIN_OUT       42     /* 竖缘窗口距路口中心外边 */
#define CRAWL_Y_GAP         6      /* 只在近端角点上方 6 行开始找 */
#define CRAWL_MIN_LEN       10     /* 轮廓最短点数（滤孤立噪点） */

/* ---- 零新增 RAM 设计 -------------------------------------------------------
 * visited 直接标记在 image_binary 上（二值图只有 0/255）：
 *   255 = 白未访问；64(CRAWL_MARK_VIS) = 爬线中已访问的白像素；
 *   爬线结束后，远端(y 8..ymax+GAP)访问点转为 200(CRAWL_MARK_TRACE) 供屏幕画
 *   青色轨迹，其余还原 255。200>=128，二值判定仍当白，不污染图像；
 *   下一帧 image_threshold 整体重写 image_binary，标记自然消失。
 * 轮廓坐标借用 wifi_scratch_buf（2820B；process_edge_points 与 wifi_debug
 *   在同核顺序执行不嵌套；WiFi 灰度图直接引用相机缓冲区）。
 * ------------------------------------------------------------------------- */
#define CRAWL_MARK_VIS      64      /* 爬线中：已访问白像素 */
#define CRAWL_MARK_TRACE    200     /* 爬线后：远端轨迹点（>=128 仍判白） */
#define crawl_cx            (wifi_scratch_buf)
#define crawl_cy            (wifi_scratch_buf + CRAWL_CONTOUR_MAX)

/* 爬线结果：crawl_show_ymax 为本帧爬线上界（-1=无角点未爬，屏幕不画）。 */
int16 crawl_show_ymax = -1;

static void crawl_vis_set(int16 x, int16 y)
{
    if (image_binary[y][x] == 255) image_binary[y][x] = CRAWL_MARK_VIS;
}
static int16 crawl_vis_get(int16 x, int16 y)
{
    return (image_binary[y][x] == CRAWL_MARK_VIS) ? 1 : 0;
}
/* display.c 绘制轨迹用：仅远端轨迹标记点返回 1 */
uint8 crawl_trace_pixel(int16 x, int16 y)
{
    if (x < 0 || x >= MT9V03X_W || y < 0 || y >= MT9V03X_H) return 0;
    return (image_binary[y][x] == CRAWL_MARK_TRACE) ? 1u : 0u;
}

/* Moore 8 邻域方向：0=E 1=SE 2=S 3=SW 4=W 5=NW 6=N 7=NE */
static const int8 crawl_dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int8 crawl_dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

static int16 crawl_is_white(int16 x, int16 y)
{
    uint8 v;
    if (x < 0 || x >= MT9V03X_W || y < 0 || y >= MT9V03X_H) return 0;
    v = image_binary[y][x];
    return (v == 255 || v == CRAWL_MARK_VIS) ? 1 : 0;
}

/* 从边界起点 (sx,sy)（白且北邻黑）跟踪一条闭合轮廓，返回点数 */
static int16 crawl_moore(int16 sx, int16 sy)
{
    int16 n = 0;
    int16 x = sx, y = sy;
    int16 d = 6;   /* 起点北邻黑：初始背景方向为 N */

    crawl_vis_set(x, y);
    crawl_cx[n] = (uint8)x; crawl_cy[n] = (uint8)y; n++;

    do
    {
        int16 k, found = 0;
        for (k = 1; k <= 8; k++)
        {
            int16 nd = (d + k) & 7;
            int16 nx = x + crawl_dx[nd];
            int16 ny = y + crawl_dy[nd];
            if (crawl_is_white(nx, ny))
            {
                d = (nd + 4) & 7;
                x = nx; y = ny;
                found = 1;
                break;
            }
        }
        if (!found) break;
        crawl_vis_set(x, y);
        crawl_cx[n] = (uint8)x; crawl_cy[n] = (uint8)y; n++;
        if (n >= CRAWL_CONTOUR_MAX) break;
    } while (!(x == sx && y == sy));

    return n;
}

/* 在 n 点轮廓上提取窗口 [xlo,xhi]、y<=ymax 内近竖直缘段的车头端 */
static void crawl_extract_tip(int16 n, int16 xlo, int16 xhi, int16 ymax,
                              int16 *out_x, int16 *out_y)
{
    int16 i;
    *out_x = -1; *out_y = -1;

    for (i = 0; i < n; i++)
    {
        int16 x = crawl_cx[i], y = crawl_cy[i];
        int16 a = (i - CRAWL_HALF + n) % n;
        int16 b = (i + CRAWL_HALF) % n;
        int16 dx = (int16)crawl_cx[b] - (int16)crawl_cx[a];
        int16 dy = (int16)crawl_cy[b] - (int16)crawl_cy[a];
        int16 adx = (dx >= 0) ? dx : -dx;
        int16 ady = (dy >= 0) ? dy : -dy;
        if (!(x >= xlo && x <= xhi && y <= ymax
              && ady * CRAWL_SLOPE_DEN >= CRAWL_SLOPE_NUM * adx && ady >= 3))
            continue;

        /* 沿轮廓正向扩展连续竖直段 */
        {
            int16 j = i, steps = 0, sylo = y, syhi = y, nj;
            while (steps < n)
            {
                int16 x2, y2, aa, bb, dx2, dy2, ax2, ay2;
                nj = (j + 1) % n;
                x2 = crawl_cx[nj]; y2 = crawl_cy[nj];
                aa = (nj - CRAWL_HALF + n) % n;
                bb = (nj + CRAWL_HALF) % n;
                dx2 = (int16)crawl_cx[bb] - (int16)crawl_cx[aa];
                dy2 = (int16)crawl_cy[bb] - (int16)crawl_cy[aa];
                ax2 = (dx2 >= 0) ? dx2 : -dx2;
                ay2 = (dy2 >= 0) ? dy2 : -dy2;
                if (!(x2 >= xlo && x2 <= xhi && y2 <= ymax
                      && ay2 * CRAWL_SLOPE_DEN >= CRAWL_SLOPE_NUM * ax2 && ay2 >= 3))
                    break;
                if (y2 < sylo) sylo = y2;
                if (y2 > syhi) syhi = y2;
                j = nj; steps++;
                if (nj == i) break;
            }

            if (syhi - sylo >= CRAWL_MIN_DY)
            {
                int16 k, cnt = steps + 1, pi = -1, bx, by, q, t;
                /* 段内 y 最大点（最靠车头） */
                for (k = 0; k < cnt; k++)
                {
                    int16 kk = (i + k) % n;
                    if ((int16)crawl_cy[kk] == syhi) { pi = kk; break; }
                }
                if (pi < 0) pi = i;
                bx = crawl_cx[pi]; by = crawl_cy[pi];
                /* 沿轮廓正/反向各吃 2 个非水平过渡点（竖缘→横路转折） */
                q = j;
                for (t = 0; t < 2; t++)
                {
                    int16 qq = (q + 1) % n;
                    int16 qx = crawl_cx[qq], qy = crawl_cy[qq];
                    int16 sdy = qy - (int16)crawl_cy[q];
                    if (sdy < 0) sdy = -sdy;
                    if (qy > by && sdy >= 1 && qx >= xlo && qx <= xhi && qy <= ymax)
                    { bx = qx; by = qy; q = qq; }
                    else break;
                }
                q = i;
                for (t = 0; t < 2; t++)
                {
                    int16 qq = (q - 1 + n) % n;
                    int16 qx = crawl_cx[qq], qy = crawl_cy[qq];
                    int16 sdy = qy - (int16)crawl_cy[q];
                    if (sdy < 0) sdy = -sdy;
                    if (qy > by && sdy >= 1 && qx >= xlo && qx <= xhi && qy <= ymax)
                    { bx = qx; by = qy; q = qq; }
                    else break;
                }
                if (*out_y < 0 || by > *out_y) { *out_x = bx; *out_y = by; }
            }
        }
    }
}

/* ----------------------------------------------------------------------------
 * 远端外角点主函数：find_corners() 之后调用（常开）。
 * 至少一个近端角点有效才爬；中心 mx 双角点取中点，单角点取迷宫入口中心。
 * 结果填 far_orig/far_rpts/far_Lpt*（display 橙色十字 + WiFi $CORNERS 自动带）。
 * --------------------------------------------------------------------------*/
void find_far_corners_crawl(void)
{
    int16 nl_x = -1, nl_y = -1, nr_x = -1, nr_y = -1;
    int16 mx, ymax, x, y;
    int16 fl_x = -1, fl_y = -1, fr_x = -1, fr_y = -1;

    /* 本帧远端结果先清零，避免沿用上一帧 */
    far_Lpt0_found = 0;
    far_Lpt1_found = 0;
    far_Lpt0_rpts0s_id = -1;
    far_Lpt1_rpts1s_id = -1;
    far_rpts0s_num = 0;
    far_rpts1s_num = 0;
    crawl_show_ymax = -1;

    if (Lpt0_found && Lpt0_rpts0s_id >= 0 && Lpt0_rpts0s_id < rpts0s_num)
    {
        nl_x = (int16)(rpts0s[Lpt0_rpts0s_id][0] * pixel_per_meter);
        nl_y = (int16)(rpts0s[Lpt0_rpts0s_id][1] * pixel_per_meter);
    }
    if (Lpt1_found && Lpt1_rpts1s_id >= 0 && Lpt1_rpts1s_id < rpts1s_num)
    {
        nr_x = (int16)(rpts1s[Lpt1_rpts1s_id][0] * pixel_per_meter);
        nr_y = (int16)(rpts1s[Lpt1_rpts1s_id][1] * pixel_per_meter);
    }
    if (nl_x < 0 && nr_x < 0) return;

    if (nl_x >= 0 && nr_x >= 0)
    {
        mx = (nl_x + nr_x) / 2;
        ymax = (nl_y < nr_y ? nl_y : nr_y) - CRAWL_Y_GAP;
    }
    else
    {
        if (maze_start_left_x >= 0 && maze_start_right_x >= 0)
            mx = (maze_start_left_x + maze_start_right_x) / 2;
        else
            mx = MT9V03X_W / 2;
        ymax = (nl_y >= 0 ? nl_y : nr_y) - CRAWL_Y_GAP;
    }
    if (ymax < 16 || mx < CRAWL_WIN_OUT || mx > MT9V03X_W - CRAWL_WIN_OUT)
        return;

    crawl_show_ymax = ymax;

    /* 枚举近端角点上方区域的边界起点：白且北邻黑（外边界与洞边界都覆盖） */
    for (y = 8; y <= ymax; y++)
    {
        for (x = 2; x < MT9V03X_W - 2; x++)
        {
            if (crawl_is_white(x, y) && !crawl_is_white(x, y - 1)
                && !crawl_vis_get(x, y))
            {
                int16 n = crawl_moore(x, y);
                if (n >= CRAWL_MIN_LEN)
                {
                    int16 lx = -1, ly = -1, rx = -1, ry = -1;
                    int16 xlo = mx - CRAWL_WIN_OUT, xhi = mx - CRAWL_WIN_IN;
                    int16 rlo = mx + CRAWL_WIN_IN, rhi = mx + CRAWL_WIN_OUT;
                    if (xlo < 2) xlo = 2;
                    if (rhi > MT9V03X_W - 3) rhi = MT9V03X_W - 3;
                    crawl_extract_tip(n, xlo, xhi, ymax, &lx, &ly);
                    crawl_extract_tip(n, rlo, rhi, ymax, &rx, &ry);
                    if (lx >= 0 && (fl_y < 0 || ly > fl_y)) { fl_x = lx; fl_y = ly; }
                    if (rx >= 0 && (fr_y < 0 || ry > fr_y)) { fr_x = rx; fr_y = ry; }
                }
            }
        }
    }

    /* 输出：像素填 far_orig，米制填 far_rpts（$CORNERS 发时再 ×pixel_per_meter） */
    if (fl_x >= 0)
    {
        far_orig0[0][0] = (float)fl_x; far_orig0[0][1] = (float)fl_y;
        far_rpts0s[0][0] = (float)fl_x / pixel_per_meter;
        far_rpts0s[0][1] = (float)fl_y / pixel_per_meter;
        far_rpts0s_num = 1;
        far_Lpt0_rpts0s_id = 0;
        far_Lpt0_found = 1;
    }
    if (fr_x >= 0)
    {
        far_orig1[0][0] = (float)fr_x; far_orig1[0][1] = (float)fr_y;
        far_rpts1s[0][0] = (float)fr_x / pixel_per_meter;
        far_rpts1s[0][1] = (float)fr_y / pixel_per_meter;
        far_rpts1s_num = 1;
        far_Lpt1_rpts1s_id = 0;
        far_Lpt1_found = 1;
    }

    /* 恢复 image_binary：远端轮廓点保留为轨迹标记 200（屏幕画青色，对 WiFi 仍为白），
       其余访问标记 64 全部还原白 255。下一帧二值化会整体重写。 */
    {
        int16 trace_yhi = ymax + CRAWL_Y_GAP;
        for (y = 0; y < MT9V03X_H; y++)
        {
            for (x = 0; x < MT9V03X_W; x++)
            {
                if (image_binary[y][x] == CRAWL_MARK_VIS)
                    image_binary[y][x] =
                        ((y >= 8 && y <= trace_yhi) ? CRAWL_MARK_TRACE : 255);
            }
        }
    }
}

/* ----------------------------------------------------------------------------
 * 四角点虚拟补线：find_far_corners_crawl() 之后调用（常开，与十字状态机无关）。
 * 同帧四角点齐全且几何合理时，输出角点像素坐标与虚拟中线两端点：
 *   左补线 NL-FL、右补线 NR-FR；中线 M0=近端两角点中点 → M1=远端两角点中点。
 * 消费方：display.c（屏幕黄补边线/品红补中线）、wifi_spi.c（boundary + $VMID）。
 * --------------------------------------------------------------------------*/
void cross_build_vlines(void)
{
    /* 四角点齐全即锁定、全程保持，与十字状态机解耦：
     * 车在远处（状态机仍 CR_NONE、cross_line_active=0）只要四角点齐全就出补线；
     * 锁定后角点丢失/爬线失效都不影响补线；状态机走过 ENTER/OUT 回 CR_NONE
     * （正常驶出）才解锁；尚未真正进入十字且四角点连续丢 30 帧按误识别解锁。
     * CR_ENTER/CR_OUT 用 find_cross_center 的路口边缘更新远端 FL/FR/M1。 */
    static uint8 vline_locked = 0;
    static uint8 ever_enter = 0;
    static uint8 miss_fr = 0;
    static int16 l_nl_x, l_nl_y, l_nr_x, l_nr_y;
    static int16 l_fl_x, l_fl_y, l_fr_x, l_fr_y;
    int16 nl_x = -1, nl_y = -1, nr_x = -1, nr_y = -1;
    int16 fl_x = -1, fl_y = -1, fr_x = -1, fr_y = -1;
    int16 i, py, join_lx = -1, join_ly = -1, join_rx = -1, join_ry = -1;
    int16 target_fl_x, target_fl_y, target_fr_x, target_fr_y, delta;
    int16 join_ld = 32767, join_rd = 32767, d;

    if (cross_flag >= CR_ENTER) ever_enter = 1;

    /* 正常驶出十字（走过 ENTER/OUT 后回 NONE）：解锁 */
    if (vline_locked && cross_flag == CR_NONE && ever_enter)
    {
        vline_locked = 0;
        ever_enter = 0;
        miss_fr = 0;
        cross_vline_valid = 0;
        return;
    }

    /* 四角点：锁存优先、实时回退（CR_NONE 时 hold 全 -1，走本帧实时检测）。
     * 近端 rpts0s/rpts1s 为米制，×pixel_per_meter 还原像素；far_orig 已是像素。 */
    if (cross_hold_nl_x >= 0 && cross_hold_nl_y >= 0)
    {
        nl_x = cross_hold_nl_x; nl_y = cross_hold_nl_y;
    }
    else if (Lpt0_found && Lpt0_rpts0s_id >= 0 && Lpt0_rpts0s_id < rpts0s_num)
    {
        nl_x = (int16)(rpts0s[Lpt0_rpts0s_id][0] * pixel_per_meter);
        nl_y = (int16)(rpts0s[Lpt0_rpts0s_id][1] * pixel_per_meter);
    }
    if (cross_hold_nr_x >= 0 && cross_hold_nr_y >= 0)
    {
        nr_x = cross_hold_nr_x; nr_y = cross_hold_nr_y;
    }
    else if (Lpt1_found && Lpt1_rpts1s_id >= 0 && Lpt1_rpts1s_id < rpts1s_num)
    {
        nr_x = (int16)(rpts1s[Lpt1_rpts1s_id][0] * pixel_per_meter);
        nr_y = (int16)(rpts1s[Lpt1_rpts1s_id][1] * pixel_per_meter);
    }
    if (cross_hold_fl_x >= 0 && cross_hold_fl_y >= 0)
    {
        fl_x = cross_hold_fl_x; fl_y = cross_hold_fl_y;
    }
    else if (far_Lpt0_found && far_Lpt0_rpts0s_id >= 0 && far_Lpt0_rpts0s_id < far_rpts0s_num)
    {
        fl_x = (int16)far_orig0[far_Lpt0_rpts0s_id][0];
        fl_y = (int16)far_orig0[far_Lpt0_rpts0s_id][1];
    }
    if (cross_hold_fr_x >= 0 && cross_hold_fr_y >= 0)
    {
        fr_x = cross_hold_fr_x; fr_y = cross_hold_fr_y;
    }
    else if (far_Lpt1_found && far_Lpt1_rpts1s_id >= 0 && far_Lpt1_rpts1s_id < far_rpts1s_num)
    {
        fr_x = (int16)far_orig1[far_Lpt1_rpts1s_id][0];
        fr_y = (int16)far_orig1[far_Lpt1_rpts1s_id][1];
    }

    /* 有远端点就直连；缺远端点就按该侧边界斜率延长。左右独立处理，
     * 对应参考代码的四点、左缺、右缺、两侧都缺四种情况。 */
    if (nl_x >= 0 && fl_x < 0)
        (void)cross_extend_far(rpts0s, rpts0s_num, Lpt0_rpts0s_id, &fl_x, &fl_y);
    if (nr_x >= 0 && fr_x < 0)
        (void)cross_extend_far(rpts1s, rpts1s_num, Lpt1_rpts1s_id, &fr_x, &fr_y);

    if (!vline_locked)
    {
        /* 两侧都必须得到有效补线端点（真实远端角点或斜率延长点）。 */
        if (nl_x < 0 || nr_x < 0 || fl_x < 0 || fr_x < 0) { cross_vline_valid = 0; return; }
        /* 几何合理性：远端角点必须在近端上方至少 2 行；左右顺序不能反；x 不出界 */
        if (fl_y >= nl_y - 2 || fr_y >= nr_y - 2) { cross_vline_valid = 0; return; }
        if (nl_x >= nr_x || fl_x >= fr_x) { cross_vline_valid = 0; return; }
        if (nl_x < 2 || nr_x > MT9V03X_W - 3
            || fl_x < 2 || fr_x > MT9V03X_W - 3) { cross_vline_valid = 0; return; }

        /* 首次锁定：四角点算出的补线此后不再受角点检测影响 */
        l_nl_x = nl_x; l_nl_y = nl_y; l_nr_x = nr_x; l_nr_y = nr_y;
        l_fl_x = fl_x; l_fl_y = fl_y; l_fr_x = fr_x; l_fr_y = fr_y;
        vline_locked = 1;
        miss_fr = 0;
    }
    else
    {
        /* 仅 NONE 候选态允许误识别超时解锁。
         * START 表示状态机已确认近端角点，车辆越过角点时不能因实时角点丢失清补线。 */
        if (cross_flag == CR_NONE)
        {
            if (nl_x < 0 && nr_x < 0 && fl_x < 0 && fr_x < 0)
            {
                if (++miss_fr >= 30)
                {
                    vline_locked = 0;
                    ever_enter = 0;
                    miss_fr = 0;
                    cross_vline_valid = 0;
                    return;
                }
            }
            else
            {
                miss_fr = 0;
            }
        }
        /* CR_ENTER/CR_OUT：用路口扫描结果更新远端 FL/FR（M1 跟到对面竖路中心）。
         * find_cross_center 在本函数之后执行，读到的是上帧结果，天然低通；
         * 扫描暂时失败保持锁定值不动。近端 NL/NR 锁定后不再更新。 */
        if (cross_flag >= CR_ENTER && cross_far_y > 0 && cross_max_x > 0
            && cross_edge_l >= 0 && cross_edge_r > cross_edge_l)
        {
            target_fl_x = cross_edge_l;
            target_fl_y = cross_far_y;
            target_fr_x = cross_edge_r;
            target_fr_y = cross_far_y;

            /* 路口扫描边缘会在横向白区内跳点。先限制单帧位移，再做
             * 1/4 低通，避免 M1 左右跳动把纯跟踪目标折成锯齿。 */
            delta = target_fl_x - l_fl_x;
            if (delta > 6) delta = 6;
            if (delta < -6) delta = -6;
            if (delta > 0 && delta < 4) delta = 4;
            if (delta < 0 && delta > -4) delta = -4;
            l_fl_x += delta / 4;
            if (l_fl_x == target_fl_x) l_fl_x = target_fl_x;
            delta = target_fr_x - l_fr_x;
            if (delta > 6) delta = 6;
            if (delta < -6) delta = -6;
            if (delta > 0 && delta < 4) delta = 4;
            if (delta < 0 && delta > -4) delta = -4;
            l_fr_x += delta / 4;
            if (l_fr_x == target_fr_x) l_fr_x = target_fr_x;
            delta = target_fl_y - l_fl_y;
            if (delta > 4) delta = 4;
            if (delta < -4) delta = -4;
            if (delta > 0 && delta < 4) delta = 4;
            if (delta < 0 && delta > -4) delta = -4;
            l_fl_y += delta / 4;
            if (l_fl_y == target_fl_y) l_fl_y = target_fl_y;
            l_fr_y = l_fl_y;
        }
    }

    cv_nl_x = l_nl_x; cv_nl_y = l_nl_y; cv_nr_x = l_nr_x; cv_nr_y = l_nr_y;
    cv_fl_x = l_fl_x; cv_fl_y = l_fl_y; cv_fr_x = l_fr_x; cv_fr_y = l_fr_y;
    cv_m0_x = (l_nl_x + l_nr_x) / 2; cv_m0_y = (l_nl_y + l_nr_y) / 2;
    cv_m1_x = (l_fl_x + l_fr_x) / 2; cv_m1_y = (l_fl_y + l_fr_y) / 2;

    /* 真实中线和虚拟中线必须从同一点交接。近端角点平均值与本行实际
     * 边线平均值会受透视/角点量化影响而相差数像素，直接拼接会形成折角。 */
    for (i = 0; i < rpts0s_num; i++)
    {
        py = (int16)(rpts0s[i][1] * pixel_per_meter);
        d = abs(py - cv_m0_y);
        if (d < join_ld)
        {
            join_ld = d;
            join_lx = (int16)(rpts0s[i][0] * pixel_per_meter);
            join_ly = py;
        }
    }
    for (i = 0; i < rpts1s_num; i++)
    {
        py = (int16)(rpts1s[i][1] * pixel_per_meter);
        d = abs(py - cv_m0_y);
        if (d < join_rd)
        {
            join_rd = d;
            join_rx = (int16)(rpts1s[i][0] * pixel_per_meter);
            join_ry = py;
        }
    }
    if (join_ld <= 5 && join_rd <= 5 && abs(join_ly - join_ry) <= 5
        && join_lx >= 0 && join_rx > join_lx)
    {
        cv_m0_x = (join_lx + join_rx) / 2;
        cv_m0_y = (join_ly + join_ry) / 2;
    }
    cross_vline_valid = 1;
}

/* ----------------------------------------------------------------------------
 * 十字中扫描对面竖路路口中心（列扫描最长白列 + 行扫描确认，二值图版）
 * 注意：本函数不做任何"远端角点"几何检测，只输出路口中心 cross_max_x/cross_far_y
 *       和路口左右边缘 cross_edge_l/r（far_ 系列仅为复用显示/协议通道）。
 * 仅在 cross_enter / cross_out 状态调用；结果：
 *   cross_max_x = 对面路口中心 x（像素），cross_far_y = 路口行
 *   cross_edge_l/r = 路口左右边缘，同时填 far_Lpt0/far_Lpt1（WiFi/显示）
 * --------------------------------------------------------------------------*/
void find_cross_center(void)
{
    int16 base, fi_x, yy, low;
    int16 best_hight, aim_x, hight;
    int16 xl, xr, middle;
    int16 pre_l = -1, pre_r = -1, pre2_l = -1, pre2_r = -1;
    int16 scan_l, scan_r;

    if (cross_flag == CR_NONE)
    {
        /* 保留本帧 find_far_corners_crawl() 的远端角点，供随后
         * cross_line_completion() 做四角点进入判定。 */
        cross_far_y = -1;
        cross_edge_l = cross_edge_r = -1;
        return;
    }
    if (cross_flag == CR_START)
    {
        /* CR_START 保留 find_far_corners_crawl 本帧爬线结果（far_ 系列），
         * 供 cross_line_completion 锁存 h_fl/h_fr；本阶段不扫描路口中心。 */
        cross_far_y = -1;
        cross_edge_l = cross_edge_r = -1;
        return;
    }

    /* CR_ENTER/CR_OUT：清掉爬线角点，改用下面列+行扫描得到的对面路口边缘 */
    far_Lpt0_found = 0;
    far_Lpt1_found = 0;
    far_Lpt0_rpts0s_id = -1;
    far_Lpt1_rpts1s_id = -1;
    far_rpts0s_num = 0;
    far_rpts1s_num = 0;

    /* ---- 动态中线：全车道宽度内找从车头连续延伸最远的白列。
     * 十字中原有边线会跳到横路，不能再仅围绕其中心 x±20 搜索；
     * 最长连续白列才是对面竖路的可靠前视基准。 */
    if (maze_start_left_x >= 0 && maze_start_right_x >= 0)
        base = (maze_start_left_x + maze_start_right_x) / 2;
    else
        base = s_aim_x;
    scan_l = (maze_start_left_x >= 2) ? maze_start_left_x + 2 : base - CR_SCAN_X_HALF;
    scan_r = (maze_start_right_x < PERS_W - 2) ? maze_start_right_x - 2 : base + CR_SCAN_X_HALF;
    if (scan_l < 2) scan_l = 2;
    if (scan_r > PERS_W - 3) scan_r = PERS_W - 3;
    if (scan_r - scan_l < 12)
    {
        scan_l = base - CR_SCAN_X_HALF;
        scan_r = base + CR_SCAN_X_HALF;
        if (scan_l < 2) scan_l = 2;
        if (scan_r > PERS_W - 3) scan_r = PERS_W - 3;
    }
    best_hight = CR_ROW_START_Y;
    aim_x = base;
    for (fi_x = scan_l; fi_x <= scan_r; fi_x += 2)
    {
        for (yy = CR_ROW_START_Y; yy >= 4; yy--)
        {
            if (image_binary[yy][fi_x] < EDGE_WHITE_THRESHOLD) break;
        }
        if (yy < best_hight) { best_hight = yy; aim_x = fi_x; }
    }
    if (best_hight < CR_ROW_START_Y) s_aim_x = aim_x;  /* 找到才更新保持 */

    /* ---- 行扫描：从车头向上，在 aim_x 两侧找连续稳定的竖路口宽度 ---- */
    middle = -1;
    hight = best_hight;
    for (low = CR_ROW_START_Y; low > best_hight + 2; low -= 3)
    {
        xl = aim_x;
        xr = aim_x;
        /* 目标列在该行必须为白（落在黑缝/噪声上时整行跳过） */
        if (image_binary[low][aim_x] < EDGE_WHITE_THRESHOLD) continue;
        while (xl > 1 && image_binary[low][xl - 1] >= EDGE_WHITE_THRESHOLD) xl--;
        while (xr < PERS_W - 2 && image_binary[low][xr + 1] >= EDGE_WHITE_THRESHOLD) xr++;

        /* 还在横路段（触界/过宽）：稳定性历史作废，继续上扫 */
        if (xl <= CR_ROW_EDGE_XL || xr >= CR_ROW_EDGE_XR || xr - xl >= CR_ROW_W_MAX)
        {
            pre2_l = pre2_r = pre_l = pre_r = -1;
            continue;
        }
        if (xr - xl <= CR_ROW_W_MIN) continue;

        if (pre_l >= 0 && pre2_l >= 0
            && abs(xl - pre_l) <= CR_ROW_STABLE && abs(xr - pre_r) <= CR_ROW_STABLE
            && abs(xl - pre2_l) <= CR_ROW_STABLE && abs(xr - pre2_r) <= CR_ROW_STABLE)
        {
            middle = (pre_l + pre_r) / 2;
            hight = low;
            xl = pre_l; xr = pre_r;     /* 稳定行边缘作为路口边缘 */
            break;
        }
        pre2_l = pre_l; pre2_r = pre_r;
        pre_l = xl;     pre_r = xr;
    }

    if (middle >= 0)
    {
        s_mid_x = middle;
        cross_max_x = middle;
    }
    else
    {
        cross_max_x = s_aim_x;          /* 行扫描未确认：用最长白列 */
    }
    cross_far_y = hight;
    cross_edge_l = xl;
    cross_edge_r = xr;

    /* ---- 填 far_ 系列（原图像素 + 米制），供 display / WiFi ---- */
    if (xl >= 0 && xr > xl)
    {
        far_orig0[0][0] = (float)xl;  far_orig0[0][1] = (float)hight;
        far_orig1[0][0] = (float)xr;  far_orig1[0][1] = (float)hight;
        far_rpts0s[0][0] = (float)xl / pixel_per_meter;
        far_rpts0s[0][1] = (float)hight / pixel_per_meter;
        far_rpts1s[0][0] = (float)xr / pixel_per_meter;
        far_rpts1s[0][1] = (float)hight / pixel_per_meter;
        far_rpts0s_num = 1;
        far_rpts1s_num = 1;
        far_Lpt0_rpts0s_id = 0;
        far_Lpt1_rpts1s_id = 0;
        far_Lpt0_found = 1;
        far_Lpt1_found = 1;
    }
}

/* ----------------------------------------------------------------------------
 * 十字状态机（每帧在 find_corners / find_cross_center 之后调用）
 * none：左右近端角点同时有效，且左右边线同时触界/丢线 → start
 * start：入口张开/丢失 → enter；里程超时撤销
 * enter：列+行扫描找路口中心（find_cross_center）；稳定识别到出口竖道 → out
 * out ：底部竖路宽恢复 40~48 → none
 * --------------------------------------------------------------------------*/
void cross_line_completion(void)
{
    /* 角点像素坐标（本帧失效则为 -1） */
    int16 nl_x = -1, nl_y = -1, nr_x = -1, nr_y = -1;
    static int16 h_nl_x = -1, h_nl_y = -1, h_nr_x = -1, h_nr_y = -1;
    static int16 h_fl_x = -1, h_fl_y = -1, h_fr_x = -1, h_fr_y = -1;
    /* 同一个横路在 CR_OUT->CR_NONE 后仍可能保留近端角点数帧，
     * 设定最小重臂距离，避免单十字被二次确认而重新补线绕圈。 */
    static float cross_rearm_until = 0.0f;
    uint8 cross_candidate;
    uint8 exit_lane_ok = 0;
    int16 entry_w = -1;
    int16 near_span = -1;
    int16 near_y_gap = -1;
    float enc;
    cross_phase_distance = (cross_flag == CR_NONE) ? 0.0f
                          : (total_distance_m - cross_enc_base);

    if (Lpt0_found && Lpt0_rpts0s_id >= 0 && Lpt0_rpts0s_id < rpts0s_num)
    {
        nl_x = (int16)(rpts0s[Lpt0_rpts0s_id][0] * pixel_per_meter);
        nl_y = (int16)(rpts0s[Lpt0_rpts0s_id][1] * pixel_per_meter);
    }
    if (Lpt1_found && Lpt1_rpts1s_id >= 0 && Lpt1_rpts1s_id < rpts1s_num)
    {
        nr_x = (int16)(rpts1s[Lpt1_rpts1s_id][0] * pixel_per_meter);
        nr_y = (int16)(rpts1s[Lpt1_rpts1s_id][1] * pixel_per_meter);
    }
    if (maze_start_left_x >= 0 && maze_start_right_x >= 0)
        entry_w = maze_start_right_x - maze_start_left_x;

    cross_near_pair_valid = 0;
    cross_geometry_valid = 0;
    if (nl_x >= 0 && nr_x >= 0)
    {
        near_span = nr_x - nl_x;
        near_y_gap = (nl_y > nr_y) ? (nl_y - nr_y) : (nr_y - nl_y);
        if (near_span >= CR_NEAR_SPAN_MIN && near_y_gap <= CR_NEAR_Y_GAP_MAX)
            cross_near_pair_valid = 1;
    }
    if (cross_near_pair_valid && cross_vline_valid
        && Lpt0_rpts0s_id >= 0 && Lpt1_rpts1s_id >= 0
        && Lpt0_rpts0s_id < CR_ID_DUAL && Lpt1_rpts1s_id < CR_ID_DUAL)
        cross_geometry_valid = 1;

    if (rpts0s_num >= CR_EXIT_MIN_PTS && rpts1s_num >= CR_EXIT_MIN_PTS)
    {
        int16 exit_w = (int16)((rpts1s[5][0] - rpts0s[5][0]) * pixel_per_meter);
        if (exit_w >= CR_EXIT_W_LO && exit_w <= CR_EXIT_W_HI) exit_lane_ok = 1;
    }
    /* 出口必须同时满足：近车端双边线恢复标准窄车道，且原图底部可见竖道。
     * 仅凭其中一个条件，在双十字横路中仍会出现短暂成立。 */
    cross_exit_lane_valid = (exit_lane_ok && maze_start_y > CR_OUT_START_Y
                             && maze_start_left_x >= 0 && maze_start_right_x >= 0
                             && entry_w > 0 && entry_w < CR_OUT_START_W) ? 1 : 0;

    switch (cross_flag)
    {
    case CR_NONE:
    {
        cross_exit_confirm_frames = 0;
        cross_exit_lane_valid = 0;
        /* 新逆透视图中十字两侧边线未必碰到图像边界，触界仅作调试信息。
         * 用双近角横向间距、同高性、补线几何和连续帧确认，避免弯道单帧误判。 */
        cross_candidate = (Lpt0_found && Lpt1_found && cross_geometry_valid
                           && total_distance_m >= cross_rearm_until) ? 1 : 0;
        if (cross_candidate)
        {
            if (cross_candidate_frames < CR_CONFIRM_FRAMES)
                cross_candidate_frames++;
        }
        else
        {
            cross_candidate_frames = 0;
        }
        if (cross_candidate_frames >= CR_CONFIRM_FRAMES)
        {
            cross_flag = CR_START;
            cross_enc_base = total_distance_m;
            if (nl_x >= 0) { h_nl_x = nl_x; h_nl_y = nl_y; }
            if (nr_x >= 0) { h_nr_x = nr_x; h_nr_y = nr_y; }
        }
        break;
    }

    case CR_START:
    {
        cross_candidate_frames = CR_CONFIRM_FRAMES;
        if (nl_x >= 0) { h_nl_x = nl_x; h_nl_y = nl_y; }
        if (nr_x >= 0) { h_nr_x = nr_x; h_nr_y = nr_y; }
        /* 提前锁存爬线远端角点：车驶过近端角点后实时角点丢失、爬线失去起点，
         * 没有这份锁存 cross_vline_valid 会掉 0，三条补线和虚拟中线全灭。 */
        if (far_Lpt0_found) { h_fl_x = (int16)far_orig0[0][0]; h_fl_y = (int16)far_orig0[0][1]; }
        if (far_Lpt1_found) { h_fr_x = (int16)far_orig1[0][0]; h_fr_y = (int16)far_orig1[0][1]; }

        /* 连续三帧已经确认近角几何，双十字不再等待入口宽度/角点同高。
         * 这些条件在横路遮挡下会失效，导致长期卡在 CR_START，车辆绕行。 */
        cross_flag = CR_ENTER;
        cross_enc_base = total_distance_m;
        cross_out_enc_base = total_distance_m;
        cross_exit_confirm_frames = 0;
        break;
    }

    case CR_ENTER:
    {
        cross_candidate_frames = CR_CONFIRM_FRAMES;
        /* find_cross_center 已在本帧填好路口边缘 far_ 系列；锁存 */
        if (far_Lpt0_found) { h_fl_x = (int16)far_orig0[0][0]; h_fl_y = (int16)far_orig0[0][1]; }
        if (far_Lpt1_found) { h_fr_x = (int16)far_orig1[0][0]; h_fr_y = (int16)far_orig1[0][1]; }

        enc = total_distance_m - cross_enc_base;
        /* 0.33m 前无条件保持。之后只有连续看到出口窄竖道才驶出；
         * 双十字横路中单帧/短暂窄道不够，固定里程只用于视觉失效保护。 */
        if (enc > CR_M_ENTER_MIN && cross_exit_lane_valid)
        {
            if (cross_exit_confirm_frames < CR_EXIT_STABLE)
                cross_exit_confirm_frames++;
        }
        else
        {
            cross_exit_confirm_frames = 0;
        }
        if (cross_exit_confirm_frames >= CR_EXIT_STABLE || enc > CR_M_ENTER_TO)
        {
            cross_flag = CR_OUT;
            cross_out_enc_base = total_distance_m;
        }
        break;
    }

    case CR_OUT:
    default:
    {
        cross_candidate_frames = CR_CONFIRM_FRAMES;
        if (cross_exit_lane_valid)
        {
            if (cross_exit_confirm_frames < CR_EXIT_STABLE) cross_exit_confirm_frames++;
        }
        else cross_exit_confirm_frames = 0;
        if (cross_exit_confirm_frames >= CR_EXIT_STABLE
            || total_distance_m - cross_out_enc_base > CR_M_OUT_TO)
        {
            /* 约 0.30m 的重臂距离足以越过本十字出口，
             * 又短于常见双十字间距，不阻塞下一处真实十字。 */
            cross_rearm_until = total_distance_m + 0.30f;
            cross_flag = CR_NONE;
        }
        break;
    }
    }

    /* ---- 输出 cross_line_active 与锁存角点（WiFi/蜂鸣） ---- */
    if (cross_flag == CR_NONE)
    {
        cross_line_active = 0;
        cross_hold_nl_x = cross_hold_nl_y = -1;
        cross_hold_nr_x = cross_hold_nr_y = -1;
        cross_hold_fl_x = cross_hold_fl_y = -1;
        cross_hold_fr_x = cross_hold_fr_y = -1;
        h_nl_x = h_nl_y = h_nr_x = h_nr_y = -1;
        h_fl_x = h_fl_y = h_fr_x = h_fr_y = -1;
        cross_far_y = -1;
        cross_edge_l = cross_edge_r = -1;
        cross_max_x = MT9V03X_W / 2;
        s_aim_x = s_mid_x = MT9V03X_W / 2;
    }
    else
    {
        cross_line_active = 1;
        cross_hold_nl_x = h_nl_x; cross_hold_nl_y = h_nl_y;
        cross_hold_nr_x = h_nr_x; cross_hold_nr_y = h_nr_y;
        cross_hold_fl_x = h_fl_x; cross_hold_fl_y = h_fl_y;
        cross_hold_fr_x = h_fr_x; cross_hold_fr_y = h_fr_y;
    }
}

/* ================= 蜂鸣器统一输出：大弯道 + 十字提示 =================
 * 十字：cross_line_active 上升沿（状态机 none→start）响 cross_buzz_times 次。 */
void beeper_poll(void)
{
    static uint8 cb_prev = 0;
    static int16 cb_remain = 0;
    static int16 cb_on = 0;
    static int16 cb_off = 0;
    uint8 cb_now;
    uint8 beep = 0;

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

    if (buzzer_tick > 0)
    {
        buzzer_tick--;
        beep = 1;
    }
    if (beep) beep_on(); else beep_off();
}
