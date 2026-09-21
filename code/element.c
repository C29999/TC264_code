#include "element.h"
#include "image.h"
#include "beep.h"
#include <string.h>

/* data.c 定义（data.h 同声明）：累计行驶距离（米），十字状态机时序用 */

/* ============================================================================
 * 十字路口检测（移植自逐飞 STC32 开源例程当前启用写法）
 *
 * 思路（与旧"找四个角点+几何补线"完全不同）：
 *   1) 近端角点 = 边线上"前后方向向量真实夹角 ≥75°"的第一个折点
 *      （等距采样点上取前后各 14 点的入/出向量，atan2 求夹角；
 *        比"x 跳变 N 像素"抗噪，弯道渐变不会误触发）。
 *   2) 识别 = NL/NR/FL/FR 四角点同帧全部有效，避免弯道误判。
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
#define CR_SIDE_MIN_X    30       /* 左角点 x 下限（排除贴边噪声） */
#define CR_SIDE_MAX_X    158      /* 右角点 x 上限（178-20） */
#define CR_FOLD_SIDE_PX  8        /* 折后必须向外侧展开的最小位移（像素） */

/* ---- start → enter 参数 ---- */
#define CR_ENTRY_W_PX    48       /* 两角点/入口横向张开 > 48px 视为进入横路 */
#define CR_ENTRY_Y_PX    90       /* 两角点 y 都 > 90（进入近端）视为进入 */

/* ---- enter 列/行扫描参数（二值图版，替代原局部大津） ---- */
#define CR_SCAN_TOP_Y    90       /* 列扫描起始行（车头附近，原 car_head_y-10） */
#define CR_SCAN_X_HALF   20       /* 列扫描 x 范围 ±20 */
#define CR_ROW_START_Y   98       /* 行扫描起始行 */
#define CR_ROW_EDGE_XL   30       /* 路口左边缘必须 > 30（否则还在横路段） */
#define CR_ROW_EDGE_XR   158      /* 路口右边缘必须 < 158 */
#define CR_ROW_W_MAX     120      /* 行宽 ≥120 视为仍在横路，继续上扫 */
#define CR_ROW_W_MIN     10       /* 行宽下限（滤噪） */
#define CR_ROW_STABLE    5        /* 连续三行边界差 ≤5px 视为路口稳定 */

/* ---- enter → out / out → none 参数 ---- */
#define CR_OUT_START_Y   93       /* 对面入口回到 y>93 */
#define CR_OUT_START_W   125      /* 对面入口宽度 < 125 */
#define CR_EXIT_W_LO     40       /* 驶出：底部竖路宽 40~48px */
#define CR_EXIT_W_HI     48

/* ---- 里程阈值（米，total_distance_m，已按 11485 脉冲/米标定；
 *      对应参考例程单侧脉冲 10000/5000/15000/2000 的物理距离） ---- */
#define CR_M_START_TO     0.9f    /* start 超时撤销 */
#define CR_M_ENTER_MIN    0.44f   /* enter 最短行驶距离 */
#define CR_M_ENTER_TO     1.3f    /* enter 超时强制 out */
#define CR_M_OUT_TO       0.17f   /* out 超时结束 */

/* ---- 蜂鸣（与旧版一致） ---- */
#define CROSS_BUZZ_ON_FRAMES  8
#define CROSS_BUZZ_GAP_FRAMES 6

/* ===================== 十字状态机状态 ===================== */
/* CR_NONE/CR_START/CR_ENTER/CR_OUT 定义见 image.h */
uint8  cross_flag = CR_NONE;     /* 当前十字状态（calculation_error 消费） */
static float cross_enc_base = 0.0f; /* 当前十字阶段的起始里程（米） */
int16  cross_max_x = MT9V03X_W / 2; /* 对面路口中心 x（像素，导航目标） */
int16  cross_far_y = -1;            /* 对面路口所在行（像素） */
int16  cross_edge_l = -1;           /* 对面路口左边缘（像素，显示用） */
int16  cross_edge_r = -1;           /* 对面路口右边缘（像素，显示用） */
static int16 s_aim_x = MT9V03X_W / 2;   /* 列扫描目标保持（对应 cross_max_x） */
static int16 s_mid_x = MT9V03X_W / 2;   /* 行扫描成功的路口中心保持 */
static int16 s_far_y = -1;               /* 最近一次有效的对面入口行 */
static int16 s_edge_l = -1, s_edge_r = -1; /* 最近一次有效的对面入口边缘 */

/* cross_line_active / cross_hold 定义（WiFi $CORNERS 与蜂鸣依赖） */
uint8 cross_line_active = 0;
int16 cross_hold_nl_x = -1, cross_hold_nl_y = -1;
int16 cross_hold_nr_x = -1, cross_hold_nr_y = -1;
int16 cross_hold_fl_x = -1, cross_hold_fl_y = -1;
int16 cross_hold_fr_x = -1, cross_hold_fr_y = -1;

/* ===================== 十字虚拟边线与导航走廊 =====================
 * 四个角点 NL/NR（近端折角）与 FL/FR（远端爬线）同帧全部有效时：
 *   虚拟左边线 = NL-FL 直线，虚拟右边线 = NR-FR 直线（补出横路段断边）；
 *   虚拟中线   = M0((NL+NR)/2) → M1((FL+FR)/2)（补边线的中点连线）。
 * START 阶段锁存四角点，ENTER/OUT 阶段改画车头到对面入口的动态走廊；
 * calculation_error 使用同一个对面入口中心作为转向目标。 */
uint8 cross_vline_valid = 0;
int16 cv_nl_x = -1, cv_nl_y = -1, cv_nr_x = -1, cv_nr_y = -1;
int16 cv_fl_x = -1, cv_fl_y = -1, cv_fr_x = -1, cv_fr_y = -1;
int16 cv_m0_x = -1, cv_m0_y = -1, cv_m1_x = -1, cv_m1_y = -1;

/* 角点变量定义统一在 image.c（含 far_ 远线系列），本文件只写函数 */

/* ----------------------------------------------------------------------------
 * 近端角点：等距采样边线上的真实折角
 * 点序 rpts0s[0] 在近端（图像底部），索引增大向远端。
 * 夹角 = atan2(|叉积|, 点积)；方向用"折后向图像外侧展开"判定，
 * 不依赖坐标系符号，左右线对称。
 * --------------------------------------------------------------------------*/
void find_corners(void)
{
    int16 i;
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
        float dx_in  = rpts0s[i - CR_STEP][0] - rpts0s[i][0]; /* 指向近端 */
        float dy_in  = rpts0s[i - CR_STEP][1] - rpts0s[i][1];
        float dx_out = rpts0s[i + CR_STEP][0] - rpts0s[i][0]; /* 指向远端 */
        float dy_out = rpts0s[i + CR_STEP][1] - rpts0s[i][1];
        float cross  = dx_in * dy_out - dy_in * dx_out;
        float dot    = dx_in * dx_out + dy_in * dy_out;
        float ang    = fabsf(atan2f(cross, dot));
        int16 px     = (int16)(rpts0s[i][0] * pixel_per_meter);
        int16 px_far = (int16)(rpts0s[i + CR_STEP][0] * pixel_per_meter);

        if (ang > max_ang_l) max_ang_l = ang;
        /* 非直线：0.36~1.92m 内出现 >8° 折角 */
        if (ang > 0.1396f
            && i > (int16)(0.36f / sample_dist)
            && i < (int16)(1.92f / sample_dist))
        {
            is_straight0 = 0;
        }
        /* 第一个强折角，且折后向左侧外展开，位置不在贴边噪声区 */
        if (!Lpt0_found && ang >= CR_ANGLE_TH
            && px > CR_SIDE_MIN_X
            && px_far < px - CR_FOLD_SIDE_PX)
        {
            Lpt0_rpts0s_id = i;
            Lpt0_found = 1;
        }
    }
    conf1_max = max_ang_l;   /* 显示：ang*57.3 = 度 */

    /* ---- 右边线：对称 ---- */
    for (i = CR_STEP; i < rpts1s_num - CR_STEP; i++)
    {
        float dx_in  = rpts1s[i - CR_STEP][0] - rpts1s[i][0];
        float dy_in  = rpts1s[i - CR_STEP][1] - rpts1s[i][1];
        float dx_out = rpts1s[i + CR_STEP][0] - rpts1s[i][0];
        float dy_out = rpts1s[i + CR_STEP][1] - rpts1s[i][1];
        float cross  = dx_in * dy_out - dy_in * dx_out;
        float dot    = dx_in * dx_out + dy_in * dy_out;
        float ang    = fabsf(atan2f(cross, dot));
        int16 px     = (int16)(rpts1s[i][0] * pixel_per_meter);
        int16 px_far = (int16)(rpts1s[i + CR_STEP][0] * pixel_per_meter);

        if (ang > max_ang_r) max_ang_r = ang;
        if (ang > 0.1396f
            && i > (int16)(0.36f / sample_dist)
            && i < (int16)(1.92f / sample_dist))
        {
            is_straight1 = 0;
        }
        if (!Lpt1_found && ang >= CR_ANGLE_TH
            && px < CR_SIDE_MAX_X
            && px_far > px + CR_FOLD_SIDE_PX)
        {
            Lpt1_rpts1s_id = i;
            Lpt1_found = 1;
        }
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

/* 访问标记使用独立位图，四角点检测全程只读 image_binary，不影响普通寻线。 */
#define crawl_cx            (wifi_scratch_buf)
#define crawl_cy            (wifi_scratch_buf + CRAWL_CONTOUR_MAX)
#define CRAWL_VIS_BYTES     ((MT9V03X_W * MT9V03X_H + 7) / 8)
static uint8 crawl_visited[CRAWL_VIS_BYTES];

/* 爬线结果：crawl_show_ymax 为本帧爬线上界（-1=无角点未爬，屏幕不画）。 */
int16 crawl_show_ymax = -1;

static void crawl_vis_set(int16 x, int16 y)
{
    uint32 id = (uint32)y * MT9V03X_W + (uint32)x;
    crawl_visited[id >> 3] |= (uint8)(0x80u >> (id & 7));
}
static int16 crawl_vis_get(int16 x, int16 y)
{
    uint32 id = (uint32)y * MT9V03X_W + (uint32)x;
    return (crawl_visited[id >> 3] & (uint8)(0x80u >> (id & 7))) ? 1 : 0;
}
/* display.c 绘制本帧爬线轨迹。 */
uint8 crawl_trace_pixel(int16 x, int16 y)
{
    if (x < 0 || x >= MT9V03X_W || y < 0 || y >= MT9V03X_H) return 0;
    return crawl_vis_get(x, y) ? 1u : 0u;
}

/* Moore 8 邻域方向：0=E 1=SE 2=S 3=SW 4=W 5=NW 6=N 7=NE */
static const int8 crawl_dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int8 crawl_dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

static int16 crawl_is_white(int16 x, int16 y)
{
    uint8 v;
    if (x < 0 || x >= MT9V03X_W || y < 0 || y >= MT9V03X_H) return 0;
    v = image_binary[y][x];
    return (v >= EDGE_WHITE_THRESHOLD) ? 1 : 0;
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
    memset(crawl_visited, 0, sizeof(crawl_visited));

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

}

/* ----------------------------------------------------------------------------
 * 四角点虚拟补线：find_far_corners_crawl() 之后调用（常开，与十字状态机无关）。
 * 同帧四角点齐全且几何合理时，输出角点像素坐标与虚拟中线两端点：
 *   左补线 NL-FL、右补线 NR-FR；中线 M0=近端两角点中点 → M1=远端两角点中点。
 * 消费方：display.c（屏幕黄补边线/品红补中线）、wifi_spi.c（boundary + $VMID）。
 * --------------------------------------------------------------------------*/
void cross_build_vlines(void)
{
    int16 nl_x = -1, nl_y = -1, nr_x = -1, nr_y = -1;
    int16 fl_x = -1, fl_y = -1, fr_x = -1, fr_y = -1;

    cross_vline_valid = 0;

    /* 角点检测在后台常开只为触发十字；正常寻线不生成任何补线。 */
    if (!cross_line_active) return;

    /* 车已进入十字后，旧近端角点已经驶到车后。此时用车头处标准赛道宽
     * 连接当前扫描到的对面入口，形成随画面更新的左右导航走廊。 */
    if (cross_line_active && cross_flag >= CR_ENTER
        && cross_far_y > 0 && cross_max_x > 0)
    {
        int16 target_l = cross_edge_l;
        int16 target_r = cross_edge_r;
        if (target_l < 0 || target_r <= target_l)
        {
            target_l = cross_max_x - TRACK_HALF_W;
            target_r = cross_max_x + TRACK_HALF_W;
        }
        if (target_l < 2) target_l = 2;
        if (target_r > MT9V03X_W - 3) target_r = MT9V03X_W - 3;
        cv_nl_x = MT9V03X_W / 2 - TRACK_HALF_W;
        cv_nl_y = MT9V03X_H - 10;
        cv_nr_x = MT9V03X_W / 2 + TRACK_HALF_W;
        cv_nr_y = MT9V03X_H - 10;
        cv_fl_x = target_l; cv_fl_y = cross_far_y;
        cv_fr_x = target_r; cv_fr_y = cross_far_y;
        cv_m0_x = MT9V03X_W / 2; cv_m0_y = MT9V03X_H - 10;
        cv_m1_x = cross_max_x; cv_m1_y = cross_far_y;
        cross_vline_valid = 1;
        return;
    }

    /* START 阶段优先使用状态机锁存值，角点短暂漏检时补线不闪断。 */
    if (cross_line_active)
    {
        nl_x = cross_hold_nl_x; nl_y = cross_hold_nl_y;
        nr_x = cross_hold_nr_x; nr_y = cross_hold_nr_y;
        fl_x = cross_hold_fl_x; fl_y = cross_hold_fl_y;
        fr_x = cross_hold_fr_x; fr_y = cross_hold_fr_y;
    }

    /* 近端角点：rpts0s/rpts1s 为米制，×pixel_per_meter 还原像素 */
    if (nl_x < 0 && Lpt0_found && Lpt0_rpts0s_id >= 0 && Lpt0_rpts0s_id < rpts0s_num)
    {
        nl_x = (int16)(rpts0s[Lpt0_rpts0s_id][0] * pixel_per_meter);
        nl_y = (int16)(rpts0s[Lpt0_rpts0s_id][1] * pixel_per_meter);
    }
    if (nr_x < 0 && Lpt1_found && Lpt1_rpts1s_id >= 0 && Lpt1_rpts1s_id < rpts1s_num)
    {
        nr_x = (int16)(rpts1s[Lpt1_rpts1s_id][0] * pixel_per_meter);
        nr_y = (int16)(rpts1s[Lpt1_rpts1s_id][1] * pixel_per_meter);
    }
    /* 远端角点：far_orig 已是像素坐标 */
    if (fl_x < 0 && far_Lpt0_found && far_Lpt0_rpts0s_id >= 0 && far_Lpt0_rpts0s_id < far_rpts0s_num)
    {
        fl_x = (int16)far_orig0[far_Lpt0_rpts0s_id][0];
        fl_y = (int16)far_orig0[far_Lpt0_rpts0s_id][1];
    }
    if (fr_x < 0 && far_Lpt1_found && far_Lpt1_rpts1s_id >= 0 && far_Lpt1_rpts1s_id < far_rpts1s_num)
    {
        fr_x = (int16)far_orig1[far_Lpt1_rpts1s_id][0];
        fr_y = (int16)far_orig1[far_Lpt1_rpts1s_id][1];
    }

    /* 四点必须全有效 */
    if (nl_x < 0 || nr_x < 0 || fl_x < 0 || fr_x < 0) return;

    /* 几何合理性：远端角点必须在近端上方至少 2 行；左右顺序不能反；x 不出界 */
    if (fl_y >= nl_y - 2 || fr_y >= nr_y - 2) return;
    if (nl_x >= nr_x || fl_x >= fr_x) return;
    if (nl_x < 2 || nr_x > MT9V03X_W - 3
        || fl_x < 2 || fr_x > MT9V03X_W - 3) return;

    cv_nl_x = nl_x; cv_nl_y = nl_y; cv_nr_x = nr_x; cv_nr_y = nr_y;
    cv_fl_x = fl_x; cv_fl_y = fl_y; cv_fr_x = fr_x; cv_fr_y = fr_y;
    cv_m0_x = (nl_x + nr_x) / 2; cv_m0_y = (nl_y + nr_y) / 2;
    cv_m1_x = (fl_x + fr_x) / 2; cv_m1_y = (fl_y + fr_y) / 2;
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
    int16 xl = -1, xr = -1, middle;
    int16 pre_l = -1, pre_r = -1, pre2_l = -1, pre2_r = -1;
    uint8 column_found = 0;

    far_Lpt0_found = 0;
    far_Lpt1_found = 0;
    far_Lpt0_rpts0s_id = -1;
    far_Lpt1_rpts1s_id = -1;
    far_rpts0s_num = 0;
    far_rpts1s_num = 0;

    if (cross_flag != CR_ENTER && cross_flag != CR_OUT)
    {
        cross_far_y = -1;
        cross_edge_l = cross_edge_r = -1;
        return;
    }

    /* ---- 列扫描：在目标 x±20 内逐列向上找白段顶端，取最远（hight 最小）列 ---- */
    if (maze_start_left_x >= 0 && maze_start_right_x >= 0)
        base = (maze_start_left_x + maze_start_right_x) / 2;
    else
        base = s_aim_x;
    best_hight = CR_SCAN_TOP_Y + 1;
    aim_x = base;
    for (fi_x = base - CR_SCAN_X_HALF; fi_x <= base + CR_SCAN_X_HALF; fi_x += 4)
    {
        if (fi_x < 2 || fi_x > PERS_W - 3) continue;
        for (yy = CR_SCAN_TOP_Y; yy >= 4; yy -= 4)
        {
            /* 黑顶 + 下方两像素白 = 白色段顶端 */
            if (image_binary[yy][fi_x] < EDGE_WHITE_THRESHOLD
                && image_binary[yy + 1][fi_x] >= EDGE_WHITE_THRESHOLD
                && image_binary[yy + 2][fi_x] >= EDGE_WHITE_THRESHOLD)
            {
                if (yy < best_hight)
                {
                    best_hight = yy;
                    aim_x = fi_x;
                    column_found = 1;
                }
                break;
            }
        }
    }
    if (column_found) s_aim_x = aim_x;

    /* ---- 行扫描：从车头向上，在 aim_x 两侧找连续稳定的竖路口宽度 ---- */
    middle = -1;
    hight = best_hight;
    for (low = CR_ROW_START_Y; column_found && low > best_hight + 2; low -= 3)
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
        s_far_y = hight;
        s_edge_l = xl;
        s_edge_r = xr;
    }
    else if (column_found)
    {
        s_mid_x = s_aim_x;              /* 行扫描未确认：用最长白列 */
        if (s_far_y < 0) s_far_y = best_hight;
    }
    cross_max_x = s_mid_x;
    cross_far_y = s_far_y;
    cross_edge_l = s_edge_l;
    cross_edge_r = s_edge_r;

    /* ---- 填 far_ 系列（原图像素 + 米制），供 display / WiFi ---- */
    if (cross_edge_l >= 0 && cross_edge_r > cross_edge_l && cross_far_y > 0)
    {
        far_orig0[0][0] = (float)cross_edge_l;  far_orig0[0][1] = (float)cross_far_y;
        far_orig1[0][0] = (float)cross_edge_r;  far_orig1[0][1] = (float)cross_far_y;
        far_rpts0s[0][0] = (float)cross_edge_l / pixel_per_meter;
        far_rpts0s[0][1] = (float)cross_far_y / pixel_per_meter;
        far_rpts1s[0][0] = (float)cross_edge_r / pixel_per_meter;
        far_rpts1s[0][1] = (float)cross_far_y / pixel_per_meter;
        far_rpts0s_num = 1;
        far_rpts1s_num = 1;
        far_Lpt0_rpts0s_id = 0;
        far_Lpt1_rpts1s_id = 0;
        far_Lpt0_found = 1;
        far_Lpt1_found = 1;
    }
}

/* ----------------------------------------------------------------------------
 * 十字状态机（每帧在 find_corners / find_far_corners_crawl 之后调用）
 * none：NL/NR/FL/FR 四角点同帧齐全 → start
 * start：入口张开/丢失 → enter；里程超时撤销
 * enter：列+行扫描找路口中心（find_cross_center）；对面入口回到底部 → out
 * out ：底部竖路宽恢复 40~48 → none
 * --------------------------------------------------------------------------*/
void cross_line_completion(void)
{
    /* 角点像素坐标（本帧失效则为 -1） */
    int16 nl_x = -1, nl_y = -1, nr_x = -1, nr_y = -1;
    static int16 h_nl_x = -1, h_nl_y = -1, h_nr_x = -1, h_nr_y = -1;
    static int16 h_fl_x = -1, h_fl_y = -1, h_fr_x = -1, h_fr_y = -1;
    uint8 four_corners;
    int16 entry_w = -1;
    float enc;

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

    /* 爬线远端角点必须在 find_cross_center 清空/改写前锁存。 */
    if (far_Lpt0_found)
    {
        h_fl_x = (int16)far_orig0[0][0];
        h_fl_y = (int16)far_orig0[0][1];
    }
    if (far_Lpt1_found)
    {
        h_fr_x = (int16)far_orig1[0][0];
        h_fr_y = (int16)far_orig1[0][1];
    }

    switch (cross_flag)
    {
    case CR_NONE:
    {
        four_corners = (Lpt0_found && Lpt1_found
                && far_Lpt0_found && far_Lpt1_found
                && Lpt0_rpts0s_id < CR_ID_DUAL
                && Lpt1_rpts1s_id < CR_ID_DUAL) ? 1 : 0;
        if (four_corners)
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
        if (nl_x >= 0) { h_nl_x = nl_x; h_nl_y = nl_y; }
        if (nr_x >= 0) { h_nr_x = nr_x; h_nr_y = nr_y; }

        /* 入口丢失、入口张开 >48px、或两角点都进入 y>90 → 车进入横路 */
        if (maze_start_y < 0
            || entry_w > CR_ENTRY_W_PX
            || (nl_x >= 0 && nr_x >= 0
                && (nr_x - nl_x > CR_ENTRY_W_PX
                    || (nl_y > CR_ENTRY_Y_PX && nr_y > CR_ENTRY_Y_PX))))
        {
            cross_flag = CR_ENTER;
            cross_enc_base = total_distance_m;
        }
        else if (total_distance_m - cross_enc_base > CR_M_START_TO)
        {
            cross_flag = CR_NONE;   /* 超时撤销（误识别） */
        }
        break;
    }

    case CR_ENTER:
    {
        enc = total_distance_m - cross_enc_base;
        /* 对面竖路入口重新出现在底部且宽度恢复 */
        if (enc > CR_M_ENTER_MIN
            && maze_start_y > CR_OUT_START_Y
            && maze_start_left_x >= 0 && maze_start_right_x >= 0
            && entry_w < CR_OUT_START_W)
        {
            cross_flag = CR_OUT;
            cross_enc_base = total_distance_m;
        }
        else if (enc > CR_M_ENTER_TO)
        {
            cross_flag = CR_OUT;   /* 超时强制驶出 */
            cross_enc_base = total_distance_m;
        }
        break;
    }

    case CR_OUT:
    default:
    {
        uint8 width_ok = 0;
        if (rpts0s_num > 5 && rpts1s_num > 5)
        {
            int16 w = (int16)((rpts1s[5][0] - rpts0s[5][0]) * pixel_per_meter);
            if (w >= CR_EXIT_W_LO && w <= CR_EXIT_W_HI) width_ok = 1;
        }
        if (width_ok || total_distance_m - cross_enc_base > CR_M_OUT_TO)
        {
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
        s_far_y = s_edge_l = s_edge_r = -1;
    }
    else
    {
        cross_line_active = 1;
        cross_hold_nl_x = h_nl_x; cross_hold_nl_y = h_nl_y;
        cross_hold_nr_x = h_nr_x; cross_hold_nr_y = h_nr_y;
        cross_hold_fl_x = h_fl_x; cross_hold_fl_y = h_fl_y;
        cross_hold_fr_x = h_fr_x; cross_hold_fr_y = h_fr_y;

        /* 所有原始边线数据保持不变；ENTER 后仅控制与显示切换到十字分支。 */
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
