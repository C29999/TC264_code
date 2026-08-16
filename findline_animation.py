# -*- coding: utf-8 -*-
"""
左右手巡线算法动画演示 (IDE 风格代码面板 + 高亮当前行红底 + 全程黑框完整囊括)
图像尺寸: 188 (宽) x 120 (高)  —— 完全对应 MT9V03X_W=188, MT9V03X_H=120

三阶段动画:
  阶段1. 扫描找白点(左手起点)  - 从底部中央向左扫, 找"白→黑"跳变点
  阶段2. 扫描找白点(右手起点)  - 从底部中央向右扫, 找"黑→白"跳变点
  阶段3. 贴墙爬线(左右手同步)  - while 三分支 + turn<4 保护

用法:
    python findline_animation.py            # 弹窗 + 保存所有文件到 D:/AI OUT
    python findline_animation.py --noshow   # 只保存, 不弹窗
    python findline_animation.py --nosave   # 只弹窗, 不保存
"""

import argparse
import io
import os
import logging
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle

# 先把 matplotlib 字体缺失的烦人 warning 降到 ERROR 级别 (不影响画图, 只去日志噪音)
logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)
logging.getLogger("matplotlib.mathtext").setLevel(logging.ERROR)

# =========================================================
# 中文字体 + 等宽字体 (Windows 微软雅黑 / 黑体, 解决中文变方块 + 等宽代码)
# =========================================================
import matplotlib.font_manager as fm
_CN_FONT_PATHS = [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"]
_CN_MONO_PATHS = [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simsun.ttc"]
_cn_font = None
for _p in _CN_FONT_PATHS:
    if os.path.exists(_p):
        try:
            _cn_font = fm.FontProperties(fname=_p)
            fm.fontManager.addfont(_p)
            break
        except Exception:
            pass
for _p in _CN_MONO_PATHS:
    if os.path.exists(_p):
        try:
            fm.fontManager.addfont(_p)
        except Exception:
            pass

# 全局 rcParams: 无衬线中文 + 等宽代码, 确保中文字形存在
plt.rcParams["font.sans-serif"] = [
    "Microsoft YaHei", "微软雅黑", "SimHei", "黑体",
    "DejaVu Sans", "Arial",
]
# 等宽字体优先级: 先 Windows 自带等宽, 再带中文的等宽 fallback, 最后默认
plt.rcParams["font.monospace"] = [
    "Cascadia Code", "Consolas", "Courier New",
    "Microsoft YaHei", "SimSun", "NSimSun",
    "DejaVu Sans Mono",
]
plt.rcParams["axes.unicode_minus"] = False
CN_PROPS = _cn_font
# 代码/行号用: 优先等宽 (C代码英文用), fallback 到中文无衬线处理注释
_CODE_FONT_FAMILY = ["Cascadia Code", "Consolas", "Courier New",
                     "Microsoft YaHei", "SimHei", "DejaVu Sans Mono"]

# =========================================================
# 图像尺寸 (和你的 TC264 项目完全一致)
# =========================================================
W = 188
H = 120
EDGE_WHITE_THRESHOLD = 128

# =========================================================
# 方向表 —— 和你 image.c 里的 edge_dir_* 一模一样
# 0上, 1右, 2下, 3左
# =========================================================
edge_dir_front = np.array([
    [ 0, -1],   # 0 上
    [ 1,  0],   # 1 右
    [ 0,  1],   # 2 下
    [-1,  0],   # 3 左
], dtype=np.int32)

edge_dir_frontleft = np.array([
    [-1, -1],   # 0 左上
    [ 1, -1],   # 1 右上
    [ 1,  1],   # 2 右下
    [-1,  1],   # 3 左下
], dtype=np.int32)

edge_dir_frontright = np.array([
    [ 1, -1],   # 0 右上
    [ 1,  1],   # 1 右下
    [-1,  1],   # 2 左下
    [-1, -1],   # 3 左上
], dtype=np.int32)

DIR_NAME = ["上 ↑", "右 →", "下 ↓", "左 ←"]

# =========================================================
# 用户提供的完整版 C 代码 (原封不动, 逐行保存)
# =========================================================
_CODE_RAW = r"""// 二值图中白色像素的判断阈值。
#define EDGE_WHITE_THRESHOLD (128)

// 当前方向的正前方坐标变化表。
// 0表示向上，1表示向右，2表示向下，3表示向左。
static const int16 edge_dir_front[4][2] =
{
    { 0, -1 },
    { 1,  0 },
    { 0,  1 },
    {-1,  0 }
};

// 当前方向的左前方坐标变化表。
static const int16 edge_dir_frontleft[4][2] =
{
    {-1, -1},
    { 1, -1},
    { 1,  1},
    {-1,  1}
};

// 当前方向的右前方坐标变化表。
static const int16 edge_dir_frontright[4][2] =
{
    { 1, -1},
    { 1,  1},
    {-1,  1},
    {-1, -1}
};

// 左手巡线函数。
void findline_lefthand_binary(
    const uint8 binary[MT9V03X_H][MT9V03X_W],
    int16 start_x,
    int16 start_y,
    int16 points[][2],
    uint16 *point_count)
{
    // 保存输出数组的最大容量。
    uint16 max_points;

    // 保存当前搜索点的横坐标。
    int16 x;

    // 保存当前搜索点的纵坐标。
    int16 y;

    // 保存已经找到的边线点数量。
    uint16 step;

    // 保存当前行进方向。
    int16 dir;

    // 保存连续转弯次数。
    int16 turn;

    // 检查输入图像、输出数组和数量指针是否有效。
    if ((binary == 0) || (points == 0) || (point_count == 0))
    {
        // 参数错误时直接退出。
        return;
    }

    // 读取调用者传入的数组容量。
    max_points = *point_count;

    // 先把输出数量清零。
    *point_count = 0;

    // 如果数组容量为零，则无法保存边线点。
    if (max_points == 0)
    {
        // 直接结束函数。
        return;
    }

    // 起点不能位于图像最外圈。
    if ((start_x <= 0) ||
        (start_x >= (MT9V03X_W - 1)) ||
        (start_y <= 0) ||
        (start_y >= (MT9V03X_H - 1)))
    {
        // 起点越界时直接结束。
        return;
    }

    // 起点必须是白色区域。
    if (binary[start_y][start_x] < EDGE_WHITE_THRESHOLD)
    {
        // 起点是黑色时不进行巡线。
        return;
    }

    // 将起点横坐标赋给当前坐标。
    x = start_x;

    // 将起点纵坐标赋给当前坐标。
    y = start_y;

    // 初始方向设置为向上。
    dir = 0;

    // 初始连续转弯次数为零。
    turn = 0;

    // 初始找到的点数量为零。
    step = 0;

    // 满足条件时持续搜索边线。
    while ((step < max_points) &&
           (x > 0) &&
           (x < (MT9V03X_W - 1)) &&
           (y > 0) &&
           (y < (MT9V03X_H - 1)) &&
           (turn < 4))
    {
        // 计算正前方的横坐标。
        int16 front_x = x + edge_dir_front[dir][0];

        // 计算正前方的纵坐标。
        int16 front_y = y + edge_dir_front[dir][1];

        // 计算左前方的横坐标。
        int16 frontleft_x = x + edge_dir_frontleft[dir][0];

        // 计算左前方的纵坐标。
        int16 frontleft_y = y + edge_dir_frontleft[dir][1];

        // 读取正前方像素。
        uint8 front_value = binary[front_y][front_x];

        // 读取左前方像素。
        uint8 frontleft_value = binary[frontleft_y][frontleft_x];

        // 如果正前方是黑色，说明前方不能直行。
        if (front_value < EDGE_WHITE_THRESHOLD)
        {
            // 左手巡线遇到障碍时向右转。
            dir = (dir + 1) % 4;

            // 连续转弯次数加一。
            turn++;
        }

        // 如果正前方是白色、左前方是黑色，则向前直走。
        else if (frontleft_value < EDGE_WHITE_THRESHOLD)
        {
            // 按当前方向更新横坐标。
            x += edge_dir_front[dir][0];

            // 按当前方向更新纵坐标。
            y += edge_dir_front[dir][1];

            // 保存当前点的横坐标。
            points[step][0] = x;

            // 保存当前点的纵坐标。
            points[step][1] = y;

            // 找到的点数量加一。
            step++;

            // 成功前进后清零转弯次数。
            turn = 0;
        }

        // 如果正前方和左前方都是白色，则向左前方走。
        else
        {
            // 按左前方方向更新横坐标。
            x += edge_dir_frontleft[dir][0];

            // 按左前方方向更新纵坐标。
            y += edge_dir_frontleft[dir][1];

            // 当前方向逆时针转动一个方向。
            dir = (dir + 3) % 4;

            // 保存当前点的横坐标。
            points[step][0] = x;

            // 保存当前点的纵坐标。
            points[step][1] = y;

            // 找到的点数量加一。
            step++;

            // 成功前进后清零转弯次数。
            turn = 0;
        }
    }

    // 将实际找到的边线点数量返回。
    *point_count = step;
}

// ============================================================

// 右手巡线函数。
void findline_righthand_binary(
    const uint8 binary[MT9V03X_H][MT9V03X_W],
    int16 start_x,
    int16 start_y,
    int16 points[][2],
    uint16 *point_count)
{
    // 保存输出数组的最大容量。
    uint16 max_points;

    // 保存当前搜索点的横坐标。
    int16 x;

    // 保存当前搜索点的纵坐标。
    int16 y;

    // 保存已经找到的边线点数量。
    uint16 step;

    // 保存当前行进方向。
    int16 dir;

    // 保存连续转弯次数。
    int16 turn;

    // 检查输入图像、输出数组和数量指针是否有效。
    if ((binary == 0) || (points == 0) || (point_count == 0))
    {
        // 参数错误时直接退出。
        return;
    }

    // 读取调用者传入的数组容量。
    max_points = *point_count;

    // 先把输出数量清零。
    *point_count = 0;

    // 如果数组容量为零，则无法保存边线点。
    if (max_points == 0)
    {
        // 直接结束函数。
        return;
    }

    // 起点不能位于图像最外圈。
    if ((start_x <= 0) ||
        (start_x >= (MT9V03X_W - 1)) ||
        (start_y <= 0) ||
        (start_y >= (MT9V03X_H - 1)))
    {
        // 起点越界时直接结束。
        return;
    }

    // 起点必须是白色区域。
    if (binary[start_y][start_x] < EDGE_WHITE_THRESHOLD)
    {
        // 起点是黑色时不进行巡线。
        return;
    }

    // 将起点横坐标赋给当前坐标。
    x = start_x;

    // 将起点纵坐标赋给当前坐标。
    y = start_y;

    // 初始方向设置为向上。
    dir = 0;

    // 初始连续转弯次数为零。
    turn = 0;

    // 初始找到的点数量为零。
    step = 0;

    // 满足条件时持续搜索边线。
    while ((step < max_points) &&
           (x > 0) &&
           (x < (MT9V03X_W - 1)) &&
           (y > 0) &&
           (y < (MT9V03X_H - 1)) &&
           (turn < 4))
    {
        // 计算正前方的横坐标。
        int16 front_x = x + edge_dir_front[dir][0];

        // 计算正前方的纵坐标。
        int16 front_y = y + edge_dir_front[dir][1];

        // 计算右前方的横坐标。
        int16 frontright_x = x + edge_dir_frontright[dir][0];

        // 计算右前方的纵坐标。
        int16 frontright_y = y + edge_dir_frontright[dir][1];

        // 读取正前方像素。
        uint8 front_value = binary[front_y][front_x];

        // 读取右前方像素。
        uint8 frontright_value = binary[frontright_y][frontright_x];

        // 如果正前方是黑色，说明前方不能直行。
        if (front_value < EDGE_WHITE_THRESHOLD)
        {
            // 右手巡线遇到障碍时向左转。
            dir = (dir + 3) % 4;

            // 连续转弯次数加一。
            turn++;
        }

        // 如果正前方是白色、右前方是黑色，则向前直走。
        else if (frontright_value < EDGE_WHITE_THRESHOLD)
        {
            // 按当前方向更新横坐标。
            x += edge_dir_front[dir][0];

            // 按当前方向更新纵坐标。
            y += edge_dir_front[dir][1];

            // 保存当前点的横坐标。
            points[step][0] = x;

            // 保存当前点的纵坐标。
            points[step][1] = y;

            // 找到的点数量加一。
            step++;

            // 成功前进后清零转弯次数。
            turn = 0;
        }

        // 如果正前方和右前方都是白色，则向右前方走。
        else
        {
            // 按右前方方向更新横坐标。
            x += edge_dir_frontright[dir][0];

            // 按右前方方向更新纵坐标。
            y += edge_dir_frontright[dir][1];

            // 当前方向顺时针转动一个方向。
            dir = (dir + 1) % 4;

            // 保存当前点的横坐标。
            points[step][0] = x;

            // 保存当前点的纵坐标。
            points[step][1] = y;

            // 找到的点数量加一。
            step++;

            // 成功前进后清零转弯次数。
            turn = 0;
        }
    }

    // 将实际找到的边线点数量返回。
    *point_count = step;
}
"""
CODE_LINES = _CODE_RAW.split("\n")
_CODE_TOTAL = len(CODE_LINES)
_CODE_VIEW_N = 78   # 一次看几行(IDE风格字体字号合适就清晰)

# =========================================================
# 高亮映射表: (阶段, 动作) → 用户 C 代码里要高亮的行号范围 [start, end] (1-based, 闭区间)
# 对照上面 CODE_LINES 真实行号数出来:
# =========================================================
# 左手三分支行号 (1-based):
#   ① 前方黑 → 右转: 行 219 (if) + 220~225 (代码块里 dir/turn++)
#   ② 前白+左前黑 → 直走: 行 229 (else if) + 230~247 (x/y/step/turn)
#   ③ 都白 → 斜前+左转: 行 251 (else) + 252~272
#   while 行: 193~198
#   计算 f/frontleft 那几句: 201~216
# 右手三分支行号:
#   ① 前方黑 → 左转: 行 387~393
#   ② 前白+右前黑 → 直走: 行 397~415
#   ③ 都白 → 斜前+右转: 行 419~440
#   while 行: 361~366
#   计算 front/frontright 那几句: 368~384
# 阶段①② 扫描 → 高亮 方向表和宏定义 (行 84~112) 和 初始化行 (178~190)
HIGHLIGHT_TABLE = {
    "phase1":   [(83, 112), (178, 190)],      # 扫左手起点 → 宏+方向表+左手初始化
    "phase2":   [(178, 200), (345, 360)],     # 扫右手起点 → 左右手初始化/while前
    "left_branch1":  [(219, 225)],             # 左手 ①前方黑右转 (dir+1, turn++)
    "left_branch2":  [(229, 247)],             # 左手 ②前白+左前黑 直走
    "left_branch3":  [(251, 272)],             # 左手 ③都白 斜前+左转
    "left_compute":  [(193, 218)],             # 左手 while+计算坐标
    "right_branch1": [(387, 393)],             # 右手 ①前方黑左转
    "right_branch2": [(397, 415)],             # 右手 ②前白+右前黑 直走
    "right_branch3": [(419, 440)],             # 右手 ③都白 斜前+右转
    "right_compute": [(361, 386)],             # 右手 while+计算坐标
}


def collect_highlight_keys(i, phase1_N, phase2_N, sl, sr):
    """把当前帧对应的高亮 key 列表返回"""
    keys = []
    if i < phase1_N:
        keys = ["phase1"]
        return keys
    if i < phase1_N + phase2_N:
        keys = ["phase2"]
        return keys

    # 阶段③爬线: 左右手同步, 两边都高亮各自的行
    def branch_of(s):
        act = s.get("action", "")
        if "START" in act:
            return "compute"
        if "①" in act and "左转" in act:   # 右手 "前方黑→左转"
            return "branch1"
        if "①" in act and "右转" in act:   # 左手 "前方黑→右转"
            return "branch1"
        if "②" in act:
            return "branch2"
        if "③" in act:
            return "branch3"
        if "结束" in act:
            return "compute"
        return "compute"

    # 左手
    bl = branch_of(sl)
    if bl == "compute":
        keys.append("left_compute")
    else:
        # 每步先算坐标再分支, 所以 compute+branch 一起亮
        keys.append("left_compute")
        keys.append("left_" + bl)
    # 右手
    br = branch_of(sr)
    if br == "compute":
        keys.append("right_compute")
    else:
        keys.append("right_compute")
        keys.append("right_" + br)
    return keys


def highlight_rows_in_view(hl_keys, view_start, view_end):
    """返回 viewport 内需要高亮红底的行号 set (0-based, 数组索引, 相对 CODE_LINES)"""
    rows = set()
    for k in hl_keys:
        for (start_1b, end_1b) in HIGHLIGHT_TABLE.get(k, []):
            s = max(start_1b - 1, view_start)     # 转 0-based, 夹到视口内
            e = min(end_1b   - 1, view_end - 1)
            if s <= e:
                rows.update(range(s, e + 1))
    return rows


def code_viewport(i, phase1_N, phase2_N, hl_keys):
    """返回 (view_start, view_end), 尽量把高亮区段放在视口中央"""
    # 先收集所有高亮行的中心行号, 用这些行号找中心
    anchors = []
    for k in hl_keys:
        for (s, e) in HIGHLIGHT_TABLE.get(k, []):
            anchors.append((s + e) // 2)
    # 再叠加默认区段
    if i < phase1_N:
        anchors.append(90)
    elif i < phase1_N + phase2_N:
        anchors.append(250)
    else:
        anchors.append(240)
    center = int(np.mean(anchors))
    s = center - _CODE_VIEW_N // 2
    s = max(0, min(s, _CODE_TOTAL - _CODE_VIEW_N))
    e = s + _CODE_VIEW_N
    if e > _CODE_TOTAL:
        e = _CODE_TOTAL
        s = max(0, e - _CODE_VIEW_N)
    return s, e


# =========================================================
# IDE 风格代码面板渲染: 逐行绘制 + 红底高亮 + 完整黑外框
# =========================================================
_CODE_ARTIFACTS = []   # 缓存上一帧画的 patches/texts, 每帧先清掉

def clear_code_artifacts():
    for a in _CODE_ARTIFACTS:
        try:
            a.remove()
        except Exception:
            pass
    _CODE_ARTIFACTS.clear()


def render_code_panel(ax_code, phase_label, hl_keys, view_start, view_end):
    clear_code_artifacts()

    # 颜色方案 (Material Dark IDE 配色)
    BG_PANEL    = "#273137"   # 代码区背景
    BG_LINENUM  = "#1e262b"   # 行号栏背景
    COLOR_CODE  = "#dbe4eb"   # 普通代码字色
    COLOR_COMMENT = "#82c48c" # 注释字色 (偏绿)
    COLOR_LINENUM = "#7a8a95" # 行号字色
    COLOR_HL_BG = "#e53935"   # 高亮行背景(正红)
    COLOR_HL_FG = "#ffffff"   # 高亮行前景(白)
    COLOR_OUTER = "#000000"   # 完整外框(纯黑)
    COLOR_TITLE_BG = "#0d47a1"# 阶段标题栏底
    COLOR_TITLE_FG = "#ffffff"

    # --- 0. 确定 axes 坐标空间 (0~10 横竖, 方便百分比定位) ---
    X_MIN, X_MAX = 0, 10
    Y_MIN, Y_MAX = 0, 10
    ax_code.set_xlim(X_MIN, X_MAX)
    ax_code.set_ylim(Y_MIN, Y_MAX)
    ax_code.set_facecolor(BG_PANEL)
    ax_code.set_xticks([]); ax_code.set_yticks([])

    # --- 0.1 完整外黑框 (保证四周全部囊括, 绝对不切边) ---
    # Rectangle 用 axes 坐标, (xy 左下, width, height)
    outer = Rectangle(
        (0, 0), 1, 1,
        transform=ax_code.transAxes,
        fill=False, linewidth=2.4, edgecolor=COLOR_OUTER, zorder=50, clip_on=False,
    )
    ax_code.add_patch(outer); _CODE_ARTIFACTS.append(outer)

    # --- 1. 顶部阶段标题条 ---
    TITLE_H = 1.10  # axes Y 单位
    # 整个 title 条背景 (从顶往下占 TITLE_H)
    title_rect = Rectangle(
        (0, 1 - TITLE_H / (Y_MAX - Y_MIN)), 1, TITLE_H / (Y_MAX - Y_MIN),
        transform=ax_code.transAxes,
        facecolor=COLOR_TITLE_BG, edgecolor=COLOR_OUTER, linewidth=1.8, zorder=49, clip_on=False,
    )
    ax_code.add_patch(title_rect); _CODE_ARTIFACTS.append(title_rect)

    # 阶段标题 主副标题两行
    vlines_info = f"代码: {view_start+1}~{min(view_end, _CODE_TOTAL)} / 共 {_CODE_TOTAL} 行 (滚动视口)"
    t1 = ax_code.text(0.5, (1 - TITLE_H/(Y_MAX-Y_MIN)) + (TITLE_H/(Y_MAX-Y_MIN))*0.74,
                      f"当 前 阶 段:  {phase_label}",
                      transform=ax_code.transAxes,
                      ha="center", va="center",
                      fontsize=13.2, fontweight="bold", color=COLOR_TITLE_FG, zorder=51)
    t2 = ax_code.text(0.5, (1 - TITLE_H/(Y_MAX-Y_MIN)) + (TITLE_H/(Y_MAX-Y_MIN))*0.26,
                      vlines_info,
                      transform=ax_code.transAxes,
                      ha="center", va="center",
                      fontsize=9.6, color="#b7d7ff", zorder=51)
    _CODE_ARTIFACTS.extend([t1, t2])

    # --- 2. 代码区域坐标 (标题下方, 留一点边距) ---
    CODE_TOP_AX = 1 - (TITLE_H + 0.25) / (Y_MAX - Y_MIN)   # axes Y: 顶部开始
    CODE_BOTTOM_AX = 0.04 / (Y_MAX - Y_MIN)
    CODE_LEFT_AX = 0.03
    CODE_RIGHT_AX = 0.97
    LINENUM_RATIO = 0.085   # 行号栏占代码区宽度 8.5%

    view_total_n = view_end - view_start
    if view_total_n <= 0:
        return
    row_h = ((CODE_TOP_AX - CODE_BOTTOM_AX) * (Y_MAX - Y_MIN)) / view_total_n  # 每格行高(axes Y 空间单位)
    row_h_axes = (CODE_TOP_AX - CODE_BOTTOM_AX) / view_total_n

    # --- 2.1 行号栏整列背景 ---
    x0_linenum = CODE_LEFT_AX
    x1_linenum = CODE_LEFT_AX + (CODE_RIGHT_AX - CODE_LEFT_AX) * LINENUM_RATIO
    r1 = Rectangle(
        (x0_linenum, CODE_BOTTOM_AX),
        x1_linenum - x0_linenum, CODE_TOP_AX - CODE_BOTTOM_AX,
        transform=ax_code.transAxes,
        facecolor=BG_LINENUM, edgecolor="none", zorder=5, clip_on=True,
    )
    ax_code.add_patch(r1); _CODE_ARTIFACTS.append(r1)

    # 行号栏和代码区的分隔线
    p1, = ax_code.plot([x1_linenum, x1_linenum], [CODE_BOTTOM_AX, CODE_TOP_AX],
                       color="#000000", lw=1.2, transform=ax_code.transAxes, zorder=15, clip_on=True)
    _CODE_ARTIFACTS.append(p1)

    # --- 2.2 先画高亮行红底 ---
    hl_rows = highlight_rows_in_view(hl_keys, view_start, view_end)
    for row_arr_idx in hl_rows:
        rel_idx = row_arr_idx - view_start   # 0..view_total_n-1
        # 行顶Y axes坐标: CODE_TOP_AX - (rel_idx + 0)*row_h_axes
        y_top = CODE_TOP_AX - rel_idx * row_h_axes
        y_bot = y_top - row_h_axes
        hr = Rectangle(
            (x0_linenum, y_bot),
            CODE_RIGHT_AX - x0_linenum, y_top - y_bot,
            transform=ax_code.transAxes,
            facecolor=COLOR_HL_BG, edgecolor="none", alpha=0.92, zorder=8, clip_on=True,
        )
        ax_code.add_patch(hr); _CODE_ARTIFACTS.append(hr)

    # 上略/下略指示条 (视口上下边界处加个半透明渐变条视觉提示)
    if view_start > 0:
        top_shade = Rectangle(
            (x0_linenum, CODE_TOP_AX - 0.6*row_h_axes),
            CODE_RIGHT_AX - x0_linenum, 0.6*row_h_axes,
            transform=ax_code.transAxes,
            facecolor="#ffd54f", edgecolor="none", alpha=0.45, zorder=7, clip_on=True,
        )
        ax_code.add_patch(top_shade); _CODE_ARTIFACTS.append(top_shade)
    if view_end < _CODE_TOTAL:
        bot_shade = Rectangle(
            (x0_linenum, CODE_BOTTOM_AX),
            CODE_RIGHT_AX - x0_linenum, 0.6*row_h_axes,
            transform=ax_code.transAxes,
            facecolor="#ffd54f", edgecolor="none", alpha=0.45, zorder=7, clip_on=True,
        )
        ax_code.add_patch(bot_shade); _CODE_ARTIFACTS.append(bot_shade)

    # --- 3. 逐行画行号 + 代码文本 ---
    # 字号按行数自适应
    fs_code = 8.3
    if _CODE_VIEW_N <= 55:  fs_code = 9.2
    elif _CODE_VIEW_N <= 70: fs_code = 8.6
    elif _CODE_VIEW_N >= 90: fs_code = 7.7

    for rel_idx in range(view_total_n):
        arr_idx = view_start + rel_idx
        if arr_idx >= _CODE_TOTAL:
            break
        raw_line = CODE_LINES[arr_idx]
        is_hl = (arr_idx in hl_rows)

        y_center = CODE_TOP_AX - (rel_idx + 0.5) * row_h_axes  # 行中心 axes Y

        # 行号 (行号栏居中)
        ln_text = str(arr_idx + 1)
        ln_color = COLOR_HL_FG if is_hl else COLOR_LINENUM
        ln_fontweight = "bold" if is_hl else "normal"
        tx_ln = ax_code.text(
            (x0_linenum + x1_linenum) / 2, y_center, ln_text,
            transform=ax_code.transAxes, ha="center", va="center",
            fontsize=fs_code * 0.95, color=ln_color, fontweight=ln_fontweight,
            family=_CODE_FONT_FAMILY, zorder=20, clip_on=True,
        )
        _CODE_ARTIFACTS.append(tx_ln)

        # 代码文本 (行号栏右一点开始)
        x_code_text = x1_linenum + 0.006
        # 判断是不是注释 (开头 // 或 /*) 来改颜色
        stripped = raw_line.lstrip()
        is_comment = stripped.startswith("//") or stripped.startswith("/*") or stripped.startswith("*")
        if is_hl:
            fg = COLOR_HL_FG
        else:
            fg = COLOR_COMMENT if is_comment else COLOR_CODE
        fw = "bold" if (is_hl and "if" in stripped[:5]) else "normal"
        tx_cd = ax_code.text(
            x_code_text, y_center, raw_line,
            transform=ax_code.transAxes, ha="left", va="center",
            fontsize=fs_code, color=fg, fontweight=fw,
            family="Consolas, monospace, DejaVu Sans Mono", zorder=20, clip_on=True,
        )
        _CODE_ARTIFACTS.append(tx_cd)


# =========================================================
# 1. 生成模拟赛道二值图 (188x120)
# =========================================================
def make_track():
    img = np.zeros((H, W), dtype=np.uint8)
    cx = W // 2
    track_w = 34
    for y in range(60, H):
        left = cx - track_w // 2
        right = cx + track_w // 2
        img[y, left:right + 1] = 255

    # 左大弯
    for y in range(30, 61):
        dy = 60 - y
        outer_r = 45 + track_w // 2
        inner_r = 45 - track_w // 2
        cy = 60
        cx_curve = cx - 45
        for x in range(W):
            dx = x - cx_curve
            r2 = dx*dx + dy*dy
            if inner_r*inner_r <= r2 <= outer_r*outer_r and x <= cx_curve:
                img[y, x] = 255

    # 顶部水平段, 从弯道口向左延伸到 x=20
    bend_top_y = 30
    bend_top_x = cx - 45 - track_w // 2 - (45 - track_w // 2)
    for x in range(max(18, bend_top_x - 40), bend_top_x + track_w // 2 + 1):
        for y in range(bend_top_y, bend_top_y + track_w + 1):
            if y < H and 0 <= x < W:
                img[y, x] = 255
    return img


# =========================================================
# 2. 扫描找起点 与 爬线阶段 (纯模拟)
# =========================================================
def simulate_scan_left(binary, start_y, from_x):
    steps = []
    x = from_x
    steps.append({"phase": "scan_left", "x": x, "y": start_y, "status": "出发",
                  "action": f"从中央 ({from_x},{start_y}) 向左扫", "hit": False})
    while x > 1:
        left_px = int(binary[start_y, x - 1])
        if left_px < EDGE_WHITE_THRESHOLD:
            steps.append({"phase": "scan_left", "x": x, "y": start_y, "status": "命中",
                          "action": f"发现跳变: left={left_px}(黑)  cur={int(binary[start_y,x])}(白)\n✅ 左手起点确定 ({x},{start_y})",
                          "hit": True})
            return steps
        x -= 1
        steps.append({"phase": "scan_left", "x": x, "y": start_y, "status": "扫描中",
                      "action": f"左移1列 → ({x},{start_y})  left={int(binary[start_y,x-1])}(白)",
                      "hit": False})
    steps.append({"phase": "scan_left", "x": x, "y": start_y, "status": "失败",
                  "action": "扫到边界未找到", "hit": False})
    return steps


def simulate_scan_right(binary, start_y, from_x):
    steps = []
    x = from_x
    steps.append({"phase": "scan_right", "x": x, "y": start_y, "status": "出发",
                  "action": f"从中央 ({from_x},{start_y}) 向右扫", "hit": False})
    while x < W - 2:
        right_px = int(binary[start_y, x + 1])
        if right_px < EDGE_WHITE_THRESHOLD:
            steps.append({"phase": "scan_right", "x": x, "y": start_y, "status": "命中",
                          "action": f"发现跳变: right={right_px}(黑)  cur={int(binary[start_y,x])}(白)\n✅ 右手起点确定 ({x},{start_y})",
                          "hit": True})
            return steps
        x += 1
        steps.append({"phase": "scan_right", "x": x, "y": start_y, "status": "扫描中",
                      "action": f"右移1列 → ({x},{start_y})  right={int(binary[start_y,x+1])}(白)",
                      "hit": False})
    steps.append({"phase": "scan_right", "x": x, "y": start_y, "status": "失败",
                  "action": "扫到边界未找到", "hit": False})
    return steps


def simulate_lefthand(binary, start_x, start_y, max_points=2000):
    steps = []
    if (start_x <= 0) or (start_x >= W - 1) or (start_y <= 0) or (start_y >= H - 1):
        steps.append({"reason": "起点越界", "x": start_x, "y": start_y})
        return steps
    if binary[start_y, start_x] < EDGE_WHITE_THRESHOLD:
        steps.append({"reason": "起点为黑", "x": start_x, "y": start_y})
        return steps

    x, y = start_x, start_y
    dir_ = 0
    turn = 0
    step = 0

    steps.append({
        "phase": "line_left", "x": x, "y": y, "dir": dir_, "turn": turn, "step": step,
        "action": "爬线开始 START (方向朝上 摸左侧黑墙)",
        "fx": None, "fy": None, "fv": None, "sx": None, "sy": None, "sv": None,
        "saved": True,
    })

    while (step < max_points) and (x > 0) and (x < W - 1) and (y > 0) and (y < H - 1) and (turn < 4):
        front_x = x + edge_dir_front[dir_][0]
        front_y = y + edge_dir_front[dir_][1]
        sl_x = x + edge_dir_frontleft[dir_][0]
        sl_y = y + edge_dir_frontleft[dir_][1]
        front_v = int(binary[front_y, front_x])
        side_v = int(binary[sl_y, sl_x])

        if front_v < EDGE_WHITE_THRESHOLD:
            dir_ = (dir_ + 1) % 4
            turn += 1
            action = "① 前方黑 → 右转(撞墙)  ↑turn"
        elif side_v < EDGE_WHITE_THRESHOLD:
            x += edge_dir_front[dir_][0]
            y += edge_dir_front[dir_][1]
            step += 1
            turn = 0
            action = "② 前白+左前黑 → 直走(贴墙)  ↑step"
        else:
            x += edge_dir_frontleft[dir_][0]
            y += edge_dir_frontleft[dir_][1]
            dir_ = (dir_ + 3) % 4
            step += 1
            turn = 0
            action = "③ 都白 → 斜前走+左转(缺墙补位)  ↑step"

        steps.append({
            "phase": "line_left",
            "x": x, "y": y, "dir": dir_, "turn": turn, "step": step,
            "action": action,
            "fx": front_x, "fy": front_y, "fv": front_v,
            "sx": sl_x, "sy": sl_y, "sv": side_v,
            "saved": ("直走" in action or "斜前" in action),
        })

    reason = ""
    if step >= max_points: reason = f"达到 max_points={max_points}"
    elif not (x > 0 and x < W - 1 and y > 0 and y < H - 1): reason = f"越界 x={x},y={y}"
    elif turn >= 4: reason = f"原地转满一圈 turn={turn}≥4 —— 卡死保护"
    steps.append({
        "phase": "line_left", "x": x, "y": y, "dir": dir_, "turn": turn, "step": step,
        "action": f"🔚 结束 EXIT: {reason}",
    })
    return steps


def simulate_righthand(binary, start_x, start_y, max_points=2000):
    steps = []
    if (start_x <= 0) or (start_x >= W - 1) or (start_y <= 0) or (start_y >= H - 1):
        steps.append({"reason": "起点越界", "x": start_x, "y": start_y})
        return steps
    if binary[start_y, start_x] < EDGE_WHITE_THRESHOLD:
        steps.append({"reason": "起点为黑", "x": start_x, "y": start_y})
        return steps

    x, y = start_x, start_y
    dir_ = 0
    turn = 0
    step = 0

    steps.append({
        "phase": "line_right", "x": x, "y": y, "dir": dir_, "turn": turn, "step": step,
        "action": "爬线开始 START (方向朝上 摸右侧黑墙)",
        "saved": True,
    })

    while (step < max_points) and (x > 0) and (x < W - 1) and (y > 0) and (y < H - 1) and (turn < 4):
        front_x = x + edge_dir_front[dir_][0]
        front_y = y + edge_dir_front[dir_][1]
        sr_x = x + edge_dir_frontright[dir_][0]
        sr_y = y + edge_dir_frontright[dir_][1]
        front_v = int(binary[front_y, front_x])
        side_v = int(binary[sr_y, sr_x])

        if front_v < EDGE_WHITE_THRESHOLD:
            dir_ = (dir_ + 3) % 4
            turn += 1
            action = "① 前方黑 → 左转(撞墙)  ↑turn"
        elif side_v < EDGE_WHITE_THRESHOLD:
            x += edge_dir_front[dir_][0]
            y += edge_dir_front[dir_][1]
            step += 1
            turn = 0
            action = "② 前白+右前黑 → 直走(贴墙)  ↑step"
        else:
            x += edge_dir_frontright[dir_][0]
            y += edge_dir_frontright[dir_][1]
            dir_ = (dir_ + 1) % 4
            step += 1
            turn = 0
            action = "③ 都白 → 斜前走+右转(缺墙补位)  ↑step"

        steps.append({
            "phase": "line_right",
            "x": x, "y": y, "dir": dir_, "turn": turn, "step": step,
            "action": action,
            "fx": front_x, "fy": front_y, "fv": front_v,
            "sx": sr_x, "sy": sr_y, "sv": side_v,
            "saved": ("直走" in action or "斜前" in action),
        })

    reason = ""
    if step >= max_points: reason = f"达到 max_points={max_points}"
    elif not (x > 0 and x < W - 1 and y > 0 and y < H - 1): reason = f"越界 x={x},y={y}"
    elif turn >= 4: reason = f"原地转满一圈 turn={turn}≥4 —— 卡死保护"
    steps.append({
        "phase": "line_right", "x": x, "y": y, "dir": dir_, "turn": turn, "step": step,
        "action": f"🔚 结束 EXIT: {reason}",
    })
    return steps


# =========================================================
# 3. 动画渲染
# =========================================================
def draw_arrow(ax, x, y, dir_):
    L = 5
    dx, dy = edge_dir_front[dir_]
    dx *= L; dy *= L
    return ax.annotate(
        "", xy=(x + dx, y + dy), xytext=(x, y),
        arrowprops=dict(arrowstyle="->", lw=2.2, color="#ff1744"),
        zorder=10,
    )


def render_fig_to_rgb(fig):
    """把 fig 渲染成 numpy RGB 数组 (BytesIO + PIL, 跨 backend)"""
    from PIL import Image
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=fig.dpi,
                facecolor=fig.get_facecolor(), edgecolor="none",
                bbox_inches="tight", pad_inches=0)
    buf.seek(0)
    im = Image.open(buf).convert("RGB")
    arr = np.array(im)
    buf.close()
    return arr


def run_animation(args):
    binary = make_track()

    # --- 扫描找起点 ---
    start_y = H - 10
    cx = W // 2
    from_x_left = cx - 5
    from_x_right = cx + 5
    scan_left_steps  = simulate_scan_left(binary, start_y, from_x_left)
    scan_right_steps = simulate_scan_right(binary, start_y, from_x_right)
    start_x0 = scan_left_steps[-1]["x"]
    start_x1 = scan_right_steps[-1]["x"]
    if not scan_left_steps[-1].get("hit"):  start_x0 = from_x_left
    if not scan_right_steps[-1].get("hit"): start_x1 = from_x_right
    print(f"[扫描] 左手起点 ({start_x0},{start_y})  右手起点 ({start_x1},{start_y})")

    left_steps  = simulate_lefthand(binary, start_x0, start_y)
    right_steps = simulate_righthand(binary, start_x1, start_y)
    print(f"[爬线] 左手 {len(left_steps)} 步, 轨迹点 {sum(1 for s in left_steps if s.get('saved'))}")
    print(f"[爬线] 右手 {len(right_steps)} 步, 轨迹点 {sum(1 for s in right_steps if s.get('saved'))}")

    # --- 阶段拼接成全局帧 ---
    phase1_N = len(scan_left_steps)
    phase2_N = len(scan_right_steps)
    phase3_N = max(len(left_steps), len(right_steps))
    N = phase1_N + phase2_N + phase3_N

    def build_whole(scan_steps, scan_phase_name, line_steps):
        out = []
        if scan_phase_name == "scan_left":
            out += list(scan_steps)
            out += [scan_steps[-1]] * phase2_N
        else:
            out += [{"phase": "idle", "x": None, "y": None, "action": "[等待左手扫描]"}] * phase1_N
            out += list(scan_steps)
        ls = list(line_steps) + [line_steps[-1]] * (phase3_N - len(line_steps))
        out += ls
        if len(out) < N:
            out += [out[-1]] * (N - len(out))
        return out[:N]

    whole_left  = build_whole(scan_left_steps,  "scan_left",  left_steps)
    whole_right = build_whole(scan_right_steps, "scan_right", right_steps)

    # --- 画布: 左右两列 (去掉右侧代码面板) + 全新配色 ---
    fig = plt.figure(figsize=(20, 10), dpi=92)
    fig.patch.set_facecolor("#dbe3ec")   # 画布整体柔和蓝灰背景
    gs = fig.add_gridspec(1, 2, width_ratios=[1, 1], wspace=0.08,
                          left=0.035, right=0.965, top=0.905, bottom=0.06)
    ax_left  = fig.add_subplot(gs[0, 0])
    ax_right = fig.add_subplot(gs[0, 1])

    fig.suptitle(
        "巡线算法全流程演示  |  阶段①: 左手扫黑白跳变点 → 阶段②: 右手扫黑白跳变点 → 阶段③: 贴墙爬线   |   188×120  MT9V03X",
        fontsize=18, fontweight="bold", y=0.980, color="#0d1b3e",
    )

    titles = ["左手法则 (贴左侧黑墙走)", "右手法则 (贴右侧黑墙走)"]
    track_colors = [(252, 252, 252), (224, 240, 255)]
    ground_color = [19, 28, 54]
    for i, ax in enumerate([ax_left, ax_right]):
        disp = np.zeros((H, W, 3), dtype=np.uint8)
        disp[binary == 0] = ground_color
        disp[binary == 255] = track_colors[i]
        ax.imshow(disp, origin="upper")
        ax.set_title(titles[i], fontsize=14, fontweight="bold",
                     color="#0d3b7a", pad=9)
        ax.set_facecolor("#cfd8e3")
        ax.set_xlim(-2.2, W + 1.2)
        ax.set_ylim(H + 3.5, -3.5)
        ax.set_xticks([0, 47, 94, 141, 187])
        ax.set_yticks([0, 30, 60, 90, 119])
        ax.tick_params(labelsize=8.2, colors="#333", length=2.5, width=1.1)
        ax.grid(alpha=0.22, color="#ffffff", linewidth=0.7)
        # 子图边框更明显
        for sp in ax.spines.values():
            sp.set_linewidth(1.6)
            sp.set_color("#283593")

    # 起始点(菱形黄标)
    mark_left, = ax_left.plot([], [], "D", ms=11.5,
                              markerfacecolor="#ffb300", markeredgecolor="black",
                              markeredgewidth=1.3, zorder=9, label="起点")
    mark_right, = ax_right.plot([], [], "D", ms=11.5,
                                markerfacecolor="#ffb300", markeredgecolor="black",
                                markeredgewidth=1.3, zorder=9, label="起点")

    # 轨迹线
    line_left, = ax_left.plot([], [], color="#00c853", lw=2.6, alpha=0.95, zorder=4, label="轨迹")
    line_right, = ax_right.plot([], [], color="#2979ff", lw=2.6, alpha=0.95, zorder=4, label="轨迹")

    # 当前点红圈
    dot_left, = ax_left.plot([], [], "o", ms=13, markerfacecolor="#ff1744",
                             markeredgecolor="black", markeredgewidth=1.4, zorder=8)
    dot_right, = ax_right.plot([], [], "o", ms=13, markerfacecolor="#ff1744",
                               markeredgecolor="black", markeredgewidth=1.4, zorder=8)
    arrow_left = [None]
    arrow_right = [None]

    # 扫描线
    scan_vline_left = [None]
    scan_vline_right = [None]
    scan_hline_left = [None]
    scan_hline_right = [None]

    # 信息文本框 (美化)
    info_box = dict(boxstyle="round,pad=0.4", fc="#ffffffef", ec="#283593",
                    alpha=0.96, linewidth=1.3)
    info_left = ax_left.text(
        0.025, 0.03, "", transform=ax_left.transAxes,
        va="bottom", ha="left", fontsize=10.3, color="#102040", fontweight="semibold",
        bbox=info_box,
    )
    info_right = ax_right.text(
        0.025, 0.03, "", transform=ax_right.transAxes,
        va="bottom", ha="left", fontsize=10.3, color="#102040", fontweight="semibold",
        bbox=info_box,
    )
    ax_left.legend(loc="upper right", fontsize=9.5, framealpha=0.96, edgecolor="#283593")
    ax_right.legend(loc="upper right", fontsize=9.5, framealpha=0.96, edgecolor="#283593")

    # 轨迹累积
    trace_lx, trace_ly = [], []
    trace_rx, trace_ry = [], []
    left_start_xy = [None, None]
    right_start_xy = [None, None]

    def fmt_scan_info(s, ph):
        if s.get("phase") == "idle":
            return f"[{ph}]\n等待另一手扫描…"
        if "reason" in s:
            return f"⚠ {s['reason']}"
        x, y = s.get("x"), s.get("y")
        extra = s.get("action", "")
        status = s.get("status", "")
        return f"[{ph}]  {status}  针({x},{y})\n{extra}"

    def fmt_line_info(s):
        if "reason" in s:
            return f"⚠ {s['reason']}"
        fx = s.get("fx")
        probe = ""
        if fx is not None:
            probe = f"\n探: 前({s['fx']},{s['fy']})={s['fv']}  侧前({s['sx']},{s['sy']})={s['sv']}"
        return (
            f"step={s.get('step','-')}  turn={s.get('turn','-')}\n"
            f"dir={DIR_NAME[s.get('dir', 0)]}  pos=({s.get('x','')},{s.get('y','')})\n"
            f"{s.get('action','')}{probe}"
        )

    def sub_phase(i):
        if i < phase1_N: return "阶段①: 扫描左手起点 (向左找黑白跳变)"
        if i < phase1_N + phase2_N: return "阶段②: 扫描右手起点 (向右找黑白跳变)"
        return "阶段③: 贴墙爬线 (左右手同步)"

    def frame(i):
        nonlocal trace_lx, trace_ly, trace_rx, trace_ry

        sl = whole_left[i]
        sr = whole_right[i]
        ph = sub_phase(i)

        # ========================= 左手子图 =========================
        if sl.get("phase") == "scan_left":
            x, y = sl.get("x"), sl.get("y")
            dot_left.set_data([x], [y])
            if scan_vline_left[0] is not None: scan_vline_left[0].remove()
            scan_vline_left[0] = ax_left.axvline(x, color="#00e676", lw=2.8,
                                                 alpha=0.9, zorder=5)
            if scan_hline_left[0] is not None: scan_hline_left[0].remove()
            scan_hline_left[0] = ax_left.axhline(start_y, color="#ffea00",
                                                 lw=2.2, alpha=0.78, zorder=5, linestyle="--")
            if sl.get("hit"):
                left_start_xy[0] = x; left_start_xy[1] = y
            if left_start_xy[0] is not None:
                mark_left.set_data([left_start_xy[0]], [left_start_xy[1]])
            else:
                mark_left.set_data([], [])
            info_left.set_text(f"帧{i+1}/{N}\n{fmt_scan_info(sl, ph)}")
            if arrow_left[0] is not None: arrow_left[0].remove(); arrow_left[0] = None
        elif sl.get("phase") == "idle":
            dot_left.set_data([], [])
            mark_left.set_data([], [])
            line_left.set_data([], [])
            trace_lx.clear(); trace_ly.clear()
            if arrow_left[0] is not None: arrow_left[0].remove(); arrow_left[0] = None
            if scan_vline_left[0] is not None: scan_vline_left[0].remove(); scan_vline_left[0] = None
            if scan_hline_left[0] is not None: scan_hline_left[0].remove(); scan_hline_left[0] = None
            info_left.set_text(f"帧{i+1}/{N}\n{fmt_scan_info(sl, ph)}")
        else:
            if scan_vline_left[0] is not None: scan_vline_left[0].remove(); scan_vline_left[0] = None
            if scan_hline_left[0] is not None: scan_hline_left[0].remove(); scan_hline_left[0] = None
            x, y = sl.get("x"), sl.get("y")
            dot_left.set_data([x], [y])
            if sl.get("saved") and (not trace_lx or (trace_lx[-1], trace_ly[-1]) != (x, y)):
                trace_lx.append(x); trace_ly.append(y)
            line_left.set_data(trace_lx, trace_ly)
            if left_start_xy[0] is not None:
                mark_left.set_data([left_start_xy[0]], [left_start_xy[1]])
            if arrow_left[0] is not None: arrow_left[0].remove()
            if "dir" in sl and x is not None:
                arrow_left[0] = draw_arrow(ax_left, x, y, sl["dir"])
            info_left.set_text(f"帧{i+1}/{N}\n{fmt_line_info(sl)}")

        # ========================= 右手子图 =========================
        if sr.get("phase") == "scan_right":
            x, y = sr.get("x"), sr.get("y")
            dot_right.set_data([x], [y])
            if scan_vline_right[0] is not None: scan_vline_right[0].remove()
            scan_vline_right[0] = ax_right.axvline(x, color="#00e676", lw=2.8,
                                                   alpha=0.9, zorder=5)
            if scan_hline_right[0] is not None: scan_hline_right[0].remove()
            scan_hline_right[0] = ax_right.axhline(start_y, color="#ffea00",
                                                   lw=2.2, alpha=0.78, zorder=5, linestyle="--")
            if sr.get("hit"):
                right_start_xy[0] = x; right_start_xy[1] = y
            if right_start_xy[0] is not None:
                mark_right.set_data([right_start_xy[0]], [right_start_xy[1]])
            else:
                mark_right.set_data([], [])
            info_right.set_text(f"帧{i+1}/{N}\n{fmt_scan_info(sr, ph)}")
            if arrow_right[0] is not None: arrow_right[0].remove(); arrow_right[0] = None
        elif sr.get("phase") == "idle":
            dot_right.set_data([], [])
            mark_right.set_data([], [])
            line_right.set_data([], [])
            trace_rx.clear(); trace_ry.clear()
            if arrow_right[0] is not None: arrow_right[0].remove(); arrow_right[0] = None
            if scan_vline_right[0] is not None: scan_vline_right[0].remove(); scan_vline_right[0] = None
            if scan_hline_right[0] is not None: scan_hline_right[0].remove(); scan_hline_right[0] = None
            info_right.set_text(f"帧{i+1}/{N}\n{fmt_scan_info(sr, ph)}")
        else:
            if scan_vline_right[0] is not None: scan_vline_right[0].remove(); scan_vline_right[0] = None
            if scan_hline_right[0] is not None: scan_hline_right[0].remove(); scan_hline_right[0] = None
            x, y = sr.get("x"), sr.get("y")
            dot_right.set_data([x], [y])
            if sr.get("saved") and (not trace_rx or (trace_rx[-1], trace_ry[-1]) != (x, y)):
                trace_rx.append(x); trace_ry.append(y)
            line_right.set_data(trace_rx, trace_ry)
            if right_start_xy[0] is not None:
                mark_right.set_data([right_start_xy[0]], [right_start_xy[1]])
            if arrow_right[0] is not None: arrow_right[0].remove()
            if "dir" in sr and x is not None:
                arrow_right[0] = draw_arrow(ax_right, x, y, sr["dir"])
            info_right.set_text(f"帧{i+1}/{N}\n{fmt_line_info(sr)}")

        # (已去掉代码面板)

        return (dot_left, mark_left, line_left, info_left,
                dot_right, mark_right, line_right, info_right)

    anim = FuncAnimation(
        fig, frame, frames=N, interval=args.interval,
        blit=False, repeat=args.repeat,
    )

    out_dir = r"D:\AI OUT"
    os.makedirs(out_dir, exist_ok=True)

    if args.save:
        # ---- 1. GIF ----
        out_path = os.path.join(out_dir, "findline_demo.gif")
        print(f"[GIF] 写入 {out_path}  共 {N} 帧 ...")
        writer = PillowWriter(fps=1000 // args.interval, metadata=dict(artist="TC264-demo"))
        anim.save(out_path, writer=writer, dpi=92)
        print(f"[GIF] 完成 → {out_path}")

        # ---- 2. MP4 (如果环境有 cv2/ffmpeg 就生成, 没有就跳过) ----
        if args.mp4:
            mp4_path = os.path.join(out_dir, "findline_demo.mp4")
            fps = 1000 // args.interval
            try:
                import cv2
                print(f"[MP4] cv2 方式 → {mp4_path}")
                frame(0)
                rgb = render_fig_to_rgb(fig)
                h, w = rgb.shape[:2]
                vw = cv2.VideoWriter(mp4_path, cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (w, h))
                if not vw.isOpened():
                    vw = cv2.VideoWriter(mp4_path, cv2.VideoWriter_fourcc(*"avc1"), float(fps), (w, h))
                if not vw.isOpened():
                    raise RuntimeError("VideoWriter 无法打开")
                trace_lx.clear(); trace_ly.clear(); trace_rx.clear(); trace_ry.clear()
                left_start_xy[0] = None; left_start_xy[1] = None
                right_start_xy[0] = None; right_start_xy[1] = None
                for a in [scan_vline_left, scan_vline_right, scan_hline_left, scan_hline_right]:
                    if a[0] is not None: a[0].remove(); a[0] = None
                for j in range(N):
                    frame(j)
                    rgb = render_fig_to_rgb(fig)
                    vw.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                vw.release()
                print(f"[MP4] 完成 → {mp4_path}")
            except Exception as e:
                print(f"[MP4] cv2 失败: {e}")
                try:
                    from matplotlib.animation import FFMpegWriter
                    anim.save(mp4_path, writer=FFMpegWriter(fps=fps), dpi=92)
                    print(f"[MP4] 完成 → {mp4_path}")
                except Exception as e2:
                    print(f"[MP4] ffmpeg 也失败: {e2} (不影响 GIF/JPG)")

        # ---- 3. 关键帧拼图 JPG (20帧 5×4) ----
        if args.poster:
            poster_path = os.path.join(out_dir, "findline_steps.jpg")
            final_path  = os.path.join(out_dir, "findline_final.jpg")
            try:
                picks = list(np.linspace(0, N - 1, 20, dtype=int))
                cols, rows = 5, 4
                figp, axp = plt.subplots(rows, cols, figsize=(25, 16), dpi=100)
                figp.suptitle(
                    "巡线全流程 20 关键帧 (188×120)\n"
                    "①扫左手起点 → ②扫右手起点 → ③贴墙爬线    黄菱形=起点  红点=当前  绿/蓝=轨迹",
                    fontsize=17, fontweight="bold", y=0.996, color="#0d1b3e",
                )
                figp.patch.set_facecolor("#dbe3ec")

                trace_lx.clear(); trace_ly.clear(); trace_rx.clear(); trace_ry.clear()
                left_start_xy[0] = None; left_start_xy[1] = None
                right_start_xy[0] = None; right_start_xy[1] = None
                for a in [scan_vline_left, scan_vline_right, scan_hline_left, scan_hline_right]:
                    if a[0] is not None: a[0].remove(); a[0] = None

                last_j = -1
                for idx, k in enumerate(picks):
                    for j in range(last_j + 1, k + 1):
                        frame(j)
                    last_j = k
                    rgb = render_fig_to_rgb(fig)
                    sub = axp[idx // cols][idx % cols]
                    sub.imshow(rgb)
                    ph_now = sub_phase(k)
                    sub.set_title(f"帧{k+1}/{N}   {ph_now}", fontsize=9.5,
                                  fontweight="bold", color="#1a237e")
                    sub.set_xticks([]); sub.set_yticks([])
                    for sp in sub.spines.values():
                        sp.set_linewidth(1.3); sp.set_color("#283593")
                figp.tight_layout(rect=[0, 0, 1, 0.965])
                try:
                    figp.savefig(poster_path, format="jpg",
                                 pil_kwargs={"quality": 93}, bbox_inches="tight",
                                 facecolor=figp.get_facecolor())
                except TypeError:
                    figp.savefig(poster_path, format="jpg", quality=93,
                                 bbox_inches="tight", facecolor=figp.get_facecolor())
                plt.close(figp)
                print(f"[拼图JPG] 完成 → {poster_path}  (20帧 5×4)")

                # 完整终图 JPG
                for j in range(last_j + 1, N):
                    frame(j)
                rgb = render_fig_to_rgb(fig)
                try:
                    import cv2 as _cv2
                    _cv2.imwrite(final_path, _cv2.cvtColor(rgb, _cv2.COLOR_RGB2BGR),
                                [int(_cv2.IMWRITE_JPEG_QUALITY), 96])
                except Exception:
                    from PIL import Image as _PILImage
                    _PILImage.fromarray(rgb).save(final_path, quality=96)
                print(f"[完整终图JPG] 完成 → {final_path}")
            except Exception as e:
                print(f"[拼图JPG] 失败: {e}")

    if args.show:
        print("[显示动画] 关闭GUI窗口后程序退出")
        plt.show()
    else:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="左右手巡线算法动画 - IDE代码面板+红底高亮行+扫起点")
    parser.add_argument("--nosave", dest="save", action="store_false",
                        help="不保存任何文件 (默认保存 GIF+MP4(若可用)+JPG拼图)")
    parser.add_argument("--noshow", dest="show", action="store_false",
                        help="不弹GUI窗口直接生成文件 (默认弹窗)")
    parser.add_argument("--interval", type=int, default=110,
                        help="每帧间隔毫秒 (默认110ms ≈ 9fps)")
    parser.add_argument("--norepeat", dest="repeat", action="store_false",
                        help="动画只播一次, 不循环 (默认循环)")
    parser.add_argument("--nomp4", dest="mp4", action="store_false",
                        help="不尝试生成 MP4 (默认尝试)")
    parser.add_argument("--noposter", dest="poster", action="store_false",
                        help="不生成 20 帧关键帧 JPG 拼图 (默认生成)")
    args = parser.parse_args()

    print("=" * 70)
    print("  左右手巡线算法 全流程动画 (IDE风格代码面板 + 红底高亮当前行)")
    print("  图像 MT9V03X: W=188, H=120")
    print("=" * 70)
    run_animation(args)
    print("Done.")


if __name__ == "__main__":
    main()
