#include "image.h"
#include "camera_param.h"
#include "servo.h"
#include "beep.h"
#include "IfxStm.h"
#define IMAGE_OTSU_BLOCK_W      (188)  // 每个局部区域的宽度
#define IMAGE_OTSU_BLOCK_H      (20)  // 每个局部区域的高度

#define IMAGE_OTSU_BLOCK_COLS \
    ((MT9V03X_W + IMAGE_OTSU_BLOCK_W - 1) / IMAGE_OTSU_BLOCK_W) // 横向区域数量

#define IMAGE_OTSU_BLOCK_ROWS \
    ((MT9V03X_H + IMAGE_OTSU_BLOCK_H - 1) / IMAGE_OTSU_BLOCK_H) // 纵向区域数量

//#define IPTS_MAX (200) //边线点的数组容量
uint8 image_threshold_map[IMAGE_OTSU_BLOCK_ROWS][IMAGE_OTSU_BLOCK_COLS];
// 保存每个局部区域计算出来的阈值

uint8 image_binary[MT9V03X_H][MT9V03X_W];
int16 left_line_points[IPTS_MAX][2];
int16 right_line_points[IPTS_MAX][2];
uint16 left_line_count = 0;
uint16 right_line_count = 0;
int16 lookahead_lx = -1, lookahead_rx = -1, lookahead_y = 0;   // 前瞻行左右边界点(原图像素,-1=无)

/* ================ 鸟瞰图（逆透视） ================ */
uint8 img_pers_data[PERS_H][PERS_W];                    // 鸟瞰灰度图
image_t img_pers = { (uint8 *)img_pers_data, PERS_W, PERS_H, PERS_W };
int16 touch_boundary0 = 0;      // 左巡线碰到图像边界
int16 touch_boundary1 = 0;      // 右巡线碰到图像边界
int16 maze_start_y = 0;      // 迷宫法实际起始行（find_binary_start 定位，显示调试用）
int16 maze_start_left_x = -1;
int16 maze_start_right_x = -1;
// 二值图中白色像素的判断阈值。
#define BINARY_START_MIN_WIDTH (8)
#define BINARY_START_MAX_WIDTH (PERS_W - 8)

// 当前方向的正前方坐标变化表。
// 0表示向上，1表示向右，2表示向下，3表示向左。
static const int16 edge_dir_front[4][2] =
{
    { 0, -1},
    { 1,  0},
    { 0,  1},
    {-1,  0}
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
// 保存最终二值化图像，0表示黑色，255表示白色
// 在有序边线折线上按指定 y 插值得到 x。返回的点严格位于边线段上，
// 两侧使用同一个 target_y 后取中点，得到的前瞻点就在真实双边中线上。
static uint8 sample_line_x_at_y(float pts[][2], int16 num, float target_y,
                                float *x, int16 *nearest_idx)
{
    int16 i;

    if (num < 2) return 0;
    for (i = 0; i < num - 1; i++)
    {
        float y0 = pts[i][1];
        float y1 = pts[i + 1][1];
        float dy = y1 - y0;

        if ((target_y - y0) * (target_y - y1) > 0.0f) continue;
        if (fabsf(dy) < 0.0001f) continue;

        {
            float t = (target_y - y0) / dy;
            *x = pts[i][0] + t * (pts[i + 1][0] - pts[i][0]);
            *nearest_idx = (t < 0.5f) ? i : (i + 1);
        }
        return 1;
    }
    return 0;
}

/**
 * @brief 对局部图像进行大律法处理
 * @param  输入图像，左边界（包含），右边界（不包含），上边界（包含），下边界（不包含）
 * @return 阈值
 * @attention 该函数仅对指定局部区域进行处理
 * @addtogroup  大津法寻找最佳阈值的逻辑：1.先统计当前区域的灰度直方图肌 代码：histogram[level]++;
 * 2.然后把每个灰度值当作候选阈值 
 * 3.把图像分成两类：低灰度<=100,高灰度>100 back_count += histogram[level]; back_sum += histogram[level] * level;
 * 4.根据当前level计算比例和平均灰度 然后计算类间方差 sigma_b=back_rate*fore_rate*(back_mean-fore_mean)*(back_mean-fore_mean);
 * 5.通过比较类间方差来找出最佳阈值
 */

static uint8 otsu_local_threshold(const uint8 image[MT9V03X_H][MT9V03X_W],uint16 x_left_yes, uint16 x_right_no,uint16 y_top_yes, uint16 y_bottom_no)
{
    uint32 histogram[256] = {0};
    uint16 x; //当前像素点的x坐标
    uint16 y; //当前像素点的y坐标
    uint16 level; //当前像素点的灰度等级
    uint16 min_value = 0; //灰度等级最小值
    uint16 max_value = 0; //灰度等级最大值

    uint32 pixel_count = 0; //像素点总数
    uint32 gray_sum = 0; //灰度总和

    uint32 back_count = 0; // 低灰度类别的像素数量
    uint32 back_sum = 0; // 低灰度类别的灰度总和

    float fore_rate = 0;
    float back_rate=0; // 低灰度类别的像素数量占总像素数量的比例
    float back_mean=0; // 低灰度类别的平均灰度
    float fore_mean = 0;
    float sigma_b=0; // 类间方差
    float max_sigma_b=-1.0f; // 最大类间方差

    uint8 result=128; // 最终计算得到的阈值

    //二维数组遍历局部区域，每隔一个像素采样一次，降低计算量

    for(y=y_top_yes;y<y_bottom_no;y+=2)
    {
        for(x=x_left_yes;x<x_right_no;x+=2)
        {
            level=image[y][x]; //读取当前灰度值

            histogram[level]++; //灰度值计数
            pixel_count++; //像素点总数计数
            gray_sum+=level; //灰度总和
        }
    }
    if(pixel_count==0) //如果像素点总数为0，说明局部区域没有采样到像素点，直接返回默认阈值128
    {
        return 128;
    }
    //从小到大查找区域中的最小灰度值
    for(level=0;level<256;level++)
    {
        if(histogram[level]!=0)
        {
            min_value=level;
            break;
            //结束最小灰度值搜索
        }
        /*
        PS:第一个二维数组是往里面计数，第二个遍历是取的坐标，level：是0-255灰度值，当对应的数组有次数说明在这个矩形内有这个灰度值，那么遍历到最小的就是最小灰度值
        */
    }
    max_value = min_value;
    for(level=255;level>min_value;level--)
    {
        if(histogram[level]!=0)
        {
            max_value=level;
            break;
        }
    }
    if(min_value==max_value)
    {
        if(min_value<128)
        {
            return (uint8)(min_value+1);
        }
        else
        {
            return (uint8)min_value;
        }
        //如果区域内只有一种灰度，直接返回该灰度值
    }

    //依次把每个灰度值作为候选灰度值
    for(level=min_value;level<max_value;level++)
    {
        back_count+=histogram[level];//把当前灰度列为低灰度类别

        back_sum += (uint32)histogram[level] * (uint32)level; //低灰度类别的灰度总和

        if(back_count==0||back_count>=pixel_count)
        {
            continue;
            //如果低灰度类别的像素数量为0或者大于等于总像素数量，说明当前候选灰度值不合适，跳过
        }
        //依次把每个灰度值作为候选灰度值
        fore_rate=(float)(pixel_count-back_count)/(float)pixel_count; //高灰度类别的像素数量占总像素数量的比例
        back_rate=(float)back_count/(float)pixel_count; //低灰度类别的像素数量占总像素数量的比例
        back_mean=(float)back_sum/(float)back_count; //低灰度类别的平均灰度
        fore_mean=(float)(gray_sum-back_sum)/(float)(pixel_count-back_count); //高灰度类别的平均灰度

        sigma_b=back_rate*fore_rate*(back_mean-fore_mean)*(back_mean-fore_mean); //计算类间方差
        if(sigma_b>max_sigma_b)
        {
            max_sigma_b=sigma_b; //更新最大类间方差
            result=(uint8)level; //更新最终计算得到的阈值
        }
    }
    //PS：最大类间方差
    if(result==0)
    {
        result=1;
    }
    return result; //返回最终计算得到的阈值
}
/**
 * 
* @brief 对整个图像进行分块局部大律法处理
* @param  输入图像
* @return void
* @addtogroup 先分块算阈值，然后把每块的阈值给对应的像素点
*/
void image_threshold_block(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint16 block_x;//当前局部区域的横向编号
    uint16 block_y;

    uint16 x;//当前图像像素的横坐标
    uint16 y;//当前像素的纵坐标

    uint16 x_left;//当前区域左边界
    uint16 x_right;//当前区域的右边界
    uint16 y_top;//当前区域的上边界
    uint16 y_bottle;//当前区域的下边界

    //遍历所有纵向局部区域

    for(block_y=0;block_y<IMAGE_OTSU_BLOCK_ROWS;block_y++)
    {
        y_top=block_y*IMAGE_OTSU_BLOCK_H;// // 计算当前区域的上边界
        y_bottle=y_top+IMAGE_OTSU_BLOCK_H;//// 计算当前区域的下边界

        if(y_bottle>MT9V03X_H)
        {
            y_bottle=MT9V03X_H;// 最后一行区域可能不足完整高度，需要限制在图像范围内
        }

        for(block_x=0;block_x<IMAGE_OTSU_BLOCK_COLS;block_x++)
        {
            x_left=block_x*IMAGE_OTSU_BLOCK_W;
            // 计算当前区域的左边界

            x_right=x_left+IMAGE_OTSU_BLOCK_W;
            // 计算当前区域的右边界

            if(x_right>MT9V03X_W)
            {
                x_right=MT9V03X_W;
                // 最后一列区域可能不足完整宽度，需要限制在图像范围内
            }

            image_threshold_map[block_y][block_x] =
                otsu_local_threshold(image,x_left,x_right,y_top,y_bottle);//计算这个区域的局部最大阈值
        }
    }
    //遍历整幅图像的每一个像素点
    for(y=0;y<MT9V03X_H;y++)
    {
        for(x=0;x<MT9V03X_W;x++)
        {
            block_x=x/IMAGE_OTSU_BLOCK_W;
            // 根据像素横坐标确定它属于哪个局部区域

            block_y=y/IMAGE_OTSU_BLOCK_H;
            // 根据像素纵坐标确定它属于哪个局部区域

            if(image[y][x]<image_threshold_map[block_y][block_x])
            {
                image_binary[y][x]=0;
            }
            else
            {
                image_binary[y][x]=255;
            }
        }
    }
}
/**
 * @author 春之雪  
* @brief 用这个函数来显示每块阈值
* @param  输入图像
* @return void
* @addtogroup 
*/
static void __attribute__((unused)) display_otsu_thresholds(void)
{
    uint16 block_x;       // 当前区域横向编号
    uint16 block_y;       // 当前区域纵向编号
    uint16 display_x;     // 阈值显示的横坐标
    uint16 display_y;     // 阈值显示的纵坐标

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);

    ips200_show_string(0, 184, "otsu:");

    for (block_y = 0; block_y < IMAGE_OTSU_BLOCK_ROWS; block_y++)
    {
        for (block_x = 0; block_x < IMAGE_OTSU_BLOCK_COLS; block_x++)
        {
            display_x = block_x * 40;
            display_y = 204 + block_y * 20;

            ips200_show_uint(display_x,display_y,image_threshold_map[block_y][block_x],3);
        }
    }
}
/**
 * @brief 从直方图计算大津阈值（核心计算）
 * @param histogram 直方图数组 [256]
 * @param pixel_count 总像素数
 * @return 最佳阈值（1~255）
 */
static uint8 otsu_compute(const uint32 histogram[256], uint32 pixel_count)
{
    int min_t = 0;
    int max_t = 255;
    int t;

    // 找有效灰度范围
    while (min_t < 256 && histogram[min_t] == 0) min_t++;
    while (max_t > min_t && histogram[max_t] == 0) max_t--;

    if (min_t >= max_t)
    {
        return (uint8)((min_t < 128) ? (min_t + 1) : min_t);
    }

    // 计算总加权和
    float sum_total = 0.0f;
    for (t = min_t; t <= max_t; t++)
    {
        sum_total += (float)t * histogram[t];
    }

    float sum_back = 0.0f;
    uint32 w_back = 0;
    float var_max = -1.0f;
    uint8 threshold = (uint8)min_t;

    for (t = min_t; t < max_t; t++)
    {
        w_back += histogram[t];
        sum_back += (float)t * histogram[t];

        uint32 w_fore = pixel_count - w_back;
        if (w_back == 0 || w_fore == 0) continue;

        // 类间方差 = wB * wF * (μB - μF)²
        // 用 sumB*N - sum*wB 避免除法（等价于 wB*wF*(μB-μF)²）
        float diff = sum_back * (float)pixel_count - sum_total * (float)w_back;
        float var = (diff * diff) / ((float)w_back * (float)w_fore);
        if (var > var_max)
        {
            var_max = var;
            threshold = (uint8)t;
        }
    }

    if (threshold == 0) threshold = 1;
    return threshold;
}

/**
 * @brief 双重迭代大津法二值化
 *        全图统一直方图 → 第一次大津 T1 → 直方图低位合并到 T1
 *        → 第二次大津 T2 → 用 T2 二值化
 * @param image 输入灰度图
 * @return void（结果写入 image_binary）
 */
void image_threshold(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint32 histogram[256] = {0};
    uint32 pixel_count = 0;
    uint16 x, y;
    uint16 level;
    uint8 threshold;
    uint32 low_sum;

    // === Step 1: 统计直方图（隔行采样加速，每 2 像素取 1） ===
    for (y = 0; y < MT9V03X_H; y += 2)
    {
        for (x = 0; x < MT9V03X_W; x += 2)
        {
            level = image[y][x];
            histogram[level]++;
            pixel_count++;
        }
    }
    if (pixel_count == 0) return;

    threshold = otsu_compute(histogram, pixel_count);

    low_sum = 0;
    for (level = 0; level < threshold; level++)
    {
        low_sum += histogram[level];
        histogram[level] = 0;
    }
    histogram[threshold] += low_sum;

    threshold = otsu_compute(histogram, pixel_count);
    for (y = 0; y < MT9V03X_H; y++)
    {
        for (x = 0; x < MT9V03X_W; x++)
        {
            image_binary[y][x] = (image[y][x] < threshold) ? 0 : 255;
        }
    }
}
/* ========================================================================
 *
 * 每个鸟瞰像素 (i,j) 查 invx/invy 得到原图坐标，直接拷贝灰度值
 * 查表越界的像素填黑（防上一帧残留）
 * 占位恒等表下鸟瞰图=原图；标定换真表后本函数一行不用改
 * ======================================================================== */
void anti_perspective_fast(void)
{
    int16 i;
    int16 j;
    int16 sx;
    int16 sy;

    for (i = 0; i < PERS_W; i++)
    {
        for (j = 0; j < PERS_H; j++)
        {
            sx = invx[j][i];
            sy = invy[j][i];
            if (sx >= 0 && sy >= 0 && sy < MT9V03X_H && sx < MT9V03X_W)
            {
                /* 鸟瞰调试图直接保存二值结果，避免再分配一块 22 KB 缓冲。 */
                img_pers_data[j][i] = image_binary[sy][sx];
            }
            else
            {
                img_pers_data[j][i] = 0;
            }
        }
    }
}
void findline_lefthand_binary(const uint8 binary[PERS_H][PERS_W], int16 x, int16 y, int16 points[][2], uint16 *point_count)
{
    uint16 max_points = *point_count;
    uint16 step = 0;
    int16 dir = 0;
    int16 turn = 0;

    *point_count = 0;
    if (max_points == 0 || x <= 0 || x >= PERS_W - 1 || y <= 0 || y >= PERS_H - 1 ||
        binary[y][x] < EDGE_WHITE_THRESHOLD)
    {
        return;
    }

    points[0][0] = x;
    points[0][1] = y;
    while (step < max_points - 1 && x > 0 && y > 0 && x < PERS_W - 1 && y < PERS_H - 1 && turn < 4)
    {
        int16 fx = x + edge_dir_front[dir][0];
        int16 fy = y + edge_dir_front[dir][1];
        int16 flx = x + edge_dir_frontleft[dir][0];
        int16 fly = y + edge_dir_frontleft[dir][1];

        if ((x == 1 && y < PERS_H - 20) || x == PERS_W - 2 || y == 1)
        {
            touch_boundary0 = 1;
            break;
        }

        if (binary[fy][fx] >= EDGE_WHITE_THRESHOLD)
        {
            if (binary[fly][flx] >= EDGE_WHITE_THRESHOLD)
            {
                dir = (dir + 3) % 4;
                x = flx;
                y = fly;
            }
            else
            {
                x = fx;
                y = fy;
            }
            turn = 0;
            step++;
            points[step][0] = x;
            points[step][1] = y;
        }
        else
        {
            dir = (dir + 1) % 4;
            turn++;
        }
    }
    *point_count = step + 1;
}

void findline_righthand_binary(const uint8 binary[PERS_H][PERS_W], int16 x, int16 y, int16 points[][2], uint16 *point_count)
{
    uint16 max_points = *point_count;
    uint16 step = 0;
    int16 dir = 0;
    int16 turn = 0;

    *point_count = 0;
    if (max_points == 0 || x <= 0 || x >= PERS_W - 1 || y <= 0 || y >= PERS_H - 1 ||
        binary[y][x] < EDGE_WHITE_THRESHOLD)
    {
        return;
    }

    points[0][0] = x;
    points[0][1] = y;
    while (step < max_points - 1 && x > 0 && y > 0 && x < PERS_W - 1 && y < PERS_H - 1 && turn < 4)
    {
        int16 fx = x + edge_dir_front[dir][0];
        int16 fy = y + edge_dir_front[dir][1];
        int16 frx = x + edge_dir_frontright[dir][0];
        int16 fry = y + edge_dir_frontright[dir][1];

        if ((x == PERS_W - 2 && y < PERS_H - 20) || x == 1 || y == 1)
        {
            touch_boundary1 = 1;
            break;
        }

        if (binary[fy][fx] >= EDGE_WHITE_THRESHOLD)
        {
            if (binary[fry][frx] >= EDGE_WHITE_THRESHOLD)
            {
                dir = (dir + 1) % 4;
                x = frx;
                y = fry;
            }
            else
            {
                x = fx;
                y = fy;
            }
            turn = 0;
            step++;
            points[step][0] = x;
            points[step][1] = y;
        }
        else
        {
            dir = (dir + 3) % 4;
            turn++;
        }
    }
    *point_count = step + 1;
}

// 左右迷宫法独立运行，某一侧可能在远端绕到另一侧轮廓。
// 从起始行向前逐行配对；一旦左右顺序反转或间距几乎为零，截断该行后的两条边线。
static void truncate_crossed_edges(int16 start_y)
{
    int16 y;
    int16 bad_y = -1;
    uint16 i;

    for (y = start_y; y >= 1; y--)
    {
        int16 lx = -1;
        int16 rx = -1;

        for (i = 0; i < left_line_count; i++)
        {
            if (left_line_points[i][1] == y)
            {
                lx = left_line_points[i][0];
                break;
            }
        }
        for (i = 0; i < right_line_count; i++)
        {
            if (right_line_points[i][1] == y)
            {
                rx = right_line_points[i][0];
                break;
            }
        }

        if (lx >= 0 && rx >= 0 && rx - lx <= 3)
        {
            bad_y = y;
            break;
        }
    }

    if (bad_y < 0) return;

    for (i = 1; i < left_line_count; i++)
    {
        if (left_line_points[i][1] <= bad_y)
        {
            left_line_count = i;
            break;
        }
    }
    for (i = 1; i < right_line_count; i++)
    {
        if (right_line_points[i][1] <= bad_y)
        {
            right_line_count = i;
            break;
        }
    }
}

static uint8 find_binary_start(const uint8 binary[PERS_H][PERS_W], int16 *y, int16 *left_x, int16 *right_x)
{
    int16 yy;
    static uint8 previous_ready = 0;
    static int16 previous_center = PERS_W / 2;
    static int16 previous_width = TRACK_HALF_W * 2;

    // 从图像底部向上搜索。每行枚举所有完整白色区间，选择中心最靠近
    // 图像中心的有效区间；区间两侧必须存在黑色像素，确保得到两个边界。
    for (yy = PERS_H - 2; yy >= 2; yy--)
    {
        int16 x = 1;
        int16 best_left = -1;
        int16 best_right = -1;
        int16 best_score = 32767;

        while (x < PERS_W - 1)
        {
            int16 run_left;
            int16 run_right;
            int16 width;
            int16 center;
            int16 score;

            while (x < PERS_W - 1 && binary[yy][x] < EDGE_WHITE_THRESHOLD) x++;
            if (x >= PERS_W - 1) break;
            run_left = x;
            while (x < PERS_W - 1 && binary[yy][x] >= EDGE_WHITE_THRESHOLD) x++;
            run_right = x - 1;
            width = run_right - run_left + 1;

            if (run_left > 1 && run_right < PERS_W - 2 &&
                width >= BINARY_START_MIN_WIDTH && width <= BINARY_START_MAX_WIDTH &&
                binary[yy][run_left - 1] < EDGE_WHITE_THRESHOLD &&
                binary[yy][run_right + 1] < EDGE_WHITE_THRESHOLD)
            {
                center = (run_left + run_right) / 2;
                score = previous_ready
                      ? abs(center - previous_center) * 3 + abs(width - previous_width)
                      : abs(center - PERS_W / 2);
                if (score < best_score)
                {
                    best_score = score;
                    best_left = run_left;
                    best_right = run_right;
                }
            }
        }

        if (best_left >= 0)
        {
            *y = yy;
            *left_x = best_left;
            *right_x = best_right;
            previous_center = (best_left + best_right) / 2;
            previous_width = best_right - best_left + 1;
            previous_ready = 1;
            return 1;
        }
    }
    return 0;
}

// 从起始赛道区间逐行向远端跟踪。每行只选择与上一行最连续的完整白色区间，
// 保证一行一对边界且点序只向前，避免迷宫法回头、闭环和左右串线。
static void track_lane_by_rows(const uint8 binary[PERS_H][PERS_W],
                               int16 start_y, int16 start_left, int16 start_right)
{
    int16 y;
    int16 previous_center = (start_left + start_right) / 2;
    int16 previous_width = start_right - start_left + 1;
    uint8 miss_count = 0;
    uint8 saw_touch_l = 0;   /* 巡线中出现贴左边缘白段（横路贯通/弯内触界） */
    uint8 saw_touch_r = 0;

    left_line_count = 0;
    right_line_count = 0;
    for (y = start_y; y >= 1 && left_line_count < IPTS_MAX; y--)
    {
        int16 x = 1;
        int16 best_left = -1;
        int16 best_right = -1;
        int16 best_score = 32767;

        while (x < PERS_W - 1)
        {
            int16 run_left;
            int16 run_right;
            int16 width;
            int16 center;
            int16 center_jump;
            int16 score;

            while (x < PERS_W - 1 && binary[y][x] < EDGE_WHITE_THRESHOLD) x++;
            if (x >= PERS_W - 1) break;
            run_left = x;
            while (x < PERS_W - 1 && binary[y][x] >= EDGE_WHITE_THRESHOLD) x++;
            run_right = x - 1;
            width = run_right - run_left + 1;
            center = (run_left + run_right) / 2;
            center_jump = abs(center - previous_center);

            /* 贴边宽白段：横路贯通到图像边缘（十字单角点判据），弯内触界也会置位，
             * 但十字识别还要求对侧折角，不会误判。 */
            if (width >= BINARY_START_MIN_WIDTH)
            {
                if (run_left <= 1) saw_touch_l = 1;
                if (run_right >= PERS_W - 2) saw_touch_r = 1;
            }

            if (run_left <= 1 || run_right >= PERS_W - 2 ||
                width < BINARY_START_MIN_WIDTH || width > BINARY_START_MAX_WIDTH ||
                center_jump > 14)
                continue;

            score = center_jump * 4 + abs(width - previous_width);
            if (score < best_score)
            {
                best_score = score;
                best_left = run_left;
                best_right = run_right;
            }
        }

        if (best_left < 0)
        {
            if (++miss_count >= 2) break;
            continue;
        }

        miss_count = 0;
        left_line_points[left_line_count][0] = best_left;
        left_line_points[left_line_count][1] = y;
        right_line_points[right_line_count][0] = best_right;
        right_line_points[right_line_count][1] = y;
        left_line_count++;
        right_line_count++;
        previous_center = (best_left + best_right) / 2;
        previous_width = best_right - best_left + 1;
    }
    if (saw_touch_l) touch_boundary0 = 1;
    if (saw_touch_r) touch_boundary1 = 1;
}

void find_edges_binary(void)
{
    int16 left_x;
    int16 right_x;
    int16 y = 0;

    touch_boundary0 = 0;
    touch_boundary1 = 0;
    left_line_count = 0;
    right_line_count = 0;
    maze_start_y = -1;
    maze_start_left_x = -1;
    maze_start_right_x = -1;
    if (!find_binary_start(image_binary, &y, &left_x, &right_x)) return;
    maze_start_y = y;   // 记录实际起始行（显示调试用）
    maze_start_left_x = left_x;
    maze_start_right_x = right_x;

    track_lane_by_rows(image_binary, y, left_x, right_x);
}
/**
 * 前瞻角误差计算
 * 鸟瞰米制坐标下：中线前瞻点相对车头方向的偏角 → pure_angle（方向环输入，单位：度）
 * 双边取中线前瞻；任意一侧无效时本帧不输出新中线
 */
void calculation_error(void)
{
    // 仅用于双边宽度合理性检查
    static float track_half_w = TRACK_HALF_W / pixel_per_meter;
    float mx, my;
    float dx, dy, dn;
    float cx, cy;

    // 车头投影位置：鸟瞰图底部中央（像素→米）
    cx = MT9V03X_W / 2 / pixel_per_meter;
    cy = (MT9V03X_H - 10) / pixel_per_meter;

    float target_y;
    float l_x = 0.0f, r_x = 0.0f;
    float l_front_y, r_front_y, common_front_y;
    float start_y, raw_start_y, max_target_y;
    int16 i, l_idx = -1, r_idx = -1;

    lookahead_lx = -1;
    lookahead_rx = -1;

#if CROSS_ENABLE
    /* ===== 十字中（enter/out）：导航目标钉在对面路口中心 cross_max_x =====
     * 移植自 STC32 例程 pure_track：cross_flag>1 时 pure_angle = max_x - 中线。
     * 本车 pure_angle 为纯跟踪角度量纲，故用路口中心构造虚拟前瞻点，
     * 走与正常巡线同一套 pure pursuit + 低通 + 限幅，保证量纲/手感一致。 */
    if (cross_flag >= CR_ENTER)
    {
        static float last_cross_angle = 0.0f;
        /* enter 首帧/列行扫描暂时失败：温和保持上一帧转向，
         * 绝不能落到横路乱边线上巡线 */
        if (cross_far_y <= 0 || cross_max_x <= 0)
        {
            state_flags = 0x40;
            pure_angle *= 0.95f;
            image_error_filter = (int16)pure_angle;
            return;
        }
        float cmx = cross_max_x / pixel_per_meter;
        float cmy = cross_far_y / pixel_per_meter;
        float cdx = cmx - cx;
        float cdy = cy - cmy + 0.2f;
        float cdn = sqrtf(cdx * cdx + cdy * cdy);
        float cnew;
        float cd;
        lookahead_y = cross_far_y;
        lookahead_lx = (cross_edge_l >= 0) ? cross_edge_l : -1;
        lookahead_rx = (cross_edge_r >= 0) ? cross_edge_r : -1;
        state_flags = 0x04;
        if (cdn > 0.01f)
        {
            pure_rad = -atanf(2.0f * 0.3f * cdx / (cdn * cdn));
            cnew = pure_rad * 57.2958f / SMOTOR_RATE;
            if (fabsf(cnew) < 1.0f) cnew = 0.0f;
            pure_angle = cnew * 0.35f + pure_angle * 0.65f;
            cd = pure_angle - last_cross_angle;
            if (cd >  4.0f) cd =  4.0f;
            if (cd < -4.0f) cd = -4.0f;
            pure_angle = last_cross_angle + cd;
            last_cross_angle = pure_angle;
        }
        image_error_filter = (int16)pure_angle;
        return;
    }
#endif  /* CROSS_ENABLE */

    if (maze_start_y < 0 || rpts0s_num < 2 || rpts1s_num < 2)
    {
        state_flags = 0x40;
        pure_angle *= 0.95f;
        image_error_filter = (int16)pure_angle;
        return;
    }

    // 动态前瞻：aim_distance 是最大长度；基准改为迷宫法实际起始行。
    // 双边中线较短时退回共同可用的最远端内侧 2px，避免取到边线端点。
    {
        static int16 stable_start_row = -1;
        int16 start_delta;

        if (stable_start_row < 0)
        {
            stable_start_row = maze_start_y;
        }
        else
        {
            start_delta = maze_start_y - stable_start_row;
            if (start_delta > 2) stable_start_row++;       // 超出死区后每帧最多下移 1 行
            else if (start_delta < -2) stable_start_row--; // 超出死区后每帧最多上移 1 行
        }
        start_y = stable_start_row / pixel_per_meter;
    }
    raw_start_y = maze_start_y / pixel_per_meter;
    max_target_y = ((start_y < raw_start_y) ? start_y : raw_start_y)
                 - 2.0f / pixel_per_meter;
    l_front_y = rpts0s[0][1];
    r_front_y = rpts1s[0][1];
    for (i = 1; i < rpts0s_num; i++)
        if (rpts0s[i][1] < l_front_y) l_front_y = rpts0s[i][1];
    for (i = 1; i < rpts1s_num; i++)
        if (rpts1s[i][1] < r_front_y) r_front_y = rpts1s[i][1];
    common_front_y = ((l_front_y > r_front_y) ? l_front_y : r_front_y)
                   + 2.0f / pixel_per_meter;
    if (common_front_y > max_target_y)
    {
        state_flags = 0x40;
        pure_angle *= 0.95f;
        image_error_filter = (int16)pure_angle;
        return;
    }
    target_y = start_y - aim_distance;
    if (target_y < common_front_y) target_y = common_front_y;

    // 动态长度防抖：中线缩短时立即收近，恢复后每帧只向远处移动 1px。
    // 否则远端边线点数每帧波动会让目标沿弯道前后跳，直接激励方向环摆动。
    {
        static uint8 target_y_ready = 0;
        static float filtered_target_y = 0.0f;
        const float extend_step = 1.0f / pixel_per_meter;

        if (!target_y_ready)
        {
            filtered_target_y = target_y;
            target_y_ready = 1;
        }
        else if (target_y > filtered_target_y)
        {
            filtered_target_y = target_y;       // 可用中线变短：立即缩近
        }
        else if (filtered_target_y - target_y > extend_step)
        {
            filtered_target_y -= extend_step;   // 中线恢复：缓慢向远端延伸
        }
        else
        {
            filtered_target_y = target_y;
        }

        if (filtered_target_y < common_front_y) filtered_target_y = common_front_y;
        if (filtered_target_y > max_target_y) filtered_target_y = max_target_y;
        target_y = filtered_target_y;
    }

    // 前瞻必须至少位于起始行前方 2px；更短说明没有可控的双边中线。
    if (target_y > max_target_y)
    {
        state_flags = 0x40;
        pure_angle *= 0.95f;
        image_error_filter = (int16)pure_angle;
        return;
    }

    // 前瞻附近必须存在连续双边中线，不能用断口边缘的孤立交点控制。
    {
        int16 target_row = (int16)(target_y * pixel_per_meter);
        int16 row;
        uint8 continuous = 1;
        for (row = target_row - 3; row <= target_row + 3; row++)
        {
            uint8 found = 0;
            uint16 p;
            for (p = 0; p < left_line_count; p++)
            {
                if (left_line_points[p][1] == row && right_line_points[p][1] == row)
                {
                    found = 1;
                    break;
                }
            }
            if (!found)
            {
                continuous = 0;
                break;
            }
        }
        if (!continuous)
        {
            state_flags = 0x40;
            pure_angle *= 0.95f;
            image_error_filter = (int16)pure_angle;
            return;
        }
    }

    uint8 l_ok = sample_line_x_at_y(rpts0s, rpts0s_num, target_y, &l_x, &l_idx);
    uint8 r_ok = sample_line_x_at_y(rpts1s, rpts1s_num, target_y, &r_x, &r_idx);

    // 纯双边巡线：任意一侧无效时直接判本帧无效，不使用单边理论补线。
    if (!l_ok || !r_ok)
    {
        state_flags = 0x40;
        pure_angle *= 0.95f;
        image_error_filter = (int16)pure_angle;
        return;
    }

    // 导出前瞻行左右边界点（调试：缩略图十字 + 宽度显示）
    lookahead_y = (int16)(target_y * pixel_per_meter);
    lookahead_lx = l_ok ? (int16)(l_x * pixel_per_meter) : -1;
    lookahead_rx = r_ok ? (int16)(r_x * pixel_per_meter) : -1;
    state_flags = 0;                       // 分支状态标志清零，各分支按需置位
    if (l_ok) state_flags |= 0x01;
    if (r_ok) state_flags |= 0x02;
    if (l_ok && r_ok)            // 双边都找到：同一 y 截面取中点，并更新实测半宽（带低通）
    {
        float new_half = fabsf(r_x - l_x) * 0.5f;
        float dir_diff = fabsf(rpts0a[l_idx] - rpts1a[r_idx]);   // 两侧边线形态差（弧度），闭合/串线时背离
        if (l_x < r_x   // 硬保护：左线横坐标必须小于右线（防左右反/串线）
            && new_half >= 0.05f && new_half <= track_half_w * 3.0f && dir_diff < corner_mismatch_th)
        {
            state_flags |= 0x04;                 // bit2=双边取中点（四重闸全过）
            // 宽度正常且两侧形态一致：取中点并更新实测半宽
            track_half_w = track_half_w * 0.7f + new_half * 0.3f;   // 半宽低通，防残端点污染
            mx = (l_x + r_x) / 2;
            my = target_y;
        }
        else   // 双边几何不可信：本帧无效，不降级到单边
        {
            state_flags = 0x40;
            pure_angle *= 0.95f;
            image_error_filter = (int16)pure_angle;
            return;
        }
    }
    else
    {
        state_flags |= 0x40;                 // bit6=全丢
        pure_angle *= 0.95f;   // 急弯内侧线短暂缺失时温和保持转向，避免频繁回正来回摆（出赛道保护兜底停车）
        image_error_filter = (int16)pure_angle;
        return;
    }
    // 只计算弯道角度供减速和显示使用；不再横向移动前瞻点。
    // mx 始终保持同一 y 截面上左右边界的真实中点。
    {
        float turn = 0.0f;
        int16 a_cnt = 0;
        corner_turn = 0.0f;
        if (l_ok && l_idx >= 0 && l_idx < rpts0a_num) { turn += rpts0a[l_idx]; a_cnt++; }
        if (r_ok && r_idx >= 0 && r_idx < rpts1a_num) { turn += rpts1a[r_idx]; a_cnt++; }
        if (a_cnt > 0)
        {
            turn /= a_cnt;                       // 正=右弯 负=左弯（弧度，图像坐标 y 向下）
            corner_turn = turn;                  // 调试显示
        }
    }
    // ===== 大弯道蜂鸣提示：turn 超过 corner_buz_th 边沿触发响一下 =====
    {
        static uint8 buz_prev = 0;
        uint8 buz_now = (fabsf(corner_turn) > corner_buz_th) ? 1 : 0;
        if (buz_now && !buz_prev) buzzer_tick = 8;   // 响约 8 帧（60~120fps 时约 70~130ms）
        buz_prev = buz_now;
        /* 递减与 beep_on/off 统一在 beeper_poll()（process_edge_points 末尾）执行 */
    }
    //车头指向前瞻点的向量（y轴指向车，dy>0 表示目标在前方）
    dx = mx - cx;
    dy = cy - my + 0.2f;
    dn = sqrtf(dx * dx + dy * dy);
    if (dn < 0.01f) return;

    // Pure Pursuit 曲率公式
    pure_rad = -atanf(2.0f * 0.3f * dx / (dn * dn));
    float new_angle = pure_rad * 57.2958f / SMOTOR_RATE;
    // 直道死区：偏角小于 1° 视为直道，强制归零，抑制直道舵机震荡
    if (fabsf(new_angle) < 1.0f) new_angle = 0.0f;
    // 轻度低通：0.35 新值 + 0.65 旧值，平衡响应和稳定
    pure_angle = new_angle * 0.35f + pure_angle * 0.65f;
    // 帧间变化限幅：单帧最大变化 4°，抑制急弯时偏差/舵机跳变（治抽搐）
    #define ANGLE_RATE_LIMIT (4.0f)
    {
        static float last_angle_out = 0.0f;
        static uint8 rl_ready = 0;
        float d;
        if (rl_ready)
        {
            d = pure_angle - last_angle_out;
            if (d >  ANGLE_RATE_LIMIT) d =  ANGLE_RATE_LIMIT;
            if (d < -ANGLE_RATE_LIMIT) d = -ANGLE_RATE_LIMIT;
            pure_angle = last_angle_out + d;
        }
        rl_ready = 1;
        last_angle_out = pure_angle;
    }
    image_error_filter = (int16)pure_angle;
    mid = (int16)(mx * pixel_per_meter);   // 中线前瞻点 x 像素坐标
    mid_y = (int16)(my * pixel_per_meter);   // 中线前瞻点 y 像素坐标（调试十字用）
}
//出赛道保护
void track_protection(void)
{
    static uint8 stop_outline_count = 0;  // 连续丢线帧计数
    static uint8 stop_black_count = 0;
    int16 scan_y = (track_protect_scan_row > 0) ? track_protect_scan_row : maze_start_y;
    int16 black_count = 0;
    int16 x;

#if CROSS_ENABLE
    /* 十字内部可能暂时没有常规双边线，由十字状态机负责驶出判定。 */
    if (cross_flag >= CR_ENTER)
    {
        stop_outline_count = 0;
        stop_black_count = 0;
        return;
    }
#endif

    if (scan_y < 0) scan_y = 0;
    if (scan_y >= MT9V03X_H) scan_y = MT9V03X_H - 1;
    for (x = 0; x < MT9V03X_W; x++)
        if (image_binary[scan_y][x] < EDGE_WHITE_THRESHOLD) black_count++;

    if (track_protect_black_threshold > 0 &&
        black_count >= track_protect_black_threshold)
        stop_black_count++;
    else
        stop_black_count = 0;

    // 纯双边巡线：任意一侧几乎找不到（点数 < 2），本帧判定丢线
    if (left_line_count < 2 || right_line_count < 2)
    {
        stop_outline_count++;
    }
    else
    {
        stop_outline_count = 0;  // 检测到边线，清零
    }

    // 连续 13 次检测到任意一侧丢线才停车，过滤瞬时漏检
    if (stop_outline_count > 12 ||
        (track_protect_black_frames > 0 &&
         stop_black_count >= track_protect_black_frames))
    {
        stop_outline_count = 0;
        stop_black_count = 0;
        stop_flog = 1;
        encoder_measure_flag = 0;   // 停车：结束测距，平均速度冻结
        display_flog = 1;           // 停车：恢复屏幕显示
    }
}
//整数裁剪
/* L0：原始浮点边线（从 left/right_line_points 转换来） */
float rpts0[POINTS_MAX_LEN][2];   float rpts1[POINTS_MAX_LEN][2];
int16 rpts0_num = 0, rpts1_num = 0;               // L0 实际点数

/* L1：加权平滑后边线（去锯齿） */
float rpts0b[POINTS_MAX_LEN][2];  float rpts1b[POINTS_MAX_LEN][2];
int16 rpts0b_num = 0, rpts1b_num = 0;             // L1 实际点数

/* L2：等距重采样后边线（相邻点距离恒为 sample_dist，核心数据） */
float rpts0s[POINTS_MAX_LEN][2];  float rpts1s[POINTS_MAX_LEN][2];
int16 rpts0s_num = 0, rpts1s_num = 0;             // L2 实际点数

/* L3：局部角度（每个点的转角，弧度，左转>0 右转<0） */
float rpts0a[POINTS_MAX_LEN];     float rpts1a[POINTS_MAX_LEN];
int16 rpts0a_num = 0, rpts1a_num = 0;             // L3 实际点数

/* L4：NMS 后角度（只剩局部最大峰，其他清零） */
float rpts0an[POINTS_MAX_LEN];    float rpts1an[POINTS_MAX_LEN];
int16 rpts0an_num = 0, rpts1an_num = 0;           // L4 实际点数

/* 近端 L 角点（正峰）：在 rpts0s/rpts1s 中的索引 + 找到标志 */
int16 Lpt0_rpts0s_id = 0, Lpt1_rpts1s_id = 0;     // 左/右 L 角点索引
int16 N_Lpt0_rpts0s_id = 0, N_Lpt1_rpts1s_id = 0; // 左/右 反L角点索引（负峰）
int16 Lpt0_found = 0, Lpt1_found = 0;             // 1=找到 L 角点
int16 N_Lpt0_found = 0, N_Lpt1_found = 0;         // 1=找到 反L 角点

/* 远端 L 角点（十字判据用：两个入口角点之间的垂直距离校验） */
int16 far_Lpt0_rpts0s_id = 0, far_Lpt1_rpts1s_id = 0; // 远端角点索引
int16 far_Lpt0_found = 0, far_Lpt1_found = 0;     // 1=找到远端角点

/* 远端边线段（十字内部导航时切到远端边线算偏差） */
float far_rpts0s[POINTS_MAX_LEN][2]; float far_rpts1s[POINTS_MAX_LEN][2];
float far_orig0[POINTS_MAX_LEN][2]; float far_orig1[POINTS_MAX_LEN][2];
int16 far_rpts0s_num = 0, far_rpts1s_num = 0;     // 远端边线实际点数

/* 直线度 + 置信度（用于区分直道/弯道/十字） */
int16 is_straight0 = 0, is_straight1 = 0;         // 1=该边线近似直线
float conf1 = 0, conf2 = 0;                       // 当前帧角度峰值
float conf1_max = 0, conf2_max = 0;               // 最近端最大角度峰值
int16 clip(int16 x,int16 low,int16 up)
{
    if(x>up) return up;
    if(x<low) return low;
    return x;
}
//浮点数限幅
float fclip(float x,float low,float up)
{
    if(x>up) return up;
    if(x<low) return low;
    return x;
}
/* ========================================================================
 * L1：加权平滑
 * 功能：对输入点集做三角形加权滑动平均，去掉二值图边线的锯齿
 * 原理：每个点取前后各 half 个邻居，权重 1,2,...,half+1,...,2,1
 *       权重和 = (half+1)^2，除以权重和得到加权平均
 * 参数：
 *   pts_in  - 输入点集（L0 原始浮点）
 *   num     - 输入点数
 *   pts_out - 输出平滑后点集（L1）
 *   kernel  - 窗口大小，必须是奇数（3/5/7），越大越平滑
 * 边界处理：用 clip 把越界索引拉回 0~num-1（镜像延伸）
 * ======================================================================== */
void blur_points(float pts_in[][2],int16 num,float pts_out[][2],int16 kernel)
{
    int16 half=kernel/2;
    int i,j;

    for(i=0;i<num;i++)
    {
        pts_out[i][0]=0;
        pts_out[i][1]=0;

        for(j=-half;j<=half;j++)
        {
            //把越界索引拉回合法范围
           int16 k=clip(i+j,0,num-1);
           //三角权重
           int16 w=half+1-abs(j);
           pts_out[i][0] += pts_in[k][0] * w;   // x 加权累加
           pts_out[i][1] += pts_in[k][1] * w;   // y 加权累加
        }
        // 除以权重和 (half+1)^2 得到加权平均
        // kernel=3: half=1, 权重 1,2,1, 和=4=(1+1)^2
        // kernel=5: half=2, 权重 1,2,3,2,1, 和=9=(2+1)^2
        pts_out[i][0] /= (half + 1) * (half + 1);
        pts_out[i][1] /= (half + 1) * (half + 1);
    }
}
/*
 * L2：等距重采样
 * 功能：把间距不均的边线点重新取点，让相邻点距离恒为 dist
 * 为什么是核心：原始边线点直道稀疏、弯道密集，无法做"距离"判断；
 *              重采样后 点数=距离，"1.6米内找角点"才能实现
 * 原理：remain 累加器——沿曲线一段段走，每走满 dist 距离插一个点
 * 参数：
 *   pts_in  - 输入点集（L1 平滑后）
 *   num1    - 输入点数
 *   pts_out - 输出等距点集（L2）
 *   num2    - 输入时=输出容量(POINTS_MAX_LEN)，返回时=实际点数
 *   dist    - 采样间距（sample_dist=0.02，单位与 pixel_per_meter 绑定）
 *  */
void resample_points(float pts_in[][2],int16 num1,float pts_out[][2],int16 *num2,float dist)
{
    float remain=0;         // 距上次插点已走过的距离
    int16 len=0;            // 已插出的点数
    int16 i;

    for(i=0;i<num1-1 && len<*num2;i++)
    {
        float x0=pts_in[i][0];              // 当前段起点
        float y0=pts_in[i][1];
        float dx=pts_in[i+1][0]-x0;         // 当前段方向
        float dy=pts_in[i+1][1]-y0;
        float dn=sqrtf(dx*dx+dy*dy);        // 当前段长度

        if(dn<0.0001f) continue;            // 零长度段跳过，防除零

        dx/=dn;                             // 归一化成单位方向向量
        dy/=dn;

        // 在当前段内不断插点，直到剩余长度不足一个 dist
        while(remain<dn && len<*num2)
        {
            x0+=dx*remain;                  // 从段起点向前走 remain 距离
            y0+=dy*remain;
            pts_out[len][0]=x0;             // 存采样点
            pts_out[len][1]=y0;
            len++;
            dn-=remain;                     // 当前段剩余长度减掉刚走的部分
            remain=dist;                    // 下一个插点间隔重置为 dist
        }
        remain-=dn;                         // 段走完了：把"多要的"留给下一段补
    }
    *num2=len;                              // 返回实际采样点数
}
/* ========================================================================
 * L3：局部角度
 * 功能：对等距点集的每个点，算该处的"转弯角度"（弧度，带正负号）
 * 原理：取点 i 前方 dist 个点和后方 dist 个点，算两个方向向量，
 *       用 atan2 求从方向1转到方向2的角度
 * 输出特性：直道≈0；左转>0；右转<0；角点处出现尖峰
 *           → 找角点变成"在角度波形里找峰值"
 * 参数：
 *   pts_in    - 输入点集（L2 等距重采样后）
 *   num       - 输入点数
 *   angle_out - 输出角度数组（弧度）
 *   dist      - 前后参考点间距（angle_dist=0.05 → 约等于 0.05*70=3.5 个点）
 * ======================================================================== */
void local_angle_points(float pts_in[][2],int16 num,float angle_out[],int16 dist)
{
    int16 i;

    for(i=0;i<num;i++)
    {
        // 边界点算不了（前方或后方凑不出 dist 个点），直接置0
        if(i-dist<=0 || i+dist>=num-1)
        {
            angle_out[i]=0;
            continue;
        }

        // 前段方向向量：从 i-dist 指向 i
        float dx1=pts_in[i][0]-pts_in[clip(i-dist,0,num-1)][0];
        float dy1=pts_in[i][1]-pts_in[clip(i-dist,0,num-1)][1];
        float dn1=sqrtf(dx1*dx1+dy1*dy1);

        // 后段方向向量：从 i 指向 i+dist
        float dx2=pts_in[clip(i+dist,0,num-1)][0]-pts_in[i][0];
        float dy2=pts_in[clip(i+dist,0,num-1)][1]-pts_in[i][1];
        float dn2=sqrtf(dx2*dx2+dy2*dy2);

        if(dn1<0.0001f || dn2<0.0001f)   // 向量退化（点重合），不算角度
        {
            angle_out[i]=0;
            continue;
        }

        // 归一化成单位向量 (c=cos分量, s=sin分量)
        float c1=dx1/dn1, s1=dy1/dn1;
        float c2=dx2/dn2, s2=dy2/dn2;

        // 两方向夹角：atan2(叉积, 点积)
        // 叉积 c1*s2-c2*s1 判正负（左转/右转）
        // 点积 c2*c1+s2*s1 判夹角大小
        // 比 acos 稳定（acos 在 0°/180° 附近数值误差大）
        angle_out[i]=atan2f(c1*s2-c2*s1, c2*c1+s2*s1);
    }
}
/* ========================================================================
 * L4：非极大值抑制（NMS，Non-Maximum Suppression）
 * 功能：在角度信号上滑窗，窗口内只保留 |角度| 最大的点，其余清零
 * 为什么需要：一个真实角点在 L3 里会"涂抹"成连续 3~5 个较大角度
 *            （因为角度计算看的是前后窗口，角点两侧的点也会沾上）
 *            NMS 把涂抹压回成单个峰，角点位置才唯一
 * 示例：L3输出  0 0 0 5 8 12 7 3 0 0   ← 一个角点被涂成一片
 *       L4输出  0 0 0 0 0 12 0 0 0 0   ← 只剩峰值 12
 * 参数：
 *   angle_in  - 输入角度数组（L3 局部角度）
 *   num       - 点数
 *   angle_out - 输出抑制后的角度数组（L4）
 *   kernel    - 抑制窗口大小（奇数），kernel=3 即前后各看1个点
 * ======================================================================== */
void nms_angle(float angle_in[],int16 num,float angle_out[],int16 kernel)
{
    int16 half=kernel/2;
    int16 i,j;

    for(i=0;i<num;i++)
    {
        angle_out[i]=angle_in[i];           // 先假设自己是最大值，保留

        for(j=-half;j<=half;j++)            // 看窗口内的所有邻居
        {
            int16 k=clip(i+j,0,num-1);

            if(fabsf(angle_in[k])>fabsf(angle_out[i]))
            {
                angle_out[i]=0;             // 有邻居比我的绝对值大 → 我被抑制
                break;
            }
        }
    }
}
/* ========================================================================
 * 边线法线偏移生成虚拟中线（国一算法）
 * 对边线每个点，计算切线方向，旋转90°得到法线，沿法线偏移 dist
 * 弯道时边线是斜的，法线不是水平的，这样生成的中线才不会向外折
 * ======================================================================== */
void track_leftline(float pts_in[][2], int16 num, float pts_out[][2], int16 approx_num, float dist)
{
    int16 i;
    for (i = 0; i < num; i++)
    {
        int16 i_prev = clip(i - approx_num, 0, num - 1);
        int16 i_next = clip(i + approx_num, 0, num - 1);
        float dx = pts_in[i_next][0] - pts_in[i_prev][0];
        float dy = pts_in[i_next][1] - pts_in[i_prev][1];
        float dn = sqrtf(dx * dx + dy * dy);
        if (dn < 0.0001f) { pts_out[i][0] = pts_in[i][0]; pts_out[i][1] = pts_in[i][1]; continue; }
        dx /= dn; dy /= dn;
        // 左线→中线：向右偏移，法线 = (-dy, dx)
        pts_out[i][0] = pts_in[i][0] - dy * dist;
        pts_out[i][1] = pts_in[i][1] + dx * dist;
    }
}

void track_rightline(float pts_in[][2], int16 num, float pts_out[][2], int16 approx_num, float dist)
{
    int16 i;
    for (i = 0; i < num; i++)
    {
        int16 i_prev = clip(i - approx_num, 0, num - 1);
        int16 i_next = clip(i + approx_num, 0, num - 1);
        float dx = pts_in[i_next][0] - pts_in[i_prev][0];
        float dy = pts_in[i_next][1] - pts_in[i_prev][1];
        float dn = sqrtf(dx * dx + dy * dy);
        if (dn < 0.0001f) { pts_out[i][0] = pts_in[i][0]; pts_out[i][1] = pts_in[i][1]; continue; }
        dx /= dn; dy /= dn;
        // 右线→中线：向左偏移，法线 = (dy, -dx)
        pts_out[i][0] = pts_in[i][0] + dy * dist;
        pts_out[i][1] = pts_in[i][1] - dx * dist;
    }
}
/* ========================================================================
 * 桥接函数：int16 边线 → 4级点云流水线
 * 调用时机：原图二值迷宫巡线之后（每帧一次）
 * 数据流：
 *   left_line_points(int16) → rpts0(float) → rpts0b(平滑) → rpts0s(等距)
 *   → rpts0a(角度) → rpts0an(NMS)
 * ======================================================================== */
void process_edge_points(void)
{
    int16 i;

    /* 每帧先生成逆透视灰度图，供屏幕调试和远端角点使用。 */
    anti_perspective_fast();

    /* ---- L0：int16 → float 直接复制 ---- */
    rpts0_num=(int16)left_line_count;
    rpts1_num=(int16)right_line_count;

    for(i=0;i<rpts0_num;i++)
    {
        rpts0[i][0]=left_line_points[i][0]/pixel_per_meter;   //像素→米,让sample_dist(米)有真实意义
        rpts0[i][1]=left_line_points[i][1]/pixel_per_meter;
    }
    for(i=0;i<rpts1_num;i++)
    {
        rpts1[i][0]=right_line_points[i][0]/pixel_per_meter;
        rpts1[i][1]=right_line_points[i][1]/pixel_per_meter;
    }

    /* ---- L1：平滑（kernel=3）---- */
    blur_points(rpts0,rpts0_num,rpts0b,3);
    rpts0b_num=rpts0_num;
    blur_points(rpts1,rpts1_num,rpts1b,3);
    rpts1b_num=rpts1_num;

    /* ---- L2：等距重采样（先设容量上限，返回实际点数）---- */
    rpts0s_num=POINTS_MAX_LEN;
    resample_points(rpts0b,rpts0b_num,rpts0s,&rpts0s_num,sample_dist);
    rpts1s_num=POINTS_MAX_LEN;
    resample_points(rpts1b,rpts1b_num,rpts1s,&rpts1s_num,sample_dist);

    /* ---- L3：局部角度 ----
     * 注意：第4个参数是"点数"不是米！
     * 换算：0.05米 × 70像素/米 ÷ 1.4像素/点 ≈ 2.5 → 取3个点
     * 直接传 angle_dist(0.05) 会截断成 0，等于没算角度！ */
    local_angle_points(rpts0s,rpts0s_num,rpts0a,3);
    rpts0a_num=rpts0s_num;
    local_angle_points(rpts1s,rpts1s_num,rpts1a,3);
    rpts1a_num=rpts1s_num;

    /* ---- L4：NMS ---- */
    
    nms_angle(rpts0a,rpts0a_num,rpts0an,7);
    rpts0an_num=rpts0s_num;
    nms_angle(rpts1a,rpts1a_num,rpts1an,7);
    rpts1an_num=rpts1s_num;

    
    /* 近端角点只作为十字候选触发器；NONE 状态不显示、不上报、不改寻线数据。 */
    find_corners();
#if CROSS_ENABLE
    /* 四角点检测只读原图；四点齐全后状态机才允许进入十字候选。 */
    find_far_corners_crawl();
    cross_line_completion();
    if (cross_flag >= CR_ENTER)
        find_cross_center();
#else
    far_Lpt0_found = far_Lpt1_found = 0;
    far_Lpt0_rpts0s_id = far_Lpt1_rpts1s_id = -1;
    far_rpts0s_num = far_rpts1s_num = 0;
    crawl_show_ymax = -1;
#endif
    /* START 用锁存四角点；ENTER/OUT 用车头到对面入口的动态导航走廊。 */
    cross_build_vlines();
    beeper_poll();             /* 蜂鸣器统一输出：大弯道 + 十字（关闭时只剩大弯道） */
    
}
