#include "image.h"
#include "camera_param.h"
#include "servo.h"
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

/* ================ 鸟瞰图（逆透视） ================ */
uint8 img_pers_data[PERS_H][PERS_W];                    // 鸟瞰灰度图
image_t img_pers = { (uint8 *)img_pers_data, PERS_W, PERS_H, PERS_W };
int16 touch_boundary0 = 0;      // 左巡线碰到图像边界
int16 touch_boundary1 = 0;      // 右巡线碰到图像边界
// 二值图中白色像素的判断阈值。
#define EDGE_WHITE_THRESHOLD (128)
#define BINARY_START_Y        (PERS_H - 3)
#define BINARY_START_OFFSET   (8)
#define BINARY_START_SEARCH_H (12)

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
                img_pers_data[j][i] = mt9v03x_image[sy][sx];
            }
            else
            {
                img_pers_data[j][i] = 0;
            }
        }
    }
}
static void findline_lefthand_binary(const uint8 binary[PERS_H][PERS_W], int16 x, int16 y, int16 points[][2], uint16 *point_count)
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

static void findline_righthand_binary(const uint8 binary[PERS_H][PERS_W], int16 x, int16 y, int16 points[][2], uint16 *point_count)
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

static uint8 find_binary_start(const uint8 binary[PERS_H][PERS_W], int16 *y, int16 *left_x, int16 *right_x)
{
    int16 yy;
    int16 x0;
    int16 x1;
    int16 row_offset;

    for (row_offset = 0; row_offset < BINARY_START_SEARCH_H; row_offset++)
    {
        yy = BINARY_START_Y - row_offset;
        if (binary[yy][PERS_W / 2] < EDGE_WHITE_THRESHOLD)
        {
            continue;
        }

        x0 = PERS_W / 2 - BINARY_START_OFFSET;
        while (x0 > 1 && binary[yy][x0 - 1] >= EDGE_WHITE_THRESHOLD) x0--;

        x1 = PERS_W / 2 + BINARY_START_OFFSET;
        while (x1 < PERS_W - 2 && binary[yy][x1 + 1] >= EDGE_WHITE_THRESHOLD) x1++;

        if (binary[yy][x0] >= EDGE_WHITE_THRESHOLD &&
            binary[yy][x1] >= EDGE_WHITE_THRESHOLD && x1 > x0)
        {
            *y = yy;
            *left_x = x0;
            *right_x = x1;
            return 1;
        }
    }
    return 0;
}

void find_edges_binary(void)
{
    int16 left_x;
    int16 right_x;
    int16 y = PERS_H - 3;

    touch_boundary0 = 0;
    touch_boundary1 = 0;
    left_line_count = 0;
    right_line_count = 0;
    if (!find_binary_start(image_binary, &y, &left_x, &right_x)) return;

    left_line_count = IPTS_MAX;
    right_line_count = IPTS_MAX;
    findline_lefthand_binary(image_binary, left_x, y, left_line_points, &left_line_count);
    findline_righthand_binary(image_binary, right_x, y, right_line_points, &right_line_count);
}
/**
 * 前瞻角误差计算
 * 鸟瞰米制坐标下：中线前瞻点相对车头方向的偏角 → pure_angle（方向环输入，单位：度）
 * 三分支：双边取中线前瞻 / 单边±米制半宽补线 / 全丢保持上次输出
 */
void calculation_error(void)
{
    float mx, my;
    float dx, dy, dn;
    float cx, cy;
    float half_w;

    // 车头投影位置：鸟瞰图底部中央（像素→米）
    cx = MT9V03X_W / 2 / pixel_per_meter;
    cy = (MT9V03X_H - 10) / pixel_per_meter;

    // 米制半赛道宽（单边补线用）
    half_w = TRACK_HALF_W / pixel_per_meter;

    // 目标前瞻 y 坐标（车头前方 aim_distance 米处）
    float target_y = cy - aim_distance;
    int16 i, l_idx = -1, r_idx = -1;
    float l_min_d = 1e9f, r_min_d = 1e9f;

    // 在左线找 y 最接近 target_y 的点
    for (i = 0; i < rpts0s_num; i++)
    {
        float d = fabsf(rpts0s[i][1] - target_y);
        if (d < l_min_d) { l_min_d = d; l_idx = i; }
    }
    // 在右线找 y 最接近 target_y 的点
    for (i = 0; i < rpts1s_num; i++)
    {
        float d = fabsf(rpts1s[i][1] - target_y);
        if (d < r_min_d) { r_min_d = d; r_idx = i; }
    }

    if (l_idx >= 0 && r_idx >= 0)            // 双边都找到：同一 y 截面取中点
    {
        mx = (rpts0s[l_idx][0] + rpts1s[r_idx][0]) / 2;
        my = (rpts0s[l_idx][1] + rpts1s[r_idx][1]) / 2;
    }
    else if (l_idx >= 0)                     // 只有左线：沿法线向右偏移半宽
    {
        int16 p = clip(l_idx - 3, 0, rpts0s_num - 1);
        int16 n = clip(l_idx + 3, 0, rpts0s_num - 1);
        float ddx = rpts0s[n][0] - rpts0s[p][0];
        float ddy = rpts0s[n][1] - rpts0s[p][1];
        float ddn = sqrtf(ddx * ddx + ddy * ddy);
        if (ddn > 0.0001f) { ddx /= ddn; ddy /= ddn; }
        mx = rpts0s[l_idx][0] - ddy * half_w;   // 法线 = (-dy, dx)
        my = rpts0s[l_idx][1] + ddx * half_w;
    }
    else if (r_idx >= 0)                     // 只有右线：沿法线向左偏移半宽
    {
        int16 p = clip(r_idx - 3, 0, rpts1s_num - 1);
        int16 n = clip(r_idx + 3, 0, rpts1s_num - 1);
        float ddx = rpts1s[n][0] - rpts1s[p][0];
        float ddy = rpts1s[n][1] - rpts1s[p][1];
        float ddn = sqrtf(ddx * ddx + ddy * ddy);
        if (ddn > 0.0001f) { ddx /= ddn; ddy /= ddn; }
        mx = rpts1s[r_idx][0] + ddy * half_w;   // 法线 = (dy, -dx)
        my = rpts1s[r_idx][1] - ddx * half_w;
    }
    else                                     // 全丢：角度衰减回正
    {
        pure_angle *= 0.7f;
        image_error_filter = (int16)pure_angle;
        return;
    }
    //车头指向前瞻点的向量（y轴指向车，dy>0 表示目标在前方）
    dx = mx - cx;
    dy = cy - my + 0.2f;
    dn = sqrtf(dx * dx + dy * dy);
    if (dn < 0.01f) return;

    // Pure Pursuit 曲率公式
    pure_rad = -atanf(2.0f * 0.3f * dx / (dn * dn));
    float new_angle = pure_rad * 57.2958f / SMOTOR_RATE;
    // 轻度低通：0.5 新值 + 0.5 旧值，平衡响应和稳定
    pure_angle = new_angle * 0.35f + pure_angle * 0.65f;
    image_error_filter = (int16)pure_angle;
    mid = (int16)(mx * pixel_per_meter);   // 中线前瞻点 x 像素坐标
}
//出赛道保护
void track_protection(void)
{
    static uint8 stop_outline_count = 0;  // 连续丢线帧计数

    // 左右边线都几乎找不到（点数 < 2），判定丢线
    if (left_line_count < 2 && right_line_count < 2)
    {
        stop_outline_count++;
    }
    else
    {
        stop_outline_count = 0;  // 检测到边线，清零
    }

    // 连续 12 帧都丢线才停车（防十字口误触发）
    if (stop_outline_count > 12)
    {
        stop_outline_count = 0;
        stop_flog = 1;
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

    /* ---- 找角点（函数还没写，先注释掉）----
    find_corners();
    find_far_corners();
    */
}
