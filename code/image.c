#include "image.h"
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
// 二值图中白色像素的判断阈值。
#define EDGE_WHITE_THRESHOLD (128)

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
static void display_otsu_thresholds(void)
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

    // === Step 2: 第一次大津 → T1 ===
    threshold = otsu_compute(histogram, pixel_count);

    // === Step 3: 直方图重组（把低于 T1 的像素合并到 T1 位置） ===
    // 模拟"把所有低于 T1 的像素拉高到 T1"的效果
    low_sum = 0;
    for (level = 0; level < threshold; level++)
    {
        low_sum += histogram[level];
        histogram[level] = 0;
    }
    histogram[threshold] += low_sum;

    // === Step 4: 第二次大津 → T2（最终阈值） ===
    threshold = otsu_compute(histogram, pixel_count);

    // === Step 5: 用 T2 二值化全图 ===
    for (y = 0; y < MT9V03X_H; y++)
    {
        for (x = 0; x < MT9V03X_W; x++)
        {
            image_binary[y][x] = (image[y][x] < threshold) ? 0 : 255;
        }
    }
}
/**
 * @brief 以左手法则沿二值图白边线进行路径追踪。
 * @details
 *      从起点 start_x, start_y 开始，沿着白色边线前进。
 *      该算法采用典型的“左手跟墙法”：
 *      - 正前方是白色且左前方是黑色：继续前进；
 *      - 正前方是黑色：说明前方有障碍，向右转；
 *      - 正前方和左前方都是白色：沿左前方方向前进并转向左。
 *      适用于迷宫、边线、白区跟踪等二值图场景。
 *
 * @param binary  输入二值图。
 * @param start_x 起点横坐标，必须在图像内部区域。
 * @param start_y 起点纵坐标，必须在图像内部区域。
 * @param points  输出路径点数组，按 [x, y] 的形式保存每一步经过的坐标。输入points[0][2]第二个坐标表示要输入2个数
 * @param point_count 输入时为 points 数组的最大容量；输出时为实际记录到的点数量。
 *
 * @note 该函数本质上是“左手迷宫法/跟墙法”，用于从某个白色起点沿白边线持续搜索并记录路径。
 */
void findline_lefthand_binary(const uint8 binary[MT9V03X_H][MT9V03X_W], int16 start_x, int16 start_y, int16 points[][2], uint16 *point_count)
{
    uint16 max_points;
    int16 x;
    int16 y;
    uint16 step;
    int16 dir;
    int16 turn;
    int16 fx, fy, flx, fly;          // 前半段需要补这 4 个临时坐标变量

    if (binary == 0 || points == 0 || point_count == 0)
    {
        return;
    }

    max_points = *point_count;

    *point_count = 0;

    if (max_points == 0)
    {
        return;
    }                                // ← 这里补上右括号（原来漏了，是整个前半段唯一的错）

    if ((start_x <= 0) || (start_x >= (MT9V03X_W - 1)) || (start_y <= 0) || (start_y >= (MT9V03X_H - 1)))
    {
        return;
    }

    if (binary[start_y][start_x] < EDGE_WHITE_THRESHOLD)
    {
        return;
    }

    x = start_x;
    y = start_y;
    dir = 0;                         // 初始朝上
    turn = 0;
    step = 0;

    points[0][0] = x;                // 记录起点
    points[0][1] = y;

    while ((step < max_points - 1) && (x > 0) && (y > 0) && (x < MT9V03X_W - 1) && (y < MT9V03X_H - 1) && (turn < 4))
    {
        fx  = x + edge_dir_front[dir][0];
        fy  = y + edge_dir_front[dir][1];
        flx = x + edge_dir_frontleft[dir][0];
        fly = y + edge_dir_frontleft[dir][1];

        if (binary[fy][fx] >= EDGE_WHITE_THRESHOLD)        // 正前方是白
        {
            if (binary[fly][flx] >= EDGE_WHITE_THRESHOLD)  // 左前也是白 → 走左前并左转
            {
                dir = (dir + 3) % 4;
                x = flx;
                y = fly;
            }
            else                                           // 左前是黑 → 直走
            {
                x = fx;
                y = fy;
            }
            turn = 0;
            step++;
            points[step][0] = x;
            points[step][1] = y;
        }
        else                                               // 正前方是黑 → 右转，原地不动
        {
            dir = (dir + 1) % 4;
            turn++;
        }
    }

    *point_count = step + 1;
}
void findline_righthand_binary(const uint8 binary[MT9V03X_H][MT9V03X_W], int16 start_x, int16 start_y, int16 points[][2], uint16 *point_count)
{
    uint16 max_points;
    int16 x;
    int16 y;
    uint16 step;
    int16 dir;
    int16 turn;
    int16 fx, fy, frx, fry;

    if (binary == 0 || points == 0 || point_count == 0)
    {
        return;
    }

    max_points = *point_count;
    *point_count = 0;

    if (max_points == 0)
    {
        return;
    }

    if ((start_x <= 0) || (start_x >= (MT9V03X_W - 1)) || (start_y <= 0) || (start_y >= (MT9V03X_H - 1)))
    {
        return;
    }

    if (binary[start_y][start_x] < EDGE_WHITE_THRESHOLD)
    {
        return;
    }

    x = start_x;
    y = start_y;
    dir = 0;
    turn = 0;
    step = 0;

    points[0][0] = x;
    points[0][1] = y;

    while ((step < max_points - 1) && (x > 0) && (y > 0) && (x < MT9V03X_W - 1) && (y < MT9V03X_H - 1) && (turn < 4))
    {
        fx  = x + edge_dir_front[dir][0];
        fy  = y + edge_dir_front[dir][1];
        frx = x + edge_dir_frontright[dir][0];
        fry = y + edge_dir_frontright[dir][1];

        if (binary[fy][fx] >= EDGE_WHITE_THRESHOLD)        // 正前方白
        {
            if (binary[fry][frx] >= EDGE_WHITE_THRESHOLD)  // 右前也白 → 走右前并右转
            {
                dir = (dir + 1) % 4;
                x = frx;
                y = fry;
            }
            else                                           // 右前黑 → 直走
            {
                x = fx;
                y = fy;
            }
            turn = 0;
            step++;
            points[step][0] = x;
            points[step][1] = y;
        }
        else                                               // 正前方黑 → 左转
        {
            dir = (dir + 3) % 4;
            turn++;
        }
    }

    *point_count = step + 1;
}
static void find_binary_start(const uint8 binary[MT9V03X_H][MT9V03X_W], int16 *y, int16 *left_x, int16 *right_x)
{
    int16 x;
    int16 yy;

    //从底部往上找第一行有白点的行，自动跳过底部黑边脏区
    for (yy = MT9V03X_H - 3; yy > MT9V03X_H / 2; yy--)
    {
        for (x = 1; x < MT9V03X_W - 1; x++)
        {
            if (binary[yy][x] >= EDGE_WHITE_THRESHOLD) break;   //这行扫到白点了
        }
        if (x < MT9V03X_W - 1)
        {
            break;      //说明这行有白点，就是有效起跑行
        }
    }

    *y = yy;        //把实际用的行号带回去给调用方

    //向上搜行时停下的x恰好就是该行第一个白点，直接当左边线用
    if (x >= MT9V03X_W - 1)
    {
        *left_x = 1;        //兜底：半幅图全黑没找到白点
    }
    else
    {
        *left_x = x;
    }

    for (x = MT9V03X_W - 2; x >= 0; x--)
    {
        if (binary[yy][x] >= EDGE_WHITE_THRESHOLD) break;    // 找到最后一个白点
    }
    if (x <= 0)
    {
        *right_x = MT9V03X_W - 2;
    }
    else
    {
        *right_x = x;
    }
}
void find_edges_binary(void)
{
    int16 left_x,right_x;
    int16 y0=MT9V03X_H-3;
    left_line_count=IPTS_MAX;
    right_line_count=IPTS_MAX;

    find_binary_start(image_binary,&y0,&left_x,&right_x);  

    findline_lefthand_binary(image_binary,left_x,y0,left_line_points,&left_line_count);
    findline_righthand_binary(image_binary,right_x,y0,right_line_points,&right_line_count); 
}
void calculation_error(void)
{
    int16 y_yow;        //偏差采样行
    int16 left_x;       //采样行的左边线x
    int16 right_x;      //采样行的右边线x
    int16 error_left;   //左边线单独计算偏差
    int16 error_right;  //右边线单独计算偏差
    uint16 i;

    y_yow = MT9V03X_H - 10;     //110行
    left_x = -1;
    right_x = -1;

    //边线点从底部往上记，找第一个y<=y_yow的点就是采样行的边线
    for (i = 0; i < left_line_count; i++)
    {
        if (left_line_points[i][1] <= y_yow)
        {
            left_x = left_line_points[i][0];    //采样行的左边线x
            break;
        }
    }

    for (i = 0; i < right_line_count; i++)      //和上面平级
    {
        if (right_line_points[i][1] <= y_yow)
        {
            right_x = right_line_points[i][0];  //采样行的右边线x
            break;
        }
    }

    if (left_x >= 0 && right_x >= 0)            //两边都在：取平均，并清停车标志
    {
        error_left = left_x + image_center;
        error_right = right_x - image_center;
        mid = (error_left + error_right) / 2;
        image_error = mid - image_center;
        stop_flog = 0;
    }
    else if (left_x >= 0)                       //只有左边线：左+94补中线
    {
        mid = left_x + image_center;
        image_error = mid - image_center;
    }
    else if (right_x >= 0)                      //只有右边线：右-94补中线
    {
        mid = right_x - image_center;
        image_error = mid - image_center;
    }
    else                                        //两边全丢：置停车标志，误差保持上次值
    {
       
    }
        //一阶低通：新值占1/4，旧值占3/4，越大越平滑越迟钝
    image_error_filter = image_error_filter+(image_error - image_error_filter) / 4;
}

//出赛道保护
void track_protection(void)
{
    #define WHITEM_SCAN_ROW (MT9V03X_H-10)//扫描行，从底部网上10行
    #define WHITEM_STOP_LIMIT (15) //扫描白点数的阈值

    int x;
    uint16 white_cnt=0;
    int16 y_yow=MT9V03X_H-10;
    int16 left_x=-1;
    int16 right_x=-1;

    //从底部向上爬线，找到最近的目标行作为左线x
    for(int i=0;i<left_line_count;i++)
    {
        if(left_line_points[i][1]<=y_yow)
        {
            left_x=left_line_points[i][0];
            break;
        }
    }
    //从底部向上爬线，找到最近的目标行作为右线x
    for(int i=0;i<right_line_count;i++)
    {
        if(right_line_points[i][1]<=y_yow)
        {
            right_x=right_line_points[i][0];
            break;
        }
    }
    //左右边线都找到
    if(left_x>=0&&right_x>=0)
    {
        return ;
    }
    //如果左右边线不全在扫描行，白点数大于一定阈值说明出赛道
    for(x=0;x<MT9V03X_W;x++)
    {
        if(image_binary[WHITEM_SCAN_ROW][x]>=EDGE_WHITE_THRESHOLD)
        {
            white_cnt++;
        }
    }
    //如果画面白点数小于阈值，说明没有白点，需要停车
    if(white_cnt<=WHITEM_STOP_LIMIT)
    {
        stop_flog=1;
    }
    else //说明只扫到了一条白线或者没有边线
    {
        
    }
}
