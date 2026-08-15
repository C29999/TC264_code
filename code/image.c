#include "image.h"
#define IMAGE_OTSU_BLOCK_W      (188)  // 每个局部区域的宽度
#define IMAGE_OTSU_BLOCK_H      (20)  // 每个局部区域的高度

#define IMAGE_OTSU_BLOCK_COLS \
    ((MT9V03X_W + IMAGE_OTSU_BLOCK_W - 1) / IMAGE_OTSU_BLOCK_W) // 横向区域数量

#define IMAGE_OTSU_BLOCK_ROWS \
    ((MT9V03X_H + IMAGE_OTSU_BLOCK_H - 1) / IMAGE_OTSU_BLOCK_H) // 纵向区域数量

uint8 image_threshold_map[IMAGE_OTSU_BLOCK_ROWS][IMAGE_OTSU_BLOCK_COLS];
// 保存每个局部区域计算出来的阈值

uint8 image_binary[MT9V03X_H][MT9V03X_W];


// 二值图中白色像素的判断阈值。
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

void image_threshold(const uint8 image[MT9V03X_H][MT9V03X_W])
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
 * @param points  输出路径点数组，按 [x, y] 的形式保存每一步经过的坐标。
 * @param point_count 输入时为 points 数组的最大容量；输出时为实际记录到的点数量。
 *
 * @note 该函数本质上是“左手迷宫法/跟墙法”，用于从某个白色起点沿白边线持续搜索并记录路径。
 */
void findline_lefthand_binary(const uint8 image_binary[MT9V03X_H][MT9V03X_W],int16 start_x,int16 start_y,int16 points[][2],uint16 * point_count)
{
    // 保存输出数组的最大容量。
    uint16 max_points;

    //保存当前搜索点的横坐标
    int16 x;
    int16 y;

    //保存已经找到的边线点数量
    uint6 step;

    //保存当前的行进方向

    //保存联系转弯次数

    int16 turn;
        // 检查输入图像、输出数组和数量指针是否有效。

    if(image_binary==0||(points==0)||(point_count)==0)
    {
        return ;
    }
    
    //读取调用者传入的数组容量
    max_points=* point_count;

    //先把输出的数组清0
    *point_count=0;

    //如果数组容量
    if(max_points==0)
    {
        return ;

    }


    //读取调用者的传入数组容量
    max_points=*point_count;

    *point_count=0;
    if(max_points==0)
    {
            //直接结束函数
        return ;
    }

    if(start_x<0||start_x>=(MT9V03X_))
}
