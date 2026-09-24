#include "wifi_spi.h"
#include "image.h"
#include "data.h"

#include "math.h"
#include <stdio.h>
#define PAI 3.1415926f
#define AMPLITUDE 100
#define STEP 0.1f
int16 rx_speed;
int16 rx_change_flag;//是否开启上位机调参
static float sin_phase = 0.0f;
/* $CROSS 含 13 个带符号整数；48B 不足会覆盖相邻发送状态。 */
static char tx_buf[128];
static uint8 wifi_send_ready = 0;  
uint8 wifi_remote_ready(void)
{
    return wifi_send_ready;
}
static int SinWithPhase(float phase_offset)
{
    return (int)(AMPLITUDE * sin(sin_phase + phase_offset));
}
void rx_change(void)
{
    if (rx_change_flag)
    {
        base_speed = rx_speed;
    }

}
int Sin(void)
{
    int value;

    value = SinWithPhase(0.0f);

    sin_phase += STEP;
    if (sin_phase >= 2.0f * PAI)
    {
        sin_phase -= 2.0f * PAI;
    }

    return value;
}

void my_wifi_spi_init(void)
{
    if (wifi_spi_init(NULL, NULL) != 0)
    {
        show_center("WIFI SPI Error");
        system_delay_ms(500);
        return;
    }
    else
    {
        show_center("WIFI SPI Success");
        system_delay_ms(500);
    }
    if (wifi_spi_wifi_connect("C511", "c510c510") == 0)
        {
            show_center("WIFI Connected Success");
            system_delay_ms(500); 
        }
        else
        {
            show_center("WIFI Connecting fail");
            system_delay_ms(500);
            return;
        }
    system_delay_ms(200);
    if (wifi_spi_socket_connect("TCP", "192.168.0.103", "8080", "6060") != 0)
    {
        show_center("TCP Connecting fail");
        system_delay_ms(500);
        return;
    }
    else
    {
        show_center("TCP Connecting success");
        system_delay_ms(500);
    }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    wifi_send_ready = 1;    //全部初始化成功，允许发数据
}
/* 1-bit WiFi 图像包缓冲；与 CPU1 图像处理顺序复用，不新增全帧缓冲。 */
uint8 wifi_scratch_buf[(MT9V03X_W * MT9V03X_H) / 8];

void wifi_image_send(void)
{
    static seekfree_assistant_camera_struct camera_obj;
    uint32 pixel_index;
    if (!wifi_send_ready || !wifi_flag)
    {
        return;
    }
    memset(wifi_scratch_buf, 0, sizeof(wifi_scratch_buf));
    for (pixel_index = 0; pixel_index < MT9V03X_W * MT9V03X_H; pixel_index++)
    {
        if (((uint8 *)image_binary)[pixel_index] >= EDGE_WHITE_THRESHOLD)
        {
            wifi_scratch_buf[pixel_index >> 3] |= (uint8)(0x80u >> (pixel_index & 7));
        }
    }
    seekfree_assistant_camera_config(&camera_obj,
        SEEKFREE_ASSISTANT_CAMERA_TYPE_BINARY, MT9V03X_W, MT9V03X_H, wifi_scratch_buf);
    seekfree_assistant_camera_send(&camera_obj);
}
void wifi_boundary_send(void)
{
    static seekfree_assistant_camera_boundary_struct boundary_l_obj;
    static seekfree_assistant_camera_boundary_struct boundary_r_obj;
    static seekfree_assistant_camera_boundary_struct center_obj;
    static seekfree_assistant_camera_boundary_struct lookahead_obj;
    static seekfree_assistant_camera_boundary_struct corner_near_obj;
    static seekfree_assistant_camera_boundary_struct corner_far_obj;
    static seekfree_assistant_camera_boundary_struct vline_l_obj;   /* 左补边线 NL-FL（黄） */
    static seekfree_assistant_camera_boundary_struct vline_r_obj;   /* 右补边线 NR-FR（黄） */
    static seekfree_assistant_camera_boundary_struct vmid_obj;      /* 虚拟中线 M0-M1（品红） */
    uint8 center_points[MT9V03X_H][2];
    uint8 lookahead_point[1][2];
    uint8 corner_near[2][2];
    uint8 corner_far[2][2];
    uint8 vline_pts[2][2];     /* 补线两端点，config+send 后立即复用 */
    uint16 left_points_half[(IPTS_MAX + 1) / 2][2];
    uint16 right_points_half[(IPTS_MAX + 1) / 2][2];
    uint16 left_half_count = 0;
    uint16 right_half_count = 0;
    uint16 center_count = 0;
    uint16 near_count = 0;
    uint16 far_count = 0;
    int16 y;
    uint16 i;
    if (!wifi_send_ready || !wifi_flag)
    {
        return;
    }
    /* 左边线：红色 */
    if (left_line_count > 0)
    {
        for (i = 0; i < left_line_count; i += 2)
        {
            left_points_half[left_half_count][0] = (uint16)left_line_points[i][0];
            left_points_half[left_half_count][1] = (uint16)left_line_points[i][1];
            left_half_count++;
        }
        seekfree_assistant_camera_boundary_config(&boundary_l_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT16, 0xF800,
            left_half_count, left_points_half);
        seekfree_assistant_camera_boundary_send(&boundary_l_obj);
    }
    /* 右边线：蓝色 */
    if (right_line_count > 0)
    {
        for (i = 0; i < right_line_count; i += 2)
        {
            right_points_half[right_half_count][0] = (uint16)right_line_points[i][0];
            right_points_half[right_half_count][1] = (uint16)right_line_points[i][1];
            right_half_count++;
        }
        seekfree_assistant_camera_boundary_config(&boundary_r_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT16, 0x001F,
            right_half_count, right_points_half);
        seekfree_assistant_camera_boundary_send(&boundary_r_obj);
    }

    /* 双边真实中线：逐行配对，绿色。单边缺失的行不补线。
     * 十字期间（cross_flag != CR_NONE 且虚拟中线已生效），横路段（y < cv_m0_y）
     * 不发真实中线——那一段由橙色虚拟中线 M0-M1 接管，避免横路段边线乱配对。 */
    {
        int16 y_lo = (cross_flag >= CR_ENTER && cv_m0_y >= 0) ? cv_m0_y : 0;
        for (y = MT9V03X_H - 1; y >= y_lo; y--)
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
        if (lx >= 0 && rx >= 0 && lx < rx && center_count < MT9V03X_H)
        {
            center_points[center_count][0] = (uint8)((lx + rx) / 2);
            center_points[center_count][1] = (uint8)y;
            center_count++;
        }
    }
    }
    if (center_count > 0)
    {
        seekfree_assistant_camera_boundary_config(&center_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8, 0x07E0,
            center_count, center_points);
        seekfree_assistant_camera_boundary_send(&center_obj);
    }

    /* 动态前瞻点：黄色，单点边界帧。 */
    if (mid >= 0 && mid < MT9V03X_W && mid_y >= 0 && mid_y < MT9V03X_H)
    {
        lookahead_point[0][0] = (uint8)mid;
        lookahead_point[0][1] = (uint8)mid_y;
        seekfree_assistant_camera_boundary_config(&lookahead_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8, 0xFFE0,
            1, lookahead_point);
        seekfree_assistant_camera_boundary_send(&lookahead_obj);
    }

    /* 四个角点叠加到上位机二值图：近端紫色，远端黄色。
     * 补线保持期（cross_line_active）用锁存角点，车驶过近端后角点不消失。 */
    if (cross_line_active)
    {
        /* 状态机各阶段角点逐个生效：start 阶段只有近端，enter 后才有远端，
         * 无效角点不叠加（uint8 无法表示 -1，文本协议 $CORNERS 才透传 -1）。 */
        if (cross_hold_nl_x >= 0 && cross_hold_nl_y >= 0)
        {
            corner_near[near_count][0] = (uint8)clip(cross_hold_nl_x, 0, MT9V03X_W - 1);
            corner_near[near_count][1] = (uint8)clip(cross_hold_nl_y, 0, MT9V03X_H - 1);
            near_count++;
        }
        if (cross_hold_nr_x >= 0 && cross_hold_nr_y >= 0)
        {
            corner_near[near_count][0] = (uint8)clip(cross_hold_nr_x, 0, MT9V03X_W - 1);
            corner_near[near_count][1] = (uint8)clip(cross_hold_nr_y, 0, MT9V03X_H - 1);
            near_count++;
        }
        if (cross_hold_fl_x >= 0 && cross_hold_fl_y >= 0)
        {
            corner_far[far_count][0] = (uint8)clip(cross_hold_fl_x, 0, MT9V03X_W - 1);
            corner_far[far_count][1] = (uint8)clip(cross_hold_fl_y, 0, MT9V03X_H - 1);
            far_count++;
        }
        if (cross_hold_fr_x >= 0 && cross_hold_fr_y >= 0)
        {
            corner_far[far_count][0] = (uint8)clip(cross_hold_fr_x, 0, MT9V03X_W - 1);
            corner_far[far_count][1] = (uint8)clip(cross_hold_fr_y, 0, MT9V03X_H - 1);
            far_count++;
        }
    }
    else
    {
        if (Lpt0_found && Lpt0_rpts0s_id >= 0 && Lpt0_rpts0s_id < rpts0s_num)
        {
            corner_near[near_count][0] = (uint8)clip((int16)(rpts0s[Lpt0_rpts0s_id][0] * pixel_per_meter), 0, MT9V03X_W - 1);
            corner_near[near_count][1] = (uint8)clip((int16)(rpts0s[Lpt0_rpts0s_id][1] * pixel_per_meter), 0, MT9V03X_H - 1);
            near_count++;
        }
        if (Lpt1_found && Lpt1_rpts1s_id >= 0 && Lpt1_rpts1s_id < rpts1s_num)
        {
            corner_near[near_count][0] = (uint8)clip((int16)(rpts1s[Lpt1_rpts1s_id][0] * pixel_per_meter), 0, MT9V03X_W - 1);
            corner_near[near_count][1] = (uint8)clip((int16)(rpts1s[Lpt1_rpts1s_id][1] * pixel_per_meter), 0, MT9V03X_H - 1);
            near_count++;
        }
        if (far_Lpt0_found && far_Lpt0_rpts0s_id >= 0 && far_Lpt0_rpts0s_id < far_rpts0s_num)
        {
            corner_far[far_count][0] = (uint8)clip((int16)(far_rpts0s[far_Lpt0_rpts0s_id][0] * pixel_per_meter), 0, MT9V03X_W - 1);
            corner_far[far_count][1] = (uint8)clip((int16)(far_rpts0s[far_Lpt0_rpts0s_id][1] * pixel_per_meter), 0, MT9V03X_H - 1);
            far_count++;
        }
        if (far_Lpt1_found && far_Lpt1_rpts1s_id >= 0 && far_Lpt1_rpts1s_id < far_rpts1s_num)
        {
            corner_far[far_count][0] = (uint8)clip((int16)(far_rpts1s[far_Lpt1_rpts1s_id][0] * pixel_per_meter), 0, MT9V03X_W - 1);
            corner_far[far_count][1] = (uint8)clip((int16)(far_rpts1s[far_Lpt1_rpts1s_id][1] * pixel_per_meter), 0, MT9V03X_H - 1);
            far_count++;
        }
    }
    if (near_count > 0)
    {
        seekfree_assistant_camera_boundary_config(&corner_near_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8, 0xF81F, near_count, corner_near);
        seekfree_assistant_camera_boundary_send(&corner_near_obj);
    }
    if (far_count > 0)
    {
        seekfree_assistant_camera_boundary_config(&corner_far_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8, 0x07FF, far_count, corner_far);
        seekfree_assistant_camera_boundary_send(&corner_far_obj);
    }

    /* 四角点虚拟补线（cross_build_vlines）：左 NL-FL 紫 / 右 NR-FR 靛蓝补边线
     * （左右必须异色，同色会被上位机同色通道后发覆盖），橙色 M0-M1 补中线。 */
    if (cross_vline_valid && cross_flag >= CR_ENTER)
    {
        vline_pts[0][0] = (uint8)clip(cv_nl_x, 0, MT9V03X_W - 1);
        vline_pts[0][1] = (uint8)clip(cv_nl_y, 0, MT9V03X_H - 1);
        vline_pts[1][0] = (uint8)clip(cv_fl_x, 0, MT9V03X_W - 1);
        vline_pts[1][1] = (uint8)clip(cv_fl_y, 0, MT9V03X_H - 1);
        seekfree_assistant_camera_boundary_config(&vline_l_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8, 0x801F, 2, vline_pts);
        seekfree_assistant_camera_boundary_send(&vline_l_obj);

        vline_pts[0][0] = (uint8)clip(cv_nr_x, 0, MT9V03X_W - 1);
        vline_pts[0][1] = (uint8)clip(cv_nr_y, 0, MT9V03X_H - 1);
        vline_pts[1][0] = (uint8)clip(cv_fr_x, 0, MT9V03X_W - 1);
        vline_pts[1][1] = (uint8)clip(cv_fr_y, 0, MT9V03X_H - 1);
        seekfree_assistant_camera_boundary_config(&vline_r_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8, 0x041F, 2, vline_pts);
        seekfree_assistant_camera_boundary_send(&vline_r_obj);

        vline_pts[0][0] = (uint8)clip(cv_m0_x, 0, MT9V03X_W - 1);
        vline_pts[0][1] = (uint8)clip(cv_m0_y, 0, MT9V03X_H - 1);
        vline_pts[1][0] = (uint8)clip(cv_m1_x, 0, MT9V03X_W - 1);
        vline_pts[1][1] = (uint8)clip(cv_m1_y, 0, MT9V03X_H - 1);
        seekfree_assistant_camera_boundary_config(&vmid_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT8, 0xFC00, 2, vline_pts);
        seekfree_assistant_camera_boundary_send(&vmid_obj);
    }
}
void wifi_debug_data(void)
{
    static seekfree_assistant_oscilloscope_struct scope_obj;
    static float scope_data[9];
    static uint8 scope_configured = 0;

    if (!wifi_send_ready || !wifi_flag)
    {
        return;
    }
    if (!scope_configured)
    {
        /* CH1 车速；CH2~CH9 为四个角点的 x/y 像素坐标。无效点为 -1。 */
        seekfree_assistant_oscilloscope_config(&scope_obj, 9, scope_data);
        scope_configured = 1;
    }
    /* CH1：两轮编码器当前速度的平均值（编码器计数/10ms）。 */
    scope_data[0] = ((float)encoder_left + (float)encoder_right) * 0.5f;
    scope_data[1] = -1.0f; scope_data[2] = -1.0f;
    scope_data[3] = -1.0f; scope_data[4] = -1.0f;
    scope_data[5] = -1.0f; scope_data[6] = -1.0f;
    scope_data[7] = -1.0f; scope_data[8] = -1.0f;
    if (Lpt0_found && Lpt0_rpts0s_id >= 0 && Lpt0_rpts0s_id < rpts0s_num)
    {
        scope_data[1] = rpts0s[Lpt0_rpts0s_id][0] * pixel_per_meter;
        scope_data[2] = rpts0s[Lpt0_rpts0s_id][1] * pixel_per_meter;
    }
    if (Lpt1_found && Lpt1_rpts1s_id >= 0 && Lpt1_rpts1s_id < rpts1s_num)
    {
        scope_data[3] = rpts1s[Lpt1_rpts1s_id][0] * pixel_per_meter;
        scope_data[4] = rpts1s[Lpt1_rpts1s_id][1] * pixel_per_meter;
    }
    if (far_Lpt0_found && far_Lpt0_rpts0s_id >= 0 && far_Lpt0_rpts0s_id < far_rpts0s_num)
    {
        scope_data[5] = far_orig0[far_Lpt0_rpts0s_id][0];
        scope_data[6] = far_orig0[far_Lpt0_rpts0s_id][1];
    }
    if (far_Lpt1_found && far_Lpt1_rpts1s_id >= 0 && far_Lpt1_rpts1s_id < far_rpts1s_num)
    {
        scope_data[7] = far_orig1[far_Lpt1_rpts1s_id][0];
        scope_data[8] = far_orig1[far_Lpt1_rpts1s_id][1];
    }
    seekfree_assistant_oscilloscope_send(&scope_obj);
}

/* ================ 角点文本发送：$CORNERS NLx,NLy,NRx,NRy,FLx,FLy,FRx,FRy\r\n ================
 * 原图 188x120 像素坐标，可直接叠加到上位机二值图上。
 * 无效角点输出 -1。与 wifi_boundary_send 的近端紫/远端黄角点同源（Lpt0/Lpt1/far_Lpt0/far_Lpt1）。 */
void wifi_corner_send(void)
{
    static char corner_buf[96];
    int16 nl_x = -1, nl_y = -1, nr_x = -1, nr_y = -1;
    int16 fl_x = -1, fl_y = -1, fr_x = -1, fr_y = -1;
    if (!wifi_send_ready || !wifi_flag)
    {
        return;
    }
    if (cross_line_active)
    {
        /* 状态机锁存角点（原图像素）；尚未扫到的角点保持 -1（协议规定无效输出 -1） */
        if (cross_hold_nl_x >= 0) nl_x = clip(cross_hold_nl_x, 0, MT9V03X_W - 1);
        if (cross_hold_nl_y >= 0) nl_y = clip(cross_hold_nl_y, 0, MT9V03X_H - 1);
        if (cross_hold_nr_x >= 0) nr_x = clip(cross_hold_nr_x, 0, MT9V03X_W - 1);
        if (cross_hold_nr_y >= 0) nr_y = clip(cross_hold_nr_y, 0, MT9V03X_H - 1);
        if (cross_hold_fl_x >= 0) fl_x = clip(cross_hold_fl_x, 0, MT9V03X_W - 1);
        if (cross_hold_fl_y >= 0) fl_y = clip(cross_hold_fl_y, 0, MT9V03X_H - 1);
        if (cross_hold_fr_x >= 0) fr_x = clip(cross_hold_fr_x, 0, MT9V03X_W - 1);
        if (cross_hold_fr_y >= 0) fr_y = clip(cross_hold_fr_y, 0, MT9V03X_H - 1);
    }
    else
    {
        if (Lpt0_found && Lpt0_rpts0s_id >= 0 && Lpt0_rpts0s_id < rpts0s_num)
        {
            nl_x = clip((int16)(rpts0s[Lpt0_rpts0s_id][0] * pixel_per_meter), 0, MT9V03X_W - 1);
            nl_y = clip((int16)(rpts0s[Lpt0_rpts0s_id][1] * pixel_per_meter), 0, MT9V03X_H - 1);
        }
        if (Lpt1_found && Lpt1_rpts1s_id >= 0 && Lpt1_rpts1s_id < rpts1s_num)
        {
            nr_x = clip((int16)(rpts1s[Lpt1_rpts1s_id][0] * pixel_per_meter), 0, MT9V03X_W - 1);
            nr_y = clip((int16)(rpts1s[Lpt1_rpts1s_id][1] * pixel_per_meter), 0, MT9V03X_H - 1);
        }
        if (far_Lpt0_found && far_Lpt0_rpts0s_id >= 0 && far_Lpt0_rpts0s_id < far_rpts0s_num)
        {
            fl_x = clip((int16)(far_rpts0s[far_Lpt0_rpts0s_id][0] * pixel_per_meter), 0, MT9V03X_W - 1);
            fl_y = clip((int16)(far_rpts0s[far_Lpt0_rpts0s_id][1] * pixel_per_meter), 0, MT9V03X_H - 1);
        }
        if (far_Lpt1_found && far_Lpt1_rpts1s_id >= 0 && far_Lpt1_rpts1s_id < far_rpts1s_num)
        {
            fr_x = clip((int16)(far_rpts1s[far_Lpt1_rpts1s_id][0] * pixel_per_meter), 0, MT9V03X_W - 1);
            fr_y = clip((int16)(far_rpts1s[far_Lpt1_rpts1s_id][1] * pixel_per_meter), 0, MT9V03X_H - 1);
        }
    }
    sprintf(corner_buf, "$CORNERS %d,%d,%d,%d,%d,%d,%d,%d\r\n",
            nl_x, nl_y, nr_x, nr_y, fl_x, fl_y, fr_x, fr_y);
    wifi_spi_send_string(corner_buf);
    /* 十字状态与候选判据。TMODE 由真实状态驱动，避免上位机固定显示普通巡线。 */
    sprintf(tx_buf, "$TMODE %d\r\n", (int)((cross_flag >= CR_ENTER) ? 2 : cross_flag));
    wifi_spi_send_string(tx_buf);
    sprintf(tx_buf, "$CROSS %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
            (int)cross_flag, (int)cross_candidate_frames,
            (int)cross_near_pair_valid, (int)cross_geometry_valid,
            (int)(touch_boundary0 != 0), (int)(touch_boundary1 != 0),
            nl_x, nl_y, nr_x, nr_y, (int)(cross_phase_distance * 1000.0f),
            (int)cross_exit_lane_valid, (int)cross_exit_confirm_frames);
    wifi_spi_send_string(tx_buf);
    /* 编码器原始值为每 10ms 脉冲数；第三项为发车以来平均速度 mm/s。 */
    {
        int32 average_speed_mmps = (int32)(avg_speed * 1000.0f);
        sprintf(tx_buf, "$SPD %d,%d,%ld\r\n", (int)encoder_left,
                (int)encoder_right, (long)average_speed_mmps);
        wifi_spi_send_string(tx_buf);
    }
    /* 附加调试量：$DBG fps image_error_filter */
    {
        static char dbg_buf[48];
        sprintf(dbg_buf, "$DBG %d %d\r\n", (int)fps, (int)image_error_filter);
        wifi_spi_send_string(dbg_buf);
    }
    /* 普通巡线诊断：起始行、入口左右坐标、原始点数、左右触界。 */
    {
        static char track_dbg_buf[80];
        sprintf(track_dbg_buf, "$TRACK %d %d %d %u %u %d %d\r\n",
                (int)maze_start_y, (int)maze_start_left_x,
                (int)maze_start_right_x, (unsigned int)left_line_count,
                (unsigned int)right_line_count, (int)(touch_boundary0 != 0),
                (int)(touch_boundary1 != 0));
        wifi_spi_send_string(track_dbg_buf);
    }
    wifi_crossline_send();
}

/* ================ 十字中线文本发送：$CLINE sx,sy,ex,ey\r\n ================
 * enter/out 状态下，车底参考点 → 对面路口中心(cross_max_x,cross_far_y)
 * 的导航中线（与 calculation_error 十字分支的 pure pursuit 目标同源）。
 * 原图 188x120 像素坐标；非十字中或扫描无效时四个值均为 -1。
 *   sx,sy = 中线近端点（车底参考点，图像中列、底部上 10 行）
 *   ex,ey = 中线远端点（对面竖路路口中心） */
void wifi_crossline_send(void)
{
    static char cline_buf[48];
    int16 sx = -1, sy = -1, ex = -1, ey = -1;
    if (wifi_send_ready && wifi_flag
        && cross_flag >= CR_ENTER && cross_far_y > 0
        && cross_max_x >= 0 && cross_max_x < MT9V03X_W)
    {
        sx = MT9V03X_W / 2;          /* 车底参考点，与 calculation_error 的 cx/cy 一致 */
        sy = MT9V03X_H - 10;
        ex = cross_max_x;
        ey = cross_far_y;
    }
    sprintf(cline_buf, "$CLINE %d,%d,%d,%d\r\n", sx, sy, ex, ey);
    wifi_spi_send_string(cline_buf);
}

void wifi_debug(void)
{
    /* 二值图打包仅 2.8KB/帧，每帧都发，由主循环 WIFI_IMAGE_PERIOD_MS=50 限频到 20fps。
     * 先发送本帧二值图，再发送同一帧的边界和角点，避免上位机叠加上一帧坐标。 */
    wifi_image_send();
    wifi_boundary_send();
    wifi_corner_send();
}

/* ================ WiFi 接收解析：$SPEED num1,num2 ================ */
static uint8 wifi_parse_buf[128];
static uint16 wifi_parse_len = 0;
typedef enum {
    WIFI_PARSE_WAIT_HEADER,
    WIFI_PARSE_GET_NUM1,
    WIFI_PARSE_GET_NUM2,
} wifi_parse_state_t;

// 解析结果（外部可读取）
int16 wifi_cmd_speed = 0;    // $SPEED 后的第一个数字
int16 wifi_cmd_param2 = 0;   // $SPEED 后的第二个数字
uint8 wifi_cmd_flag = 0;     // 收到新指令时置1，处理完清0
volatile uint8 wifi_go_flag = 0;
volatile uint8 wifi_stop_flag = 0;
void wifi_task(void)
{
    uint8 rx_buf[64];
    uint32 rx_len;
    uint8 ch;
    static wifi_parse_state_t state = WIFI_PARSE_WAIT_HEADER;
    static char num_buf[16];
    static uint8 num_idx = 0;
    static int16 num1 = 0;

    // 1. 从 WiFi FIFO 读数据
    rx_len = wifi_spi_read_buffer(rx_buf, sizeof(rx_buf));
    if (rx_len == 0) return;

    // 2. 追加到解析缓冲
    if (wifi_parse_len + rx_len >= sizeof(wifi_parse_buf))
    {
        wifi_parse_len = 0;  // 溢出清空
        return;
    }
    memcpy(&wifi_parse_buf[wifi_parse_len], rx_buf, rx_len);
    wifi_parse_len += rx_len;
    wifi_parse_buf[wifi_parse_len] = '\0';

    /* 动态巡线速度：$TRACKSPD -220,-180\r\n（直道、弯道；前进方向为负）。 */
    {
        uint16 speed_i;
        for (speed_i = 0; speed_i + 10 < wifi_parse_len; speed_i++)
        {
            int straight_value, corner_value;
            if (wifi_parse_buf[speed_i] == '$'
                && memcmp(&wifi_parse_buf[speed_i], "$TRACKSPD", 9) == 0
                && sscanf((char *)&wifi_parse_buf[speed_i + 9], "%d,%d",
                          &straight_value, &corner_value) == 2)
            {
                straight_speed = clip((int16)straight_value, -500, -10);
                corner_speed = clip((int16)corner_value, -500, -10);
                wifi_parse_len = 0;
                return;
            }
        }
    }

    // 遥控发车/停车命令。使用累计缓冲，可处理 TCP 拆包。
    if (wifi_parse_len >= 3)
    {
        uint16 go_i;
        for (go_i = 0; go_i + 2 < wifi_parse_len; go_i++)
        {
            if (wifi_parse_buf[go_i] == '$' &&
                wifi_parse_buf[go_i + 1] == 'G' &&
                wifi_parse_buf[go_i + 2] == 'O')
            {
                wifi_go_flag = 1;
                wifi_parse_len = 0;
                return;
            }
            if (go_i + 4 < wifi_parse_len && wifi_parse_buf[go_i] == '$'
                && wifi_parse_buf[go_i + 1] == 'S'
                && wifi_parse_buf[go_i + 2] == 'T'
                && wifi_parse_buf[go_i + 3] == 'O'
                && wifi_parse_buf[go_i + 4] == 'P')
            {
                wifi_stop_flag = 1;
                wifi_parse_len = 0;
                return;
            }
        }
    }

    // 3. 逐字节解析
    uint16 i;
    for (i = 0; i < wifi_parse_len; i++)
    {
        ch = wifi_parse_buf[i];

        switch (state)
        {
        case WIFI_PARSE_WAIT_HEADER:
            // 找 "$SPEED "（7字节）
            if (wifi_parse_len - i >= 7 &&
                memcmp(&wifi_parse_buf[i], "$SPEED ", 7) == 0)
            {
                i += 6;  // 跳过 "$SPEED"，停在空格，下次循环 i++ 跳过空格
                state = WIFI_PARSE_GET_NUM1;
                num_idx = 0;
            }
            break;

        case WIFI_PARSE_GET_NUM1:
            if ((ch >= '0' && ch <= '9') || ch == '-')
            {
                if (num_idx < 15) num_buf[num_idx++] = ch;
            }
            else if (ch == ',')
            {
                num_buf[num_idx] = '\0';
                num1 = (int16)atoi(num_buf);
                state = WIFI_PARSE_GET_NUM2;
                num_idx = 0;
            }
            else
            {
                state = WIFI_PARSE_WAIT_HEADER;  // 格式错，重来
            }
            break;

        case WIFI_PARSE_GET_NUM2:
            if ((ch >= '0' && ch <= '9') || ch == '-')
            {
                if (num_idx < 15) num_buf[num_idx++] = ch;
            }
            else if (ch == '\n' || ch == '\r' || ch == ' ')
            {
                num_buf[num_idx] = '\0';
                wifi_cmd_speed = num1;
                wifi_cmd_param2 = (int16)atoi(num_buf);
                wifi_cmd_flag = 1;  // 通知主循环有新指令
                state = WIFI_PARSE_WAIT_HEADER;
            }
            else
            {
                state = WIFI_PARSE_WAIT_HEADER;
            }
            break;
        }
    }
    if (i > 0 && i <= wifi_parse_len)
    {
        memmove(wifi_parse_buf, &wifi_parse_buf[i], wifi_parse_len - i);
        wifi_parse_len -= i;
    }
}
