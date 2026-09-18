#include "wifi_spi.h"
#include "isr.h"

#include "math.h"
#include <stdio.h>
#define PAI 3.1415926f
#define AMPLITUDE 100
#define STEP 0.1f
int16 rx_speed;
int16 rx_change_flag;//是否开启上位机调参
static float sin_phase = 0.0f;
static char tx_buf[48] __attribute__((unused));
static uint8 wifi_send_ready = 0;  
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
    if (wifi_spi_socket_connect("TCP", "192.168.0.111", "8080", "6060") != 0)
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
void wifi_image_send(void)
{
    static seekfree_assistant_camera_struct camera_obj;
    static uint8 camera_configured = 0;
    if (!wifi_send_ready || !wifi_flag)
    {
        return;
    }
    /* 全图：尺寸和缓冲固定，只需配置一次 */
    if (!camera_configured)
    {
        seekfree_assistant_camera_config(&camera_obj,
            SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X, MT9V03X_W, MT9V03X_H, image_gray);
        camera_configured = 1;
    }
    seekfree_assistant_camera_send(&camera_obj);
}
void wifi_boundary_send(void)
{
    static seekfree_assistant_camera_boundary_struct boundary_l_obj;
    static seekfree_assistant_camera_boundary_struct boundary_r_obj;
    if (!wifi_send_ready || !wifi_flag)
    {
        return;
    }
    /* 左边线：红色 */
    if (left_line_count > 0)
    {
        seekfree_assistant_camera_boundary_config(&boundary_l_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT16, 0xF800,
            (uint16)left_line_count, left_line_points);
        seekfree_assistant_camera_boundary_send(&boundary_l_obj);
    }
    /* 右边线：蓝色 */
    if (right_line_count > 0)
    {
        seekfree_assistant_camera_boundary_config(&boundary_r_obj,
            SEEKFREE_ASSISTANT_DATA_TYPE_UINT16, 0x001F,
            (uint16)right_line_count, right_line_points);
        seekfree_assistant_camera_boundary_send(&boundary_r_obj);
    }
}
void wifi_debug_data(void)
{
    if (!wifi_send_ready) return;
    char buf[384];
    int16 pure_x100 = (int16)(pure_angle * 100.0f);
    int16 servo_x100 = (int16)(angle * 100.0f);
    int16 gyro_x10 = (int16)(gyro_z * 10.0f);
    int16 p_x100 = (int16)(servo_pid.out_p * 100.0f);
    int16 d_x100 = (int16)(servo_pid.kd * servo_pid.out_d * 100.0f);
    int16 raw_angle_x100 = (int16)(trace_raw_angle * 100.0f);
    int16 raw_mid_x100 = (int16)(trace_mid_raw_px * 100.0f);
    int16 corner_turn_x1000 = (int16)(corner_turn * 1000.0f);
    int16 pid_raw_x100 = (int16)(servo_pid_raw * 100.0f);
    int16 gyro_term_x100 = (int16)(-gyro_z * servo_pid.kgyro * 100.0f);
    int16 control_error_x100 = (int16)(control_error * 100.0f);
    uint32 image_age_ms = system_ms - image_update_ms;
    sprintf(buf,
            "$TRACE,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%d,%u,%u"
            ",%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d\r\n",
            (unsigned long)system_ms,
            pure_x100, servo_x100, gyro_x10, p_x100, d_x100,
            motor_goal_left, motor_goal_right,
            encoder_left, encoder_right,
            motor_pwm_left, motor_pwm_right, motor_diff,
            (unsigned int)left_line_count, (unsigned int)right_line_count,
            (unsigned int)state_flags, mid,
            (unsigned int)fps, (unsigned int)stop_flog,
            (unsigned int)image_frame_seq, (unsigned long)image_age_ms,
            raw_angle_x100, raw_mid_x100, corner_turn_x1000,
            pid_raw_x100, gyro_term_x100, motor_base_goal,
            trace_l_target_pts, trace_r_target_pts,
            trace_l_min_mm, trace_r_min_mm,
            (unsigned int)track_stop_count, (unsigned int)track_invalid_count,
            control_error_x100);
    wifi_spi_send_string(buf);
}

void wifi_debug(void)
{
    wifi_boundary_send();
    wifi_image_send();
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
    if (wifi_parse_len + rx_len > sizeof(wifi_parse_buf))
    {
        wifi_parse_len = 0;  // 溢出清空
        return;
    }
    memcpy(&wifi_parse_buf[wifi_parse_len], rx_buf, rx_len);
    wifi_parse_len += rx_len;

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
