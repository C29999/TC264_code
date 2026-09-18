/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          cpu1_main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr.h"
#pragma section all "cpu1_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

// **************************** �������� ****************************

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

void core1_main(void)
{
    disable_Watchdog();                     
    interrupt_global_enable(0);             
    system_1_init();
    cpu_wait_event_ready();
    ips200_clear();
    show_boot_ani();
    camera_param_init();            // 初始化逆透视查找表（占位恒等表，标定后换真表）
    while (TRUE)
    {
        // 每 10ms 扫描一次按键，保证稳定周期且响应快
        static uint32 last_key_ms = 0;
        static uint32 last_display_ms = 0;
        static uint32 last_wifi_image_ms = 0;
        static uint32 last_wifi_data_ms = 0;
        const uint32 DISPLAY_REFRESH_MS = 100;
        const uint32 WIFI_IMAGE_PERIOD_MS = 200;
        const uint32 WIFI_DATA_PERIOD_MS = 20; // 50Hz control telemetry for oscillation analysis
        if (system_ms - last_key_ms >= 10)
        {
            last_key_ms = system_ms;
            key_scanner();
            diplay_key_control();
        }
        if (system_ms - last_display_ms >= DISPLAY_REFRESH_MS)
        {
            display_draw();
            last_display_ms = system_ms;
        }
        if (system_ms - last_wifi_data_ms >= WIFI_DATA_PERIOD_MS)
        {
            wifi_debug_data();
            last_wifi_data_ms = system_ms;
        }
        if (mt9v03x_finish_flag)
        {
            memcpy(image_gray, mt9v03x_image, sizeof(image_gray));
            __dsync();
            mt9v03x_finish_flag=0;
            fps_count++;
            image_threshold(image_gray); // Global Otsu preview from the stable frame.
            find_edges_binary();        // Maze tracing uses the displayed Otsu binary frame.
            process_edge_points();          // 3.边线点云处理
            calculation_error();            // 4.中线偏差
            image_frame_seq++;
            image_update_ms = system_ms;
            if (system_ms - last_wifi_image_ms >= WIFI_IMAGE_PERIOD_MS)
            {
                wifi_debug();
                last_wifi_image_ms = system_ms;
            }
        }
    }
}
#pragma section all restore
// **************************** �������� ****************************
