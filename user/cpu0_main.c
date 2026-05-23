#include "zf_common_headfile.h"
#include "IMU.h"
#include "Menu.h"

extern int16 cam_exposure;

volatile uint32 prof_tf_us = 0u;
volatile uint32 prof_inter_us = 0u;

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    /* 初始化系统时钟 */
    clock_init();

    /* 初始化调试串口 */
    debug_init();

    /* 配置显示字体和方向 */
#if !RACE_MODE
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();
#endif

    /* 初始化摄像头 */
    mt9v03x_init();
    mt9v03x_set_exposure_time((uint16)cam_exposure);
    //gpio_init(P20_8, GPO, 0, GPO_PUSH_PULL);
    //gpio_init(P20_9, GPO, 0, GPO_PUSH_PULL);
    //gpio_init(P23_1, GPO, 0, GPO_PUSH_PULL);
    //gpio_init(P22_0, GPO, 0, GPO_PUSH_PULL);
   // gpio_init(P33_4, GPO, 1, GPO_PUSH_PULL);
   // gpio_init(P33_5, GPO, 1, GPO_PUSH_PULL);
   // gpio_init(P33_6, GPO, 1, GPO_PUSH_PULL);


    /* 初始化赛道融合检测 */
    track_fusion_init();

    /* 初始化小车主控串口通讯 */
   small_driver_uart_init();

    /* 初始化增量式PID参数 */
    line_pid_init();

    /* 初始化IMU角度测量单元 */
    //imu_init();

    /* 初始化11ms周期中断（PID在ISR中直接调用） */
   pit_ms_init(CCU60_CH0, 11);

    /* 初始化按键 */
    key_init_all();

    /* 等待所有核事件就绪 */
    cpu_wait_event_ready();

    while (TRUE)
    {
        /* 帧同步 */
        while (!mt9v03x_finish_flag);
        mt9v03x_finish_flag = 0;

        /* 更新赛道融合检测结果 */
        system_start();
        track_fusion_update();
        prof_tf_us = system_getval_us();

        /* 直角预判检测（远距离，必须在 track_fusion_update 之后） */
        system_start();
        right_angle_pre_detect();

        /* 拐点画框路口检测（替代原 right_angle_detect） */
        detect_intersection();
        prof_inter_us = system_getval_us();
        

        /* 按键处理 */
        key_process();

        /* 显示菜单 */
        menu_show();
    }
}

#pragma section all restore
