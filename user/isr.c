#include "isr_config.h"
#include "isr.h"

#include "zf_device_small_driver_uart_control.h"
#include "IMU.h"
#include "Pid.h"

/* ========================================================================
 * PIT(定时器)中断服务程序
 * ======================================================================== */

/* CCU60 通道0 PIT中断 - 11ms周期
 * 功能: PID控制计算（直接在ISR中执行） */
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    line_pid_control();
}

/* CCU60 通道1 PIT中断 - 5ms周期
 * 功能: IMU数据更新 */
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    imu_update();
}

/* CCU61 通道0 PIT中断 - 备用 */
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
}

/* CCU61 通道1 PIT中断 - 备用 */
IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH1);
}

/* ========================================================================
 * EXTI(外部中断)服务程序
 * ======================================================================== */

/* 外部中断通道0和4 - 摄像头VSYNC(场同步)信号
 * 功能: 摄像头帧同步检测
 * 引脚: P15_4(通道0), P15_5(通道4) */
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

/* 外部中断通道1和5 - 预留
 * 功能: ToF(激光测距)模块中断
 * 引脚: P14_3(通道1), P15_8(通道5) */
IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
        tof_module_exti_handler();
    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
    }
}

/* 外部中断通道3和7 - 摄像头DMA完成
 * 功能: 摄像头VSYNC同步和DMA传输完成
 * 引脚: P02_0(通道3), P15_1(通道7) */
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}

/* ========================================================================
 * DMA中断服务程序
 * ======================================================================== */

/* DMA通道5中断 - 摄像头数据接收完成
 * 功能: 摄像头图像 DMA 传输完成 */
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_dma_handler();
}

/* ========================================================================
 * UART中断服务程序
 * ======================================================================== */

/* UART0 发送中断 - 调试串口发送(预留) */
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

/* UART0 接收中断 - 调试串口接收
 * 功能: 接收调试命令 */
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);

#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif
}

/* UART1 发送中断 - 摄像头发送(预留) */
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

/* UART1 接收中断 - 摄像头图像接收
 * 功能: 接收摄像头数据 */
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_uart_handler();
}

/* UART2 发送中断 - 无线模块发送(预留) */
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

/* UART2 接收中断 - 无线模块接收
 * 功能: 无线遥控数据接收 */
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    wireless_module_uart_handler();
}

/* UART3 发送中断 - 从车通信发送(预留) */
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

/* UART3 接收中断 - 从车速度接收
 * 功能: 接收从车返回的速度数据 */
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    uart_control_callback();
}

/* ========================================================================
 * UART错误中断服务程序
 * ======================================================================== */

/* UART0 错误中断 */
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart0_handle);
}

/* UART1 错误中断 */
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart1_handle);
}

/* UART2 错误中断 */
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart2_handle);
}

/* UART3 错误中断 */
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart3_handle);
}
