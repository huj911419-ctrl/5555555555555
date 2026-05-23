#include "TFT_show_image.h"
#include "Track_funsion.h"
#include "Function.h"
#include "IMU.h"
#include "Pid.h"

extern volatile uint32 prof_tf_us;
extern volatile uint32 prof_inter_us;


/* ================================================================
 *  显示逻辑：
 *    第一步：先把 Image_Binarize 显示到TFT上（黑白二值化的图像）
 *    第二步：在地图上画边界线和中线
 *    第三步：在右上角显示一些数值
 *
 *  效果：可以直接看到算法"看到"的情况（白色=赛道线，黑色=背景
 *        蓝色线=左边界  绿色线=右边界  红色线=中线
 * ================================================================ */

#define TFT_COL_SCALE_C   1.15f   // 94 -> 108, leave room for right status text
#define TFT_ROW_SCALE_C   1.35f   // 60 -> 81
#define TFT_IMG_DISP_W    108u
#define TFT_IMG_DISP_H    81u

#ifndef RGB565_YELLOW
#define RGB565_YELLOW   (0xFFE0u)
#endif
#ifndef RGB565_MAGENTA
#define RGB565_MAGENTA  (0xF81Fu)
#endif
#ifndef RGB565_CYAN
#define RGB565_CYAN     (0x07FFu)
#endif

#define IP_MARK_SIZE      3u      // half size of X mark on TFT

static uint8 tft_map_col(int16 col)
{
    int16 x = (int16)(TFT_COL_SCALE_C * (float)col + 0.5f);
    return (uint8)CLAMP(0, x, (int16)TFT_IMG_DISP_W - 1);
}

static uint8 tft_map_row(int16 row)
{
    int16 y = (int16)(TFT_ROW_SCALE_C * (float)row + 0.5f);
    return (uint8)CLAMP(0, y, (int16)TFT_IMG_DISP_H - 1);
}

static void tft_draw_x_mark(uint8 x, uint8 y, uint8 half_size, uint16 color)
{
    int16 i;
    for (i = -(int16)half_size; i <= (int16)half_size; i++)
    {
        int16 px1 = (int16)x + i;
        int16 py1 = (int16)y + i;
        int16 py2 = (int16)y - i;

        if (px1 >= 0 && px1 < (int16)TFT_IMG_DISP_W)
        {
            if (py1 >= 0 && py1 < (int16)TFT_IMG_DISP_H)
                tft180_draw_point((uint8)px1, (uint8)py1, color);
            if (py2 >= 0 && py2 < (int16)TFT_IMG_DISP_H)
                tft180_draw_point((uint8)px1, (uint8)py2, color);
        }
    }
}

static void tft_draw_rect_outline(uint8 left, uint8 top, uint8 right, uint8 bottom, uint16 color)
{
    uint8 c, r;

    if (left > right || top > bottom)
        return;

    for (c = left; c <= right; c++)
    {
        tft180_draw_point(c, top, color);
        tft180_draw_point(c, bottom, color);
    }
    for (r = top; r <= bottom; r++)
    {
        tft180_draw_point(left, r, color);
        tft180_draw_point(right, r, color);
    }
}

static void draw_inflection_overlay(void)
{
    uint8 has_box = 0u;

    if (g_ra_flag != 0u && g_inter_result.left_ip.valid)
    {
        tft_draw_x_mark(tft_map_col(g_inter_result.left_ip.col),
                        tft_map_row(g_inter_result.left_ip.row),
                        IP_MARK_SIZE, RGB565_YELLOW);
    }

    if (g_ra_flag != 0u && g_inter_result.right_ip.valid)
    {
        tft_draw_x_mark(tft_map_col(g_inter_result.right_ip.col),
                        tft_map_row(g_inter_result.right_ip.row),
                        IP_MARK_SIZE, RGB565_MAGENTA);
    }

    if (g_ra_flag != 0u &&
        (g_inter_result.left_ip.valid || g_inter_result.right_ip.valid))
    {
        if (g_inter_result.box_bottom > g_inter_result.box_top &&
            g_inter_result.box_right > g_inter_result.box_left)
        {
            has_box = 1u;
        }
    }

    if (has_box)
    {
        tft_draw_rect_outline(tft_map_col((int16)g_inter_result.box_left),
                              tft_map_row((int16)g_inter_result.box_top),
                              tft_map_col((int16)g_inter_result.box_right),
                              tft_map_row((int16)g_inter_result.box_bottom),
                              RGB565_CYAN);
    }
}

void draw_line(void)
{
    int32 yaw_deg = (yaw_angle >= 0.0f) ?
                    (int32)(yaw_angle + 0.5f) :
                    (int32)(yaw_angle - 0.5f);
    int32 ra_yaw_deg = (ra_dbg_yaw10 >= 0) ?
                       (int32)((ra_dbg_yaw10 + 5) / 10) :
                       (int32)((ra_dbg_yaw10 - 5) / 10);

    /* 第一步：显示二值化图像
     * Image_Binarize[i][j]=255 显示为白（赛道线）
     * Image_Binarize[i][j]=0   显示为黑（背景）
     * 阈值设1，255显示白，0显示黑 */
    tft180_show_gray_image(0, 0,
        Image_Binarize[0],
        COMP_W, COMP_H,
        TFT_IMG_DISP_W, TFT_IMG_DISP_H,
        1);

    /* 丢线时显示 LOST（仍叠加拐点与框） */
    if (g_tf.line_lost)
    {
        tft180_show_string(0, 0, "LOST");
    }

    /* 第二步：在地图上画边界线和中线 */
    if (!g_tf.line_lost)
    {
     for (uint8 i = TF_JIDIAN_ROW; i > TF_SEARCH_END_ROW; i--)
    {
        if (!g_tf.row_valid[i]) continue;

        uint8 tft_left  = (uint8)(TFT_COL_SCALE_C * (float)CLAMP(1, g_tf.left_edge[i],   COMP_W - 2));
        uint8 tft_right = (uint8)(TFT_COL_SCALE_C * (float)CLAMP(1, g_tf.right_edge[i],  COMP_W - 2));
        uint8 tft_mid   = (uint8)(TFT_COL_SCALE_C * (float)CLAMP(1, g_tf.center_line[i], COMP_W - 2));
        uint8 tft_row   = (uint8)(TFT_ROW_SCALE_C * (float)i);

        tft180_draw_point(tft_left,  tft_row, RGB565_BLUE);
        tft180_draw_point(tft_right, tft_row, RGB565_GREEN);
        tft180_draw_point(tft_mid,   tft_row, RGB565_RED);
    }
    }

    /* 拐点 × 与路口框（黄=左拐点 洋红=右拐点 青=框） */
    draw_inflection_overlay();

    /* 第三步：一些关键数值显示在地图下方（地图下方）*/
     tft180_show_string(0,  82, "ERR:");
    tft180_show_int   (36, 82, (int32)g_tf.error,          4);
    tft180_show_string(0,  98, "ROW:");
    tft180_show_int   (36, 98, (int32)g_tf.valid_row_count, 3);
    tft180_show_string(0, 114, "THR:");
    tft180_show_int(36, 114, (int32)Image_Threshold, 3);
    tft180_show_int(130,  0, motor_value.receive_left_speed_data, 3);
    tft180_show_int(130, 10, motor_value.receive_right_speed_data, 3);
    tft180_show_string(112, 20, "RA");
    tft180_show_int(130, 20, g_ra_flag, 3);
    tft180_show_string(112, 30, "Y:");
    tft180_show_int(130, 30, yaw_deg, 4);
    tft180_show_string(112, 40, "W:");
    tft180_show_int(130, 40, (int32)yaw_rate, 4);
    tft180_show_string(112, 50, "IP");
    tft180_show_int(130, 50, (int32)g_ip_max_row, 3);
    tft180_show_string(112, 60, "IM");
    tft180_show_int(130, 60, (int32)(imu_ready ? (imu_error ? 2 : 1) : 0), 1);
    tft180_show_string(112, 70, "ST");
    tft180_show_int(130, 70, (int32)ra_dbg_state, 1);
    tft180_show_string(112, 80, "PH");
    tft180_show_int(130, 80, (int32)ra_dbg_phase, 1);
    tft180_show_string(112, 90, "BS");
    tft180_show_int(130, 90, (int32)base_speed, 4);
    tft180_show_string(112, 100, "RY");
    tft180_show_int(130, 100, ra_yaw_deg, 4);
    tft180_show_string(112, 110, "TF");
    tft180_show_int(130, 110, (int32)prof_tf_us, 4);
    tft180_show_string(112, 120, "IN");
    tft180_show_int(130, 120, (int32)prof_inter_us, 4);
}
