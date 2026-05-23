#ifndef _TFT_SHOW_IMAGE_H_
#define _TFT_SHOW_IMAGE_H_

#include "zf_common_headfile.h"
/**
 * @brief 显示二值化底图 + 叠加边界线
 *        必须在 track_fusion_update() 之后调用
 *
 *        屏幕布局：
 *          y=0  ~ y=80  : 二值化图像（白=赛道线 黑=背景）
 *                          蓝点=左边界 绿点=右边界 红点=中线
 *                          黄×=左拐点 洋红×=右拐点 青框=路口框
 *          y=82 ~ y=127 : ERR / ROW / THR 数值
 */
void draw_line(void);


#endif
