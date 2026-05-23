#ifndef TRACK_FUSION_H
#define TRACK_FUSION_H

#include "zf_common_headfile.h"
#include "App_Config.h"

extern int16 threshold_bias;

/* Image compression: 188x120 -> 94x60. */
#define COMP_W          (MT9V03X_W / 2)
#define COMP_H          (MT9V03X_H / 2)
#define COMP_CENTER     (COMP_W / 2)

#define TF_IMG_H        COMP_H
#define TF_IMG_W        COMP_W
#define TF_IMG_CENTER   COMP_CENTER

/* Track extraction parameters. */
#ifndef TF_OTSU_INTERVAL
#if RACE_MODE
#define TF_OTSU_INTERVAL    TF_OTSU_INTERVAL_RACE
#else
#define TF_OTSU_INTERVAL    TF_OTSU_INTERVAL_DEBUG
#endif
#endif
#define TF_JIDIAN_ROW       (TF_IMG_H - 4)
#define TF_JIDIAN_MIN_ROW   (TF_IMG_H - 15)
#define TF_SEARCH_END_ROW   4
#define TF_LOCAL_RANGE      10
#define TF_MAX_MISS_ROWS    5
#define TF_MIN_TRACK_WIDTH  4
#define TF_MAX_TRACK_WIDTH  30
#define TRACK_HALF_WIDTH    8
#define TRACK_HALF_WIDTH_MIN 4
#define TRACK_HALF_WIDTH_MAX 14
#define TF_INVALID          (-1)
#define TF_OTSU_MIN_THRESHOLD 75
#define TF_DENOISE_WHITE_MIN 4
#define TF_DENOISE_BLACK_FILL 7

/* Lookahead rows. */
#define TF_LOOKAHEAD_START_ROW 35
#define TF_LOOKAHEAD_END_ROW   20

#define Image_WHITE 255u
#define Image_BLACK 0u

typedef struct
{
    int16 left_edge[TF_IMG_H];
    int16 right_edge[TF_IMG_H];
    int16 center_line[TF_IMG_H];
    uint8 row_valid[TF_IMG_H];

    int16 error;
    int16 lookahead_error;
    int16 error_trend;
    uint16 valid_row_count;
    uint8 line_lost;

    uint8 left_jidian;
    uint8 right_jidian;
} TrackFusion_t;

extern TrackFusion_t g_tf;
extern uint8 Image_Binarize[TF_IMG_H][TF_IMG_W];
extern uint8 image_0[COMP_H][COMP_W];
extern uint16 Image_Threshold;

/* Intersection detection parameters. */
#define INTER_COOLDOWN_FRAMES  12
#define INTER_MAX_LOCK_FRAMES  300
#define INTER_BOX_START_ROW    34
#define INTER_MIN_MISS_ROW     38
#define INTER_MIN_VALID_ROWS   22
#define BOX_HEIGHT             20
#define BOX_WIDTH              40
#define INTER_BOX_WIDTH_MIN    28
#define INTER_BOX_WIDTH_MAX    44
#define INTER_BOX_WIDTH_SCALE  3
#define INTER_BOX_HEIGHT_MIN   16
#define INTER_BOX_HEIGHT_MAX   24
#define INTER_BOX_HEIGHT_SCALE 2
#define INTER_BAND_THICKNESS   3
#define INTER_TOP_MIN_WHITE    8
#define INTER_SIDE_MIN_WHITE   6
#define INTER_BAND_MIN_STREAK  4
#define INTER_BRANCH_MIN_STREAK 12
#define INTER_BOX_STABLE_DELTA 3
#define INTER_BOX_STABLE_FRAMES 4
#define INTER_TYPE_VOTE_FRAMES 3
#define INTER_TYPE_VOTE_MIN    2
#define INTER_FAST_CONFIRM_ENABLE 0
#define INTER_IP_SIDE_BIAS     6
#define INTER_SYM_ROW_DELTA    3
#define INTER_SYM_CENTER_DELTA 8
#define INTER_SYM_BRANCH_STREAK 18
#define INTER_SYM_PRE_ROWS     2

typedef struct
{
    uint8 valid;
    int16 row;
    int16 col;
} InflectionPoint_t;

typedef struct
{
    InflectionPoint_t left_ip;
    InflectionPoint_t right_ip;
    uint8 box_top;
    uint8 box_bottom;
    uint8 box_left;
    uint8 box_right;
    uint8 detected_type;
} IntersectionResult_t;

/* Right-angle pre-detection parameters. */
#define RA_PRE_START_ROW    42
#define RA_PRE_END_ROW      32
#define RA_PRE_LOST_THRESH  3
#define RA_PRE_EDGE_MARGIN  3

extern uint8 g_ra_flag;
extern uint8 g_ra_pre_flag;
extern uint8 g_sym_component_flag;
extern IntersectionResult_t g_inter_result;
extern uint8 g_ip_max_row;
extern int16 ip_col_offset;
extern int16 ip_left_col;
extern int16 ip_right_col;

void track_fusion_init(void);
void track_fusion_update(void);
void right_angle_pre_detect(void);
void detect_intersection(void);

#endif /* TRACK_FUSION_H */
