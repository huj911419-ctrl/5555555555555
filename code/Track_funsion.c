#include "Track_funsion.h"
#include "Pid.h"

/* ========================================================================
 * Global variables
 * ======================================================================== */

TrackFusion_t g_tf;
uint8 Image_Binarize[TF_IMG_H][TF_IMG_W];
uint8 image_0[COMP_H][COMP_W];
uint16 Image_Threshold = (uint16)TF_OTSU_MIN_THRESHOLD;
int16 threshold_bias = 0;
static uint8 s_bin_tmp[TF_IMG_H][TF_IMG_W];

/* Otsu internals */
static uint8 s_otsu_cnt = 0;
static uint32 s_hist[256];

/* First miss row and last valid center for inflection detection */
static int16 s_first_miss_row = -1;
static int16 s_last_valid_center = TF_IMG_CENTER;
static int16 s_jidian_row = TF_JIDIAN_ROW;
static int16 s_track_half_width = TRACK_HALF_WIDTH;

#define IP_COL_BUF_SIZE 8
static int16 s_center_buf[IP_COL_BUF_SIZE];
static uint8 s_center_buf_cnt = 0u;
int16 ip_left_col = 9;
int16 ip_right_col = 84;
int16 ip_col_offset = 3;  // 0=倒数第1个，1=倒数第2个...

static uint8 side_probe_has_white(int16 row, int16 center_col);
static uint8 symmetric_component_at_row(int16 row, int16 center_col);
static int16 abs_i16(int16 v);
static void clear_inter_result(void);

static uint16 clamp_threshold_i16(int16 value)
{
    if (value < 20) value = 20;
    if (value > 240) value = 240;
    return (uint16)value;
}

/* ========================================================================
 * Image compression: 188x120 -> 94x60 (nearest-neighbor)
 * ======================================================================== */
static void compress_image(void)
{
    for (uint8 i = 0u; i < COMP_H; i++)
    {
        uint8 *dst = image_0[i];
        const uint8 *src = mt9v03x_image[(uint16)i * 2u];

        for (uint8 j = 0u; j < COMP_W; j++)
            dst[j] = src[(uint16)j * 2u];
    }
}

/* ========================================================================
 * Otsu adaptive threshold (compressed image)
 * ======================================================================== */
static uint16 calc_otsu(void)
{
    uint32 total = (uint32)COMP_H * COMP_W;
    uint32 sum = 0u;
    uint16 threshold = (uint16)TF_OTSU_MIN_THRESHOLD;

    for (uint16 i = 0; i < 256; i++) s_hist[i] = 0;
    for (uint8 i = 0; i < COMP_H; i++)
        for (uint8 j = 0; j < COMP_W; j++)
            s_hist[image_0[i][j]]++;

    for (uint16 t = 0; t < 256; t++)
        sum += (uint32)t * s_hist[t];

    uint32 sumB = 0u;
    uint32 wB = 0u;
    uint32 max_score = 0u;

    for (uint16 t = 0; t < 256; t++)
    {
        wB += s_hist[t];
        if (wB == 0) continue;

        uint32 wF = total - wB;
        if (wF == 0) break;

        sumB += (uint32)t * s_hist[t];

        uint32 meanB = sumB / wB;
        uint32 meanF = (sum - sumB) / wF;
        int32 diff = (int32)meanB - (int32)meanF;
        uint32 diff_abs = (uint32)((diff < 0) ? -diff : diff);
        uint32 weight = (wB * wF) >> 8;
        uint32 score = weight * diff_abs * diff_abs;

        if (score > max_score)
        {
            max_score = score;
            threshold = t;
        }
    }

    if (threshold < (uint16)TF_OTSU_MIN_THRESHOLD)
        threshold = (uint16)TF_OTSU_MIN_THRESHOLD;

    /* Apply threshold bias from menu. */
    return clamp_threshold_i16((int16)threshold + threshold_bias);
}

/* ========================================================================
 * Binarization (compressed image)
 * ======================================================================== */
static void binarize_image(void)
{
    for (uint16 i = 0; i < COMP_H; i++)
        for (uint16 j = 0; j < COMP_W; j++)
            Image_Binarize[i][j] = (image_0[i][j] > Image_Threshold) ? Image_WHITE : Image_BLACK;
}

/* ========================================================================
 * 3x3 double-buffer denoise on binary image
 * ======================================================================== */
static void denoise_binary_image(void)
{
    for (uint8 i = 0u; i < COMP_H; i++)
    {
        for (uint8 j = 0u; j < COMP_W; j++)
        {
            if (i == 0u || i >= (uint8)(COMP_H - 1u) ||
                j == 0u || j >= (uint8)(COMP_W - 1u))
            {
                s_bin_tmp[i][j] = Image_Binarize[i][j];
                continue;
            }

            uint8 white_cnt = 0u;

            for (int16 dr = -1; dr <= 1; dr++)
            {
                for (int16 dc = -1; dc <= 1; dc++)
                {
                    if (Image_Binarize[(uint8)((int16)i + dr)][(uint8)((int16)j + dc)] == Image_WHITE)
                        white_cnt++;
                }
            }

            if (Image_Binarize[i][j] == Image_WHITE)
            {
                s_bin_tmp[i][j] = (white_cnt >= TF_DENOISE_WHITE_MIN) ? Image_WHITE : Image_BLACK;
            }
            else
            {
                s_bin_tmp[i][j] = (white_cnt >= TF_DENOISE_BLACK_FILL) ? Image_WHITE : Image_BLACK;
            }
        }
    }

    for (uint8 i = 0u; i < COMP_H; i++)
        for (uint8 j = 0u; j < COMP_W; j++)
            Image_Binarize[i][j] = s_bin_tmp[i][j];
}

/* ========================================================================
 * Binary image helpers
 * ======================================================================== */
static inline uint8 is_white(int16 row, int16 col)
{
    if (row < 0 || row >= TF_IMG_H || col < 0 || col >= TF_IMG_W)
        return 0u;
    return (Image_Binarize[row][col] == Image_WHITE) ? 1u : 0u;
}

static inline uint8 is_left_edge(int16 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1) return 0u;
    return (!is_white(row, col - 1) && is_white(row, col) && is_white(row, col + 1)) ? 1u : 0u;
}

static inline uint8 is_right_edge(int16 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1) return 0u;
    return (is_white(row, col - 1) && is_white(row, col) && !is_white(row, col + 1)) ? 1u : 0u;
}

/* ========================================================================
 * Edge scanning functions (boundary-safe)
 * ======================================================================== */
static int16 scan_left_edge_right(int16 row, int16 start, int16 end)
{
    if (start < 1) start = 1;
    if (end >= TF_IMG_W - 1) end = TF_IMG_W - 2;
    if (start > end) return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_left_edge(row, c)) return c;
    return TF_INVALID;
}

static int16 scan_left_edge_left(int16 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1) start = TF_IMG_W - 2;
    if (end < 1) end = 1;
    if (start < end) return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_left_edge(row, c)) return c;
    return TF_INVALID;
}

static int16 scan_right_edge_left(int16 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1) start = TF_IMG_W - 2;
    if (end < 1) end = 1;
    if (start < end) return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_right_edge(row, c)) return c;
    return TF_INVALID;
}

static int16 scan_right_edge_right(int16 row, int16 start, int16 end)
{
    if (start < 1) start = 1;
    if (end >= TF_IMG_W - 1) end = TF_IMG_W - 2;
    if (start > end) return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_right_edge(row, c)) return c;
    return TF_INVALID;
}

static inline uint8 valid_pair(int16 lb, int16 rb)
{
    if (lb == TF_INVALID || rb == TF_INVALID) return 0u;
    int16 w = rb - lb;
    return (w >= TF_MIN_TRACK_WIDTH && w <= TF_MAX_TRACK_WIDTH) ? 1u : 0u;
}

static int16 clamp_edge_col(int16 col)
{
    if (col < 1) return 1;
    if (col > (int16)(TF_IMG_W - 2)) return (int16)(TF_IMG_W - 2);
    return col;
}

static int16 clamp_center_col(int16 col)
{
    if (col < 0) return 0;
    if (col > (int16)(TF_IMG_W - 1)) return (int16)(TF_IMG_W - 1);
    return col;
}

static int16 active_track_half_width(void)
{
    if (s_track_half_width < TRACK_HALF_WIDTH_MIN)
        return TRACK_HALF_WIDTH_MIN;
    if (s_track_half_width > TRACK_HALF_WIDTH_MAX)
        return TRACK_HALF_WIDTH_MAX;
    return s_track_half_width;
}

static void update_track_half_width(int16 lb, int16 rb)
{
    int16 w = rb - lb;
    int16 half;

    if (w < TF_MIN_TRACK_WIDTH || w > TF_MAX_TRACK_WIDTH)
        return;

    half = w / 2;
    if (half < TRACK_HALF_WIDTH_MIN)
        half = TRACK_HALF_WIDTH_MIN;
    if (half > TRACK_HALF_WIDTH_MAX)
        half = TRACK_HALF_WIDTH_MAX;

    s_track_half_width = (int16)((s_track_half_width * 3 + half + 2) / 4);
}

static uint8 normalize_edges_for_mode(int16 *lb, int16 *rb)
{
    int16 half = active_track_half_width();

    if (g_post_edge_side == EDGE_LEFT)
    {
        if (*lb == TF_INVALID)
            return 0u;
        *rb = clamp_edge_col((int16)(*lb + half * 2));
        return (*rb > *lb) ? 1u : 0u;
    }

    if (g_post_edge_side == EDGE_RIGHT)
    {
        if (*rb == TF_INVALID)
            return 0u;
        *lb = clamp_edge_col((int16)(*rb - half * 2));
        return (*rb > *lb) ? 1u : 0u;
    }

    if (!valid_pair(*lb, *rb))
        return 0u;

    update_track_half_width(*lb, *rb);
    return 1u;
}

static int16 calc_center_from_edges(int16 lb, int16 rb)
{
    int16 half = active_track_half_width();

    if (g_post_edge_side == EDGE_LEFT)
        return clamp_center_col((int16)(lb + half));

    if (g_post_edge_side == EDGE_RIGHT)
        return clamp_center_col((int16)(rb - half));

    return clamp_center_col((int16)((lb + rb) / 2));
}

/* ========================================================================
 * Baseline detection at bottom row
 * ======================================================================== */
static void set_jidian_row_data(int16 row, int16 lb, int16 rb)
{
    s_jidian_row = row;
    g_tf.left_jidian = (uint8)lb;
    g_tf.right_jidian = (uint8)rb;
}

static uint8 find_jidian_at_row(int16 row, int16 *out_lb, int16 *out_rb)
{
    const int16 mid = (int16)TF_IMG_CENTER;
    const int16 qtr = (int16)(TF_IMG_W / 4);
    const int16 tqtr = (int16)(TF_IMG_W * 3 / 4);
    int16 lb = TF_INVALID, rb = TF_INVALID;

    if (is_white(row, mid - 1) && is_white(row, mid) && is_white(row, mid + 1))
    {
        lb = scan_left_edge_left(row, mid, 1);
        rb = scan_right_edge_right(row, mid, TF_IMG_W - 2);
    }
    else if (is_white(row, qtr - 1) && is_white(row, qtr) && is_white(row, qtr + 1))
    {
        lb = scan_left_edge_left(row, qtr, 1);
        rb = scan_right_edge_right(row, qtr, TF_IMG_W - 2);
    }
    else if (is_white(row, tqtr - 1) && is_white(row, tqtr) && is_white(row, tqtr + 1))
    {
        lb = scan_left_edge_left(row, tqtr, 1);
        rb = scan_right_edge_right(row, tqtr, TF_IMG_W - 2);
    }
    else
    {
        lb = scan_left_edge_right(row, 1, TF_IMG_W - 2);
        if (lb != TF_INVALID)
            rb = scan_right_edge_left(row, TF_IMG_W - 2, lb + TF_MIN_TRACK_WIDTH);
    }

    if (!normalize_edges_for_mode(&lb, &rb)) return 0u;

    *out_lb = lb;
    *out_rb = rb;
    return 1u;
}

static uint8 find_jidian1(void)
{
    int16 lb = TF_INVALID;
    int16 rb = TF_INVALID;

    for (int16 row = (int16)TF_JIDIAN_ROW;
         row >= (int16)TF_JIDIAN_MIN_ROW; row--)
    {
        if (find_jidian_at_row(row, &lb, &rb))
        {
            set_jidian_row_data(row, lb, rb);
            return 1u;
        }
    }

    return 0u;
}

/* ========================================================================
 * Row-by-row edge search (local range from previous row)
 * ======================================================================== */
static uint8 search_row_edges(int16 row, int16 prev_lb, int16 prev_rb,
                              int16 *out_lb, int16 *out_rb)
{
    int16 lb = TF_INVALID, rb = TF_INVALID;
    const int16 R = TF_LOCAL_RANGE;
    const int16 MID = TF_IMG_CENTER;

    /* Left edge search: local → widen → full scan */
    lb = scan_left_edge_right(row, prev_lb - R, prev_lb + R);
    if (lb == TF_INVALID) lb = scan_left_edge_left(row, prev_lb + R, prev_lb - R);
    if (lb == TF_INVALID) lb = scan_left_edge_left(row, MID, 1);

    /* Right edge search: local → widen → full scan */
    rb = scan_right_edge_left(row, prev_rb + R, prev_rb - R);
    if (rb == TF_INVALID) rb = scan_right_edge_right(row, prev_rb - R, prev_rb + R);
    if (rb == TF_INVALID) rb = scan_right_edge_right(row, MID, TF_IMG_W - 2);

    if (!normalize_edges_for_mode(&lb, &rb))
        return 0u;

    *out_lb = lb;
    *out_rb = rb;
    return 1u;
}

/* ========================================================================
 * Initialization
 * ======================================================================== */
void track_fusion_init(void)
{
    uint16 i, j;
    for (i = 0; i < TF_IMG_H; i++)
    {
        g_tf.left_edge[i] = TF_INVALID;
        g_tf.right_edge[i] = TF_INVALID;
        g_tf.center_line[i] = TF_INVALID;
        g_tf.row_valid[i] = 0u;
    }
    g_tf.error = 0;
    g_tf.lookahead_error = 0;
    g_tf.error_trend = 0;
    g_tf.valid_row_count = 0u;
    g_tf.line_lost = 0u;
    g_tf.left_jidian = (uint8)TF_IMG_CENTER;
    g_tf.right_jidian = (uint8)TF_IMG_CENTER;
    s_jidian_row = (int16)TF_JIDIAN_ROW;
    clear_inter_result();
    Image_Threshold = (uint16)TF_OTSU_MIN_THRESHOLD;
    threshold_bias = 0;
    s_otsu_cnt = (uint8)TF_OTSU_INTERVAL;
    s_track_half_width = TRACK_HALF_WIDTH;

    for (i = 0; i < COMP_H; i++)
        for (j = 0; j < COMP_W; j++)
        {
            Image_Binarize[i][j] = Image_BLACK;
            image_0[i][j] = 0u;
            s_bin_tmp[i][j] = Image_BLACK;
        }
}

/* ========================================================================
 * Weighted center-line error calculation
 * ======================================================================== */
static int16 calc_weighted_center(int16 start_row, int16 end_row, uint8 top_heavy)
{
    int32 sum = 0, wtotal = 0;
    for (int16 row = start_row; row >= end_row; row--)
    {
        if (g_tf.row_valid[row])
        {
            int32 w = top_heavy
                ? (int32)(start_row - row + 1)
                : (int32)(row - end_row + 1);
            sum += (int32)g_tf.center_line[row] * w;
            wtotal += w;
        }
    }
    return (wtotal > 0) ? (int16)(sum / wtotal) : (int16)TF_IMG_CENTER;
}

/* ========================================================================
 * Per-frame update: compress -> Otsu -> binarize -> denoise -> scan edges
 * ======================================================================== */
void track_fusion_update(void)
{
    /* 1. Clear per-frame data */
    for (uint8 i = 0; i < TF_IMG_H; i++)
    {
        g_tf.left_edge[i] = TF_INVALID;
        g_tf.right_edge[i] = TF_INVALID;
        g_tf.center_line[i] = TF_INVALID;
        g_tf.row_valid[i] = 0u;
    }
    g_tf.error = 0;
    g_tf.valid_row_count = 0u;
    g_tf.line_lost = 0u;
    g_sym_component_flag = 0u;

    /* 2. Compress 188x120 -> 94x60 */
    compress_image();

#if TF_FIXED_THRESHOLD_ENABLE
    /* 3. Fixed threshold for stable race lighting. */
    Image_Threshold = clamp_threshold_i16(
        (int16)TF_FIXED_THRESHOLD_VALUE + threshold_bias);
#else
    /* 3. Otsu adaptive threshold */
    s_otsu_cnt++;
    if (s_otsu_cnt >= TF_OTSU_INTERVAL)
    {
        s_otsu_cnt = 0u;
        Image_Threshold = calc_otsu();
    }
#endif

    /* 4. Binarize */
    binarize_image();

    /* 5. Denoise (4-neighbor filter) */
    denoise_binary_image();

    /* 6. Find baseline at bottom row */
    if (!find_jidian1())
    {
        g_tf.line_lost = 1u;
        return;
    }

    const int16 jrow = s_jidian_row;
    int16 prev_lb = (int16)g_tf.left_jidian;
    int16 prev_rb = (int16)g_tf.right_jidian;

    g_tf.left_edge[jrow] = prev_lb;
    g_tf.right_edge[jrow] = prev_rb;
    g_tf.center_line[jrow] = calc_center_from_edges(prev_lb, prev_rb);
    g_tf.row_valid[jrow] = 1u;
    g_tf.valid_row_count = 1u;

    /* 7. Row-by-row upward edge search */
    uint8 miss_streak = 0u;
    int16 end_row = (int16)TF_SEARCH_END_ROW;
    s_first_miss_row = -1;
    s_center_buf_cnt = 0u;
    s_last_valid_center = g_tf.center_line[jrow];

    for (int16 row = jrow - 1; row >= end_row; row--)
    {
        int16 lb, rb;
        if (search_row_edges(row, prev_lb, prev_rb, &lb, &rb))
        {
            g_tf.left_edge[row] = lb;
            g_tf.right_edge[row] = rb;
            g_tf.center_line[row] = calc_center_from_edges(lb, rb);
            g_tf.row_valid[row] = 1u;
            g_tf.valid_row_count++;
            prev_lb = lb;
            prev_rb = rb;
            s_last_valid_center = g_tf.center_line[row];
            s_center_buf[s_center_buf_cnt % IP_COL_BUF_SIZE] = g_tf.center_line[row];
            s_center_buf_cnt++;
            miss_streak = 0u;

            uint8 sym_row = symmetric_component_at_row(row, g_tf.center_line[row]);
            if (sym_row)
                g_sym_component_flag = 1u;

            if (row >= (int16)INTER_MIN_MISS_ROW &&
                (side_probe_has_white(row, ip_left_col) ||
                 side_probe_has_white(row, ip_right_col)) &&
                !sym_row)
            {
                if (s_first_miss_row < 0)
                    s_first_miss_row = row;
                break;
            }
        }
        else
        {
            g_tf.left_edge[row] = lb;
            g_tf.right_edge[row] = rb;
            g_tf.row_valid[row] = 0u;
            miss_streak++;
            if (miss_streak == 2 && s_first_miss_row < 0)
                s_first_miss_row = row + 1;  /* row+1 = last valid row before miss */
            if (miss_streak >= TF_MAX_MISS_ROWS) break;
        }
    }

    if (g_tf.valid_row_count == 0u)
    {
        g_tf.line_lost = 1u;
        return;
    }

    /* 8. Weighted center-line error (bottom-heavy for normal tracking) */
    int16 avg_center = calc_weighted_center(jrow, end_row, 0);
    g_tf.error = -(avg_center - (int16)TF_IMG_CENTER);
    g_tf.line_lost = 0u;

    /* 9. Lookahead error (top-heavy for curve prediction) */
    {
        int16 la_center = calc_weighted_center(
            (int16)TF_LOOKAHEAD_START_ROW, (int16)TF_LOOKAHEAD_END_ROW, 1);
        g_tf.lookahead_error = -(la_center - (int16)TF_IMG_CENTER);
        g_tf.error_trend = g_tf.lookahead_error - g_tf.error;
    }

    /* 10. Scale errors to original coordinate system (x2) for PID compatibility */
    g_tf.error = g_tf.error * 2;
    g_tf.lookahead_error = g_tf.lookahead_error * 2;
    g_tf.error_trend = g_tf.error_trend * 2;
}

/* ========================================================================
 * Right angle pre-detection (far-field, for early speed reduction)
 * ======================================================================== */
uint8 g_ra_pre_flag = 0u;

void right_angle_pre_detect(void)
{
    static uint8 s_on_cnt = 0;
    static uint8 s_off_cnt = 0;

    uint8 right_lost = 0u;
    uint8 left_lost = 0u;
    uint8 sym_component_rows = 0u;

    for (int16 i = (int16)RA_PRE_START_ROW;
         i > (int16)RA_PRE_END_ROW; i--)
    {
        if (!g_tf.row_valid[i])
            continue;

        if (symmetric_component_at_row(i, g_tf.center_line[i]))
        {
            sym_component_rows++;
            g_sym_component_flag = 1u;
        }

        if (g_tf.right_edge[i] >= (int16)(TF_IMG_W - RA_PRE_EDGE_MARGIN))
            right_lost++;

        if (g_tf.left_edge[i] <= (int16)RA_PRE_EDGE_MARGIN)
            left_lost++;
    }

    uint8 detected = (right_lost >= RA_PRE_LOST_THRESH ||
                      left_lost >= RA_PRE_LOST_THRESH) ? 1u : 0u;

    if (sym_component_rows >= INTER_SYM_PRE_ROWS)
    {
        detected = 0u;
        s_on_cnt = 0u;
        s_off_cnt = 0u;
        g_ra_pre_flag = 0u;
        return;
    }

    if (detected)
    {
        s_on_cnt++;
        s_off_cnt = 0;
        if (s_on_cnt >= 2)
            g_ra_pre_flag = 1u;
    }
    else
    {
        s_off_cnt++;
        s_on_cnt = 0;
        if (s_off_cnt >= 5)
            g_ra_pre_flag = 0u;
    }
}

/* ========================================================================
 * Inflection point detection (scan upward from lost row) + box drawing
 *
 * Method: when row-by-row scanning loses track, scan upward from that
 * row to find white pixels at the leftmost/rightmost image columns.
 * White at left edge  → left turn,  inflection at right side of track
 * White at right edge → right turn, inflection at left side of track
 * White at both edges → cross,      inflection at center
 * ======================================================================== */

uint8 g_ra_flag = 0u;
uint8 g_sym_component_flag = 0u;
IntersectionResult_t g_inter_result;
uint8 g_ip_max_row = 0u;

static uint8 s_inter_lock_cnt = 0u;
static uint8 s_inter_cooldown_cnt = 0u;
static uint8 s_box_candidate_valid = 0u;
static uint8 s_box_lock_valid = 0u;
static uint8 s_box_stable_cnt = 0u;
static int16 s_box_last_cx = 0;
static int16 s_box_last_cy = 0;
static int16 s_box_lock_cx = 0;
static int16 s_box_lock_cy = 0;
static uint8 s_type_vote[INTER_TYPE_VOTE_FRAMES];
static uint8 s_type_vote_idx = 0u;
static uint8 s_type_vote_cnt = 0u;

static uint8 side_probe_has_white(int16 row, int16 center_col)
{
    int16 start = center_col - 2;
    int16 end = center_col + 2;
    uint8 white_cnt = 0u;

    if (row < 0 || row >= TF_IMG_H) return 0u;
    if (start < 0) start = 0;
    if (end >= TF_IMG_W) end = TF_IMG_W - 1;

    for (int16 col = start; col <= end; col++)
    {
        if (Image_Binarize[row][col] == Image_WHITE)
            white_cnt++;
    }

    return (white_cnt >= 3u) ? 1u : 0u;
}

static uint8 max_white_streak_on_row(int16 row, int16 col_start, int16 col_end)
{
    uint8 streak = 0u;
    uint8 max_streak = 0u;

    if (row < 0 || row >= TF_IMG_H) return 0u;
    if (col_start < 0) col_start = 0;
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    if (col_start > col_end) return 0u;

    for (int16 col = col_start; col <= col_end; col++)
    {
        if (Image_Binarize[row][col] == Image_WHITE)
        {
            streak++;
            if (streak > max_streak)
                max_streak = streak;
        }
        else
        {
            streak = 0u;
        }
    }

    return max_streak;
}

static uint8 symmetric_component_shape(int16 left_row, int16 right_row, int16 center_col)
{
    if (left_row < 0 || right_row < 0)
        return 0u;

    if (abs_i16(left_row - right_row) > INTER_SYM_ROW_DELTA)
        return 0u;

    if (abs_i16(center_col - (int16)TF_IMG_CENTER) > INTER_SYM_CENTER_DELTA)
        return 0u;

    uint8 left_streak = max_white_streak_on_row(
        left_row, 0, center_col - TF_MIN_TRACK_WIDTH);
    uint8 right_streak = max_white_streak_on_row(
        right_row, center_col + TF_MIN_TRACK_WIDTH, TF_IMG_W - 1);

    if (left_streak >= INTER_SYM_BRANCH_STREAK ||
        right_streak >= INTER_SYM_BRANCH_STREAK)
    {
        return 0u;
    }

    return 1u;
}

static uint8 symmetric_component_at_row(int16 row, int16 center_col)
{
    if (!side_probe_has_white(row, ip_left_col) ||
        !side_probe_has_white(row, ip_right_col))
    {
        return 0u;
    }

    return symmetric_component_shape(row, row, center_col);
}

/* Find inflection point from the first lost area using adjustable side probes.
 * Returns which side has white via found_side: 1=right, 2=left, 3=both */
static uint8 find_ip_from_lost_row(int16 lost_row, int16 last_center,
                                    InflectionPoint_t *ip, uint8 *found_side)
{
    ip->valid = 0u; ip->row = 0; ip->col = 0;
    *found_side = 0u;

    int16 left_white_row = -1;
    int16 right_white_row = -1;

    for (int16 row = lost_row; row >= (int16)TF_SEARCH_END_ROW; row--)
    {
        /* Check adjustable left probe column for white */
        if (left_white_row < 0 && side_probe_has_white(row, ip_left_col))
            left_white_row = row;

        /* Check adjustable right probe column for white */
        if (right_white_row < 0 && side_probe_has_white(row, ip_right_col))
            right_white_row = row;

        /* Both found, no need to scan further */
        if (left_white_row >= 0 && right_white_row >= 0)
            break;
    }

    if (symmetric_component_shape(left_white_row, right_white_row, last_center))
    {
        ip->valid = 0u;
        *found_side = 0u;
        g_sym_component_flag = 1u;
        return 0u;
    }

    /* Take the one closer to camera (larger row number) */
    int16 ip_row = -1;
    if (left_white_row >= 0 && right_white_row >= 0)
    {
        ip_row = (left_white_row > right_white_row) ? left_white_row : right_white_row;
        *found_side = 3u; /* both sides */
    }
    else if (right_white_row >= 0)
    {
        ip_row = right_white_row;
        *found_side = 1u; /* right side has road → right turn */
    }
    else if (left_white_row >= 0)
    {
        ip_row = left_white_row;
        *found_side = 2u; /* left side has road → left turn */
    }

    if (ip_row < 0) return 0u;

    ip->valid = 1u;
    ip->row = ip_row;
    ip->col = last_center;
    return 1u;
}

static uint8 inter_horizontal_band_has_road(int16 center_row,
                                            int16 col_start,
                                            int16 col_end,
                                            uint8 min_white,
                                            uint8 min_streak)
{
    int16 half = (int16)(INTER_BAND_THICKNESS / 2);
    int16 row_start = center_row - half;
    int16 row_end = center_row + half;
    uint16 white_count = 0u;
    uint8 max_streak = 0u;

    if (row_start < 0) row_start = 0;
    if (row_end >= TF_IMG_H) row_end = TF_IMG_H - 1;
    if (col_start < 0) col_start = 0;
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    if (row_start > row_end || col_start > col_end) return 0u;

    for (int16 r = row_start; r <= row_end; r++)
    {
        uint8 streak = 0u;
        for (int16 c = col_start; c <= col_end; c++)
        {
            if (is_white(r, c))
            {
                white_count++;
                streak++;
                if (streak > max_streak) max_streak = streak;
            }
            else
            {
                streak = 0u;
            }
        }
    }

    return (white_count >= min_white && max_streak >= min_streak) ? 1u : 0u;
}

static uint8 inter_vertical_band_has_road(int16 center_col,
                                          int16 row_start,
                                          int16 row_end,
                                          uint8 min_white,
                                          uint8 min_streak)
{
    int16 half = (int16)(INTER_BAND_THICKNESS / 2);
    int16 col_start = center_col - half;
    int16 col_end = center_col + half;
    uint16 white_count = 0u;
    uint8 max_streak = 0u;

    if (row_start < 0) row_start = 0;
    if (row_end >= TF_IMG_H) row_end = TF_IMG_H - 1;
    if (col_start < 0) col_start = 0;
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    if (row_start > row_end || col_start > col_end) return 0u;

    for (int16 c = col_start; c <= col_end; c++)
    {
        uint8 streak = 0u;
        for (int16 r = row_start; r <= row_end; r++)
        {
            if (is_white(r, c))
            {
                white_count++;
                streak++;
                if (streak > max_streak) max_streak = streak;
            }
            else
            {
                streak = 0u;
            }
        }
    }

    return (white_count >= min_white && max_streak >= min_streak) ? 1u : 0u;
}

static uint8 inter_side_branch_has_road(int16 row,
                                        int16 center_col,
                                        uint8 side)
{
    int16 row_start = row - (int16)(INTER_BAND_THICKNESS / 2);
    int16 row_end = row + (int16)(INTER_BAND_THICKNESS / 2);
    int16 col_start;
    int16 col_end;
    uint8 max_streak = 0u;

    if (row_start < 0) row_start = 0;
    if (row_end >= TF_IMG_H) row_end = TF_IMG_H - 1;

    if (side == 1u)
    {
        col_start = center_col + TF_MIN_TRACK_WIDTH;
        col_end = TF_IMG_W - 1;
    }
    else
    {
        col_start = 0;
        col_end = center_col - TF_MIN_TRACK_WIDTH;
    }

    if (col_start < 0) col_start = 0;
    if (col_end >= TF_IMG_W) col_end = TF_IMG_W - 1;
    if (col_start > col_end) return 0u;

    for (int16 r = row_start; r <= row_end; r++)
    {
        uint8 streak = max_white_streak_on_row(r, col_start, col_end);
        if (streak > max_streak)
            max_streak = streak;
    }

    return (max_streak >= INTER_BRANCH_MIN_STREAK) ? 1u : 0u;
}

static void apply_ip_col_from_buffer(InflectionPoint_t *ip, uint8 found_side)
{
    if (ip->valid && s_center_buf_cnt > 0u)
    {
        uint8 offset = (uint8)ip_col_offset;
        uint8 total = s_center_buf_cnt;
        if (offset >= total) offset = total - 1u;
        uint8 idx = (uint8)((total - 1u - offset) % IP_COL_BUF_SIZE);
        ip->col = s_center_buf[idx];
    }

    if (!ip->valid)
        return;

    if (found_side == 1u)
        ip->col = clamp_center_col((int16)(ip->col + INTER_IP_SIDE_BIAS));
    else if (found_side == 2u)
        ip->col = clamp_center_col((int16)(ip->col - INTER_IP_SIDE_BIAS));
}

static int16 abs_i16(int16 v)
{
    return (v < 0) ? (int16)(-v) : v;
}

static void reset_box_lock(void)
{
    s_box_candidate_valid = 0u;
    s_box_lock_valid = 0u;
    s_box_stable_cnt = 0u;
    s_type_vote_idx = 0u;
    s_type_vote_cnt = 0u;

    for (uint8 i = 0u; i < INTER_TYPE_VOTE_FRAMES; i++)
        s_type_vote[i] = 0u;
}

static void clear_inter_result(void)
{
    g_inter_result.left_ip.valid = 0u;
    g_inter_result.left_ip.row = 0;
    g_inter_result.left_ip.col = 0;
    g_inter_result.right_ip.valid = 0u;
    g_inter_result.right_ip.row = 0;
    g_inter_result.right_ip.col = 0;
    g_inter_result.box_top = 0u;
    g_inter_result.box_bottom = 0u;
    g_inter_result.box_left = 0u;
    g_inter_result.box_right = 0u;
    g_inter_result.detected_type = 0u;
    g_ip_max_row = 0u;
}

static int16 clamp_i16(int16 v, int16 min_v, int16 max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static int16 estimate_inter_track_width(int16 cy)
{
    for (int16 offset = 0; offset <= 8; offset++)
    {
        int16 r1 = cy + offset;
        int16 r2 = cy - offset;

        if (r1 >= 0 && r1 < TF_IMG_H && g_tf.row_valid[r1] &&
            valid_pair(g_tf.left_edge[r1], g_tf.right_edge[r1]))
        {
            return g_tf.right_edge[r1] - g_tf.left_edge[r1];
        }

        if (offset != 0 && r2 >= 0 && r2 < TF_IMG_H && g_tf.row_valid[r2] &&
            valid_pair(g_tf.left_edge[r2], g_tf.right_edge[r2]))
        {
            return g_tf.right_edge[r2] - g_tf.left_edge[r2];
        }
    }

    if (valid_pair((int16)g_tf.left_jidian, (int16)g_tf.right_jidian))
        return (int16)g_tf.right_jidian - (int16)g_tf.left_jidian;

    return (int16)(BOX_WIDTH / INTER_BOX_WIDTH_SCALE);
}

static void build_inter_box(int16 cx, int16 cy,
                            int16 *top, int16 *bottom,
                            int16 *left, int16 *right)
{
    int16 track_w = estimate_inter_track_width(cy);
    int16 box_w = clamp_i16((int16)(track_w * INTER_BOX_WIDTH_SCALE),
                            INTER_BOX_WIDTH_MIN,
                            INTER_BOX_WIDTH_MAX);
    int16 box_h = clamp_i16((int16)(track_w * INTER_BOX_HEIGHT_SCALE),
                            INTER_BOX_HEIGHT_MIN,
                            INTER_BOX_HEIGHT_MAX);
    int16 b_top = cy - box_h / 2;
    int16 b_bottom = cy + box_h / 2;
    int16 b_left = cx - box_w / 2;
    int16 b_right = cx + box_w / 2;

    if (b_top < 0) b_top = 0;
    if (b_bottom >= TF_IMG_H) b_bottom = TF_IMG_H - 1;
    if (b_left < 0) b_left = 0;
    if (b_right >= TF_IMG_W) b_right = TF_IMG_W - 1;

    *top = b_top;
    *bottom = b_bottom;
    *left = b_left;
    *right = b_right;
}

static uint8 update_box_lock(int16 cx, int16 cy)
{
    if (!s_box_candidate_valid)
    {
        s_box_candidate_valid = 1u;
        s_box_lock_valid = 0u;
        s_box_stable_cnt = 1u;
        s_box_last_cx = cx;
        s_box_last_cy = cy;
        s_box_lock_cx = cx;
        s_box_lock_cy = cy;
        return 0u;
    }

    if (abs_i16(cx - s_box_last_cx) <= INTER_BOX_STABLE_DELTA &&
        abs_i16(cy - s_box_last_cy) <= INTER_BOX_STABLE_DELTA)
    {
        if (s_box_stable_cnt < 255u)
            s_box_stable_cnt++;
    }
    else
    {
        s_box_stable_cnt = 1u;
        s_box_lock_valid = 0u;
        s_type_vote_idx = 0u;
        s_type_vote_cnt = 0u;
    }

    s_box_last_cx = cx;
    s_box_last_cy = cy;

    if (!s_box_lock_valid && s_box_stable_cnt >= INTER_BOX_STABLE_FRAMES)
    {
        s_box_lock_valid = 1u;
        s_box_lock_cx = cx;
        s_box_lock_cy = cy;
    }

    return s_box_lock_valid;
}

static uint8 vote_inter_type(uint8 detected)
{
    if (detected == 0u)
    {
        s_type_vote_idx = 0u;
        s_type_vote_cnt = 0u;
        return 0u;
    }

    s_type_vote[s_type_vote_idx] = detected;
    s_type_vote_idx++;
    if (s_type_vote_idx >= INTER_TYPE_VOTE_FRAMES)
        s_type_vote_idx = 0u;

    if (s_type_vote_cnt < INTER_TYPE_VOTE_FRAMES)
        s_type_vote_cnt++;

    if (s_type_vote_cnt < INTER_TYPE_VOTE_FRAMES)
        return 0u;

    for (uint8 type = 1u; type <= 5u; type++)
    {
        uint8 count = 0u;
        for (uint8 i = 0u; i < INTER_TYPE_VOTE_FRAMES; i++)
        {
            if (s_type_vote[i] == type)
                count++;
        }

        if (count >= INTER_TYPE_VOTE_MIN)
            return type;
    }

    return 0u;
}

static uint8 fast_confirm_inter_type(uint8 detected)
{
#if INTER_FAST_CONFIRM_ENABLE
    if (detected >= 3u && detected <= 5u)
        return detected;

    if ((detected == 1u || detected == 2u) &&
        (g_ra_pre_flag != 0u || g_tf.line_lost != 0u))
    {
        return detected;
    }
#endif

    return 0u;
}

/* Main intersection detection */
void detect_intersection(void)
{
    if (s_inter_cooldown_cnt > 0u)
    {
        s_inter_cooldown_cnt++;
        if (s_inter_cooldown_cnt >= INTER_COOLDOWN_FRAMES) s_inter_cooldown_cnt = 0u;
        clear_inter_result();
        return;
    }

    if (s_inter_lock_cnt > 0u)
    {
        InflectionPoint_t ip;
        uint8 found_side = 0u;
        ip.valid = 0u; ip.row = 0; ip.col = 0;

        if (s_first_miss_row >= (int16)INTER_MIN_MISS_ROW)
        {
            find_ip_from_lost_row(s_first_miss_row, s_last_valid_center, &ip, &found_side);
            apply_ip_col_from_buffer(&ip, found_side);
        }

        if (ip.valid)
        {
            g_inter_result.left_ip = ip;
            g_inter_result.right_ip = ip;
            g_ip_max_row = (uint8)(ip.row * 2);
        }

        if (g_ra_flag == 0u)
        {
            s_inter_lock_cnt = 0u;
            s_inter_cooldown_cnt = 1u;
            reset_box_lock();
            clear_inter_result();
        }
        else
        {
            s_inter_lock_cnt++;
            if (s_inter_lock_cnt >= INTER_MAX_LOCK_FRAMES)
            {
                g_ra_flag = 0u;
                s_inter_lock_cnt = 0u;
                s_inter_cooldown_cnt = 1u;
                reset_box_lock();
                clear_inter_result();
            }
        }
        return;
    }

    /* Find inflection point from lost row */
    InflectionPoint_t ip;
    ip.valid = 0u; ip.row = 0; ip.col = 0;
    uint8 found_side = 0u; /* 1=right, 2=left, 3=both */

    if (g_tf.valid_row_count > INTER_MIN_VALID_ROWS)
    {
        reset_box_lock();
        clear_inter_result();
        return;
    }

    if (g_tf.line_lost == 0u &&
        g_ra_pre_flag == 0u &&
        s_first_miss_row < (int16)INTER_MIN_MISS_ROW)
    {
        reset_box_lock();
        clear_inter_result();
        return;
    }

    if (s_first_miss_row >= (int16)INTER_MIN_MISS_ROW)
        find_ip_from_lost_row(s_first_miss_row, s_last_valid_center, &ip, &found_side);

    /* 用倒数第N个有效行的中点作为框列号 */
    apply_ip_col_from_buffer(&ip, found_side);

    g_inter_result.left_ip = ip;
    g_inter_result.right_ip = ip;

    g_ip_max_row = ip.valid ? (uint8)(ip.row * 2) : 0u;

    if (!ip.valid)
    {
        reset_box_lock();
        clear_inter_result();
        return;
    }

    /* Inflection point too far, skip box drawing */
    if (ip.row < INTER_BOX_START_ROW)
    {
        reset_box_lock();
        clear_inter_result();
        return;
    }

    if (!update_box_lock(ip.col, ip.row))
    {
        int16 dbg_top, dbg_bottom, dbg_left, dbg_right;

        build_inter_box(ip.col, ip.row, &dbg_top, &dbg_bottom, &dbg_left, &dbg_right);
        g_inter_result.box_top = (uint8)dbg_top;
        g_inter_result.box_bottom = (uint8)dbg_bottom;
        g_inter_result.box_left = (uint8)dbg_left;
        g_inter_result.box_right = (uint8)dbg_right;
        g_inter_result.detected_type = 0u;
        return;
    }

    /* Classify with the locked box, not the moving candidate. */
    uint8 detected = 0u;

    int16 b_top, b_bottom, b_left, b_right;

    build_inter_box(s_box_lock_cx, s_box_lock_cy, &b_top, &b_bottom, &b_left, &b_right);

    g_inter_result.box_top = (uint8)b_top;
    g_inter_result.box_bottom = (uint8)b_bottom;
    g_inter_result.box_left = (uint8)b_left;
    g_inter_result.box_right = (uint8)b_right;

    /* Check box edge bands instead of one-pixel edges. */
    uint8 top_has = inter_horizontal_band_has_road(
        b_top, b_left, b_right, INTER_TOP_MIN_WHITE, INTER_BAND_MIN_STREAK);
    uint8 left_has = inter_vertical_band_has_road(
        b_left, b_top, b_bottom, INTER_SIDE_MIN_WHITE, INTER_BAND_MIN_STREAK);
    uint8 right_has = inter_vertical_band_has_road(
        b_right, b_top, b_bottom, INTER_SIDE_MIN_WHITE, INTER_BAND_MIN_STREAK);
    uint8 left_branch_has = inter_side_branch_has_road(ip.row, ip.col, 2u);
    uint8 right_branch_has = inter_side_branch_has_road(ip.row, ip.col, 1u);

    /* 分类（框边检测）：上+左=3  上+右=4  左+右=5  上+左+右=6
     * 单边用 found_side（图像边界检测）：右=1  左=2 */
    /* No route uses type 6. Treat top+left+right as invalid, not as T. */
    if (top_has && left_has && right_has)
        detected = 0u;
    else if (left_has && right_has)
        detected = 5u;
    else if (top_has && left_has)
        detected = 3u;
    else if (top_has && right_has)
        detected = 4u;
    else if (found_side == 1u && right_branch_has)
        detected = 1u;
    else if (found_side == 2u && left_branch_has)
        detected = 2u;

    g_inter_result.detected_type = detected;

    uint8 voted_type = fast_confirm_inter_type(detected);
    if (voted_type == 0u)
        voted_type = vote_inter_type(detected);

    if (voted_type != 0u)
    {
        g_ra_flag = voted_type;
        g_inter_result.detected_type = voted_type;
        s_inter_lock_cnt = 1u;
        reset_box_lock();
    }
}
