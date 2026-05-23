#include "Pid.h"
#include "Menu.h"
#include "Track_funsion.h"
#include "IMU.h"

int16 base_speed = 0;
uint8 g_post_edge_side = EDGE_BOTH;

uint8 ra_dbg_state = 0u;
uint8 ra_dbg_phase = 0u;
uint8 ra_dbg_dir = 0u;
uint8 ra_dbg_ip_row = 0u;
uint16 ra_dbg_timer = 0u;
uint16 ra_dbg_hard_cnt = 0u;
uint8 ra_dbg_exit_good_cnt = 0u;
int16 ra_dbg_yaw10 = 0;

static float s_steer_last_pos_err = 0.0f;
static float s_steer_last_raw_err = 0.0f;
static float s_prev_steer_output = 0.0f;
static float s_steer_ff_filtered = 0.0f;
static float s_steer_filtered_err = 0.0f;
static uint8 s_steer_d_reset_flag = 0u;
static float s_speed_integral = 0.0f;
static float s_prev_target_speed = 0.0f;
static uint8 s_speed_ff_ready = 0u;
static uint32 s_motor_run_counter = 0;
static int16 s_base_speed_ramped = 0;
static uint8 s_straight_cnt = 0u;
static uint8 s_straight_active = 0u;
static uint8 s_turn_shield_frames = 0u;
static uint32 s_turn_shield_dist = 0u;
static uint8 s_edge_active = 0u;
static uint8 s_edge_side = EDGE_BOTH;
static uint16 s_edge_cnt = 0u;
static uint16 s_edge_target = 0u;
static uint8 s_lost_search_active = 0u;
static uint8 s_lost_line_cnt = 0u;
static uint16 s_lost_search_cnt = 0u;
static uint8 s_lost_search_dir = 1u;
static int16 s_lost_last_err = 0;

typedef enum { RA_ST_NONE, RA_ST_ACTIVE } RaState;
typedef enum { RA_PH_WAIT, RA_PH_SLOW, RA_PH_APPROACH, RA_PH_HARD, RA_PH_RECOVER } RaPhase;

static RaState s_ra_state = RA_ST_NONE;
static RaPhase s_ra_phase = RA_PH_WAIT;
static uint8 s_ra_dir = 0u;
static uint8 s_ra_orig_flag = 0u;
static uint8 s_ra_ip_row = 0u;
static uint8 s_ra_straight = 0u;
static uint8 s_ra_post_edge_side = EDGE_BOTH;
static uint16 s_ra_post_edge_ms = 0u;
static uint8 s_ra_exit_good_cnt = 0u;
static uint8 s_ra_recover_good_cnt = 0u;
static uint16 s_ra_approach_cnt = 0u;
static uint16 s_ra_timer = 0u;
static uint16 s_ra_hard_cnt = 0u;
static uint16 s_ra_recover_cnt = 0u;
static uint16 s_ra_phase_cnt = 0u;

static int16 abs_i16(int16 v);

#define ACT_STRAIGHT 0u
#define ACT_RIGHT    1u
#define ACT_LEFT     2u
#define ACT_AUTO     3u

typedef struct
{
    uint8 count;
    uint8 flag;
    uint8 action;
    uint8 post_edge_side;
    uint16 post_edge_ms;
} IntersectionRule;

#define RULE(count, flag, action) \
    { (count), (flag), (action), EDGE_BOTH, 0u }
#define RULE_EDGE(count, flag, action, edge_side, edge_ms) \
    { (count), (flag), (action), (edge_side), (edge_ms) }
#define RULE_AUTO(count, flag, action, edge_ms) \
    { (count), (flag), (action), EDGE_AUTO, (edge_ms) }
#define RULE_RA(count, flag) \
    { (count), (flag), ACT_AUTO, EDGE_BOTH, 0u }
#define RULE_RA_AUTO(count, flag, edge_ms) \
    { (count), (flag), ACT_AUTO, EDGE_AUTO, (edge_ms) }
#define RULE_RA_EDGE(count, flag, edge_side, edge_ms) \
    { (count), (flag), ACT_AUTO, (edge_side), (edge_ms) }

/* 路线表：按图中黑线走向填写。
 * RULE：执行指定动作，不开启单边巡线。
 * RULE_AUTO：执行指定动作，转弯结束后自动选单边。
 * RULE_EDGE：执行指定动作，结束后强制指定单边。
 * RULE_RA：直角方向自动，不开启单边。
 * RULE_RA_AUTO：直角方向自动，转完后自动选单边。
 * RULE_RA_EDGE：直角方向自动，转完后强制指定单边。
 * 直角类型：1=右直角，2=左直角。普通路口类型：3/4/5。 */
static const IntersectionRule user_rules[] = {
    /* 如果某个直角出弯后需要单边，就在这里插入：
     * RULE_RA_AUTO(第几次, 1u或2u, 持续时间),
     * 例如：RULE_RA_AUTO(2u, 1u, 500u), */

    /* 当前最快路线：
     * 右直角 -> 右直角 -> 左直角 -> 4右 -> 5左 -> 5右 -> 4右 -> 4右
     * -> 5左 -> 3左 -> 3直行 -> 5右 -> 右直角 -> 右直角后单边。 */
    RULE_RA(  1u, 1u),
    RULE(     1u, 4u, ACT_RIGHT),
    RULE_RA(  1u, 2u),
    RULE(     1u, 5u, ACT_LEFT),
    RULE(     2u, 5u, ACT_RIGHT),
    RULE(     2u, 4u, ACT_RIGHT),
    RULE(     3u, 4u, ACT_RIGHT),
    RULE(     3u, 5u, ACT_LEFT),
    RULE(     4u, 5u, ACT_LEFT),
    RULE(     1u, 3u, ACT_LEFT),
    RULE(     2u, 3u, ACT_STRAIGHT),
    RULE(     5u, 5u, ACT_RIGHT),
    RULE_RA(  2u, 1u),
    RULE_RA(  3u, 1u),
    RULE(     4u, 4u, ACT_STRAIGHT),
    RULE(     5u, 4u, ACT_STRAIGHT),
    RULE_RA(  4u, 1u),
};
#define USER_RULE_COUNT (sizeof(user_rules) / sizeof(user_rules[0]))

static uint8 s_inter_count[7] = {0u};
static uint8 s_rules_done = 0u;
static uint16 s_rules_done_timer = 0u;
uint8 route_dbg_step = 0u;
uint8 route_dbg_total = (uint8)USER_RULE_COUNT;
uint8 route_dbg_flag = 0u;
uint8 route_dbg_count = 0u;
uint8 route_dbg_action = ACT_STRAIGHT;
static uint8 s_route_pending_valid = 0u;
static uint8 s_route_pending_flag = 0u;
static uint8 s_route_pending_count = 0u;
static uint8 s_route_pending_action = ACT_STRAIGHT;

static void ra_debug_update(void);

typedef struct
{
    uint8 need_pid;
    uint8 should_return;
    float speed_scale;
} RaResult;

typedef struct
{
    uint8 action;
    uint8 post_edge_side;
    uint16 post_edge_ms;
    uint8 valid;
} RouteDecision;

typedef struct
{
    float kp_scale;
    float kd_scale;
    float ff_scale;
    float slew_max;
} SteerSchedule;

static float abs_f(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static float clamp_f(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static float lerp_f(float a, float b, float t)
{
    return a + (b - a) * t;
}

static float range_ratio_i16(int16 value, int16 start, int16 end)
{
    if (end <= start)
        return (value >= end) ? 1.0f : 0.0f;

    if (value <= start) return 0.0f;
    if (value >= end) return 1.0f;

    return (float)(value - start) / (float)(end - start);
}

static SteerSchedule steer_schedule_calc(int16 pos_err_abs)
{
    SteerSchedule s;
    int16 la_abs = abs_i16(g_tf.lookahead_error);
    int16 trend_abs = abs_i16(g_tf.error_trend);
    int16 curve_signal = pos_err_abs;

    if (la_abs > curve_signal) curve_signal = la_abs;
    if (trend_abs > curve_signal) curve_signal = trend_abs;

    float speed_t = range_ratio_i16((int16)base_speed,
                                    STEER_GAIN_SPEED_START,
                                    STEER_GAIN_SPEED_END);
    float curve_t = range_ratio_i16(curve_signal,
                                    STEER_GAIN_CURVE_T1,
                                    STEER_GAIN_CURVE_T2);
    float kp_fast = lerp_f(1.0f, (float)STEER_FAST_KP_PCT * 0.01f, speed_t);
    float kd_fast = lerp_f(1.0f, (float)STEER_FAST_KD_PCT * 0.01f, speed_t);

    s.kp_scale = lerp_f(kp_fast,
                        (float)STEER_CURVE_KP_PCT * 0.01f,
                        curve_t);
    s.kd_scale = lerp_f(kd_fast,
                        (float)STEER_CURVE_KD_PCT * 0.01f,
                        curve_t);
    s.ff_scale = range_ratio_i16((int16)base_speed,
                                 STEER_FF_SPEED_START,
                                 STEER_FF_SPEED_END) *
                 ((float)steer_ff_k * 0.01f);
    s.slew_max = STEER_SLEW_MAX * lerp_f(0.85f, 1.20f, curve_t);

    return s;
}

static int16 abs_i16(int16 v)
{
    return (v >= 0) ? v : (int16)(-v);
}

static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float ra_yaw_progress(void)
{
    float delta = normalize_angle(yaw_angle);

    if (s_ra_dir == 2u)
        delta = -delta;

    return (delta > 0.0f) ? delta : 0.0f;
}

static void ra_debug_update(void)
{
    float yaw_progress = 0.0f;

    if (s_ra_state == RA_ST_ACTIVE && s_ra_dir != 0u)
        yaw_progress = ra_yaw_progress();

    ra_dbg_state = (uint8)s_ra_state;
    ra_dbg_phase = (uint8)s_ra_phase;
    ra_dbg_dir = s_ra_dir;
    ra_dbg_ip_row = s_ra_ip_row;
    ra_dbg_timer = s_ra_timer;
    ra_dbg_hard_cnt = s_ra_hard_cnt;
    ra_dbg_exit_good_cnt = (s_ra_phase == RA_PH_RECOVER) ?
                           s_ra_recover_good_cnt : s_ra_exit_good_cnt;
    ra_dbg_yaw10 = (int16)(yaw_progress * 10.0f);
}

static int16 clamp_duty(float val)
{
    if (val != val) return 0;
    if (val > MAX_DUTY) val = MAX_DUTY;
    else if (val < -MAX_DUTY) val = -MAX_DUTY;
    return (int16)val;
}

static void ra_reset(void)
{
    s_ra_state = RA_ST_NONE;
    s_ra_phase = RA_PH_WAIT;
    s_ra_dir = 0u;
    s_ra_orig_flag = 0u;
    s_ra_ip_row = 0u;
    s_ra_straight = 0u;
    s_ra_post_edge_side = EDGE_BOTH;
    s_ra_post_edge_ms = 0u;
    s_ra_exit_good_cnt = 0u;
    s_ra_recover_good_cnt = 0u;
    s_ra_approach_cnt = 0u;
    s_ra_timer = 0u;
    s_ra_hard_cnt = 0u;
    s_ra_recover_cnt = 0u;
    s_ra_phase_cnt = 0u;
    s_route_pending_valid = 0u;
    ra_debug_update();
}

static void update_rules_done(void)
{
    uint8 all_done = 1u;

    for (uint8 i = 0u; i < USER_RULE_COUNT; i++)
    {
        uint8 flag = user_rules[i].flag;
        if (flag >= 7u || s_inter_count[flag] < user_rules[i].count)
        {
            all_done = 0u;
            break;
        }
    }

    if (all_done)
        s_rules_done = 1u;
}

static void route_debug_reset(void)
{
    route_dbg_step = 0u;
    route_dbg_total = (uint8)USER_RULE_COUNT;
    route_dbg_flag = 0u;
    route_dbg_count = 0u;
    route_dbg_action = ACT_STRAIGHT;
    s_route_pending_valid = 0u;
    s_route_pending_flag = 0u;
    s_route_pending_count = 0u;
    s_route_pending_action = ACT_STRAIGHT;
}

static void route_debug_commit(void)
{
    if (!s_route_pending_valid)
        return;

    if (route_dbg_step < route_dbg_total)
        route_dbg_step++;
    route_dbg_flag = s_route_pending_flag;
    route_dbg_count = s_route_pending_count;
    route_dbg_action = s_route_pending_action;

    s_route_pending_valid = 0u;
}

void line_pid_reset_derivative(void)
{
    s_steer_d_reset_flag = 1u;
    s_prev_steer_output = 0.0f;
    s_steer_ff_filtered = 0.0f;
    s_steer_filtered_err = 0.0f;
}

static void reset_speed_planner(void)
{
    s_base_speed_ramped = 0;
    s_straight_cnt = 0u;
    s_straight_active = 0u;
}

static void reset_speed_ff_state(void)
{
    s_prev_target_speed = 0.0f;
    s_speed_ff_ready = 0u;
}

static void turn_shield_reset(void)
{
    s_turn_shield_frames = 0u;
    s_turn_shield_dist = 0u;
}

static uint8 route_has_next_match(uint8 flag)
{
    uint8 next_count;

    if (flag >= 7u)
        return 0u;

    next_count = s_inter_count[flag] + 1u;

    for (uint8 i = 0u; i < USER_RULE_COUNT; i++)
    {
        if (user_rules[i].flag == flag &&
            user_rules[i].count == next_count)
        {
            return 1u;
        }
    }

    return 0u;
}

static uint8 ra_fallback_direct_enabled(uint8 flag)
{
#if RA_FALLBACK_DIRECT_ENABLE
    return (flag == 1u || flag == 2u) ? 1u : 0u;
#else
    (void)flag;
    return 0u;
#endif
}

static uint8 ra_accept_near_flag(uint8 flag)
{
    if (route_has_next_match(flag))
        return 1u;

    return ra_fallback_direct_enabled(flag);
}

static void lost_search_reset(void)
{
    s_lost_search_active = 0u;
    s_lost_line_cnt = 0u;
    s_lost_search_cnt = 0u;
}

static uint8 lost_search_pick_dir(int16 err)
{
    if (err > LOST_SEARCH_ERR_DEADZONE)
        return 1u;
    if (err < -LOST_SEARCH_ERR_DEADZONE)
        return 2u;

    return (s_lost_search_dir == 2u) ? 2u : 1u;
}

static uint8 lost_search_step(int16 pos_err)
{
    if (g_tf.line_lost == 0u &&
        g_tf.valid_row_count >= LOST_SEARCH_EXIT_VALID_ROWS)
    {
        s_lost_last_err = pos_err;
        lost_search_reset();
        return 0u;
    }

    if (s_ra_state != RA_ST_NONE)
    {
        lost_search_reset();
        return 0u;
    }

    if (g_ra_flag != 0u || g_ra_pre_flag != 0u)
        return 0u;

    if (g_tf.line_lost == 0u)
    {
        s_lost_line_cnt = 0u;
        return 0u;
    }

    if (s_lost_line_cnt < 255u)
        s_lost_line_cnt++;

    if (s_lost_line_cnt < LOST_SEARCH_ENTER_FRAMES)
        return 0u;

    if (!s_lost_search_active)
    {
        s_lost_search_active = 1u;
        s_lost_search_cnt = 0u;
        s_lost_search_dir = lost_search_pick_dir(s_lost_last_err);
    }

    s_lost_search_cnt++;
    if (s_lost_search_cnt >= LOST_SEARCH_SWITCH_FRAMES)
    {
        s_lost_search_cnt = 0u;
        s_lost_search_dir = (s_lost_search_dir == 1u) ? 2u : 1u;
    }

    base_speed = 0;
    s_speed_integral = 0.0f;
    reset_speed_planner();
    reset_speed_ff_state();
    line_pid_reset_derivative();

    if (s_lost_search_dir == 1u)
    {
        small_driver_set_duty(clamp_duty(-LOST_SEARCH_DUTY),
                              clamp_duty(LOST_SEARCH_DUTY));
    }
    else
    {
        small_driver_set_duty(clamp_duty(LOST_SEARCH_DUTY),
                              clamp_duty(-LOST_SEARCH_DUTY));
    }

    return 1u;
}

static void single_edge_reset(void)
{
    s_edge_active = 0u;
    s_edge_side = EDGE_BOTH;
    s_edge_cnt = 0u;
    s_edge_target = 0u;
    g_post_edge_side = EDGE_BOTH;
}

void start_single_edge(uint8 side, uint16 duration_ms)
{
    if ((side != EDGE_LEFT && side != EDGE_RIGHT) || duration_ms == 0u)
    {
        single_edge_reset();
        return;
    }

    s_edge_active = 1u;
    s_edge_side = side;
    s_edge_cnt = 0u;
    s_edge_target = (uint16)((duration_ms + 10u) / 11u);
    if (s_edge_target == 0u)
        s_edge_target = 1u;

    g_post_edge_side = side;
}

static void single_edge_tick(void)
{
    if (!s_edge_active)
        return;

    if (g_post_edge_side != s_edge_side)
        g_post_edge_side = s_edge_side;

    s_edge_cnt++;
    if (s_edge_cnt >= s_edge_target)
        single_edge_reset();
}

static void turn_shield_start(void)
{
    s_turn_shield_frames = 1u;
    s_turn_shield_dist = 0u;
    g_ra_flag = 0u;
    g_ra_pre_flag = 0u;
}

static uint16 avg_wheel_speed_abs(void)
{
    uint16 l = (uint16)abs_i16(motor_value.receive_left_speed_data);
    uint16 r = (uint16)abs_i16(motor_value.receive_right_speed_data);
    return (uint16)(((uint32)l + (uint32)r) / 2u);
}

static void turn_shield_update(void)
{
    if (s_turn_shield_frames == 0u)
        return;

    s_turn_shield_dist += (uint32)avg_wheel_speed_abs();

    if (g_ra_flag != 0u &&
        s_turn_shield_frames >= TURN_SHIELD_NEAR_ALLOW_FRAMES &&
        ra_accept_near_flag((uint8)g_ra_flag))
    {
        g_ra_pre_flag = 0u;
        turn_shield_reset();
        return;
    }

    if (g_ra_flag != 0u || g_ra_pre_flag != 0u)
    {
        g_ra_flag = 0u;
        g_ra_pre_flag = 0u;
    }

    if (s_turn_shield_frames >= TURN_SHIELD_MAX_FRAMES ||
        (s_turn_shield_frames >= TURN_SHIELD_MIN_FRAMES &&
         s_turn_shield_dist >= TURN_SHIELD_DIST_SUM))
    {
        turn_shield_reset();
        return;
    }

    s_turn_shield_frames++;
}

static void ra_finish_ex(uint8 keep_flag, uint8 use_shield)
{
    uint8 edge_side = EDGE_BOTH;

    turn_right_led_off();
    g_ra_flag = keep_flag;
    s_speed_integral = 0.0f;
    line_pid_reset_derivative();
    route_debug_commit();
    update_rules_done();

    if (s_ra_post_edge_side == EDGE_LEFT || s_ra_post_edge_side == EDGE_RIGHT)
    {
        edge_side = s_ra_post_edge_side;
    }
    else if (s_ra_post_edge_side == EDGE_AUTO)
    {
        if (s_ra_dir == 1u)
            edge_side = EDGE_LEFT;
        else if (s_ra_dir == 2u)
            edge_side = EDGE_RIGHT;
    }

    if (edge_side != EDGE_BOTH && s_ra_post_edge_ms > 0u)
        start_single_edge(edge_side, s_ra_post_edge_ms);

    if (use_shield)
        turn_shield_start();
    else
        g_ra_pre_flag = 0u;
    ra_reset();
}

static void ra_finish(void)
{
    ra_finish_ex(0u, 1u);
}

static void ra_enter_recover(void)
{
    turn_right_led_off();
    g_ra_flag = 0u;
    s_ra_phase = RA_PH_RECOVER;
    s_ra_phase_cnt = 0u;
    s_ra_recover_cnt = 0u;
    s_ra_recover_good_cnt = 0u;
    s_speed_integral = 0.0f;
    line_pid_reset_derivative();
    reset_speed_planner();
    ra_debug_update();
}

static void ra_start(uint8 dir, uint8 orig_flag, uint8 straight,
                     uint8 post_edge_side, uint16 post_edge_ms)
{
    s_ra_dir = dir;
    s_ra_orig_flag = orig_flag;
    s_ra_state = RA_ST_ACTIVE;
    s_ra_phase = straight ? RA_PH_SLOW : RA_PH_WAIT;
    s_ra_ip_row = g_ip_max_row;
    s_ra_straight = straight;
    s_ra_post_edge_side = post_edge_side;
    s_ra_post_edge_ms = post_edge_ms;
    s_ra_exit_good_cnt = 0u;
    s_ra_recover_good_cnt = 0u;
    s_ra_approach_cnt = 0u;
    s_ra_timer = 0u;
    s_ra_hard_cnt = 0u;
    s_ra_recover_cnt = 0u;
    s_ra_phase_cnt = 0u;
    s_speed_integral = 0.0f;
    reset_speed_planner();
    lost_search_reset();

    if (!straight)
        turn_right_led_on();

    ra_debug_update();
}

static void ra_enter_hard(void)
{
    if (s_ra_phase == RA_PH_HARD)
        return;

    s_ra_phase = RA_PH_HARD;
    s_ra_phase_cnt = 0u;
    s_ra_hard_cnt = 0u;
    s_ra_exit_good_cnt = 0u;
    s_ra_recover_good_cnt = 0u;
    s_ra_recover_cnt = 0u;
    s_speed_integral = 0.0f;
    line_pid_reset_derivative();

    if (imu_ready && !imu_error)
        imu_reset_yaw();

    ra_debug_update();
}

void line_pid_init(void)
{
    s_steer_last_pos_err = 0.0f;
    s_steer_last_raw_err = 0.0f;
    s_prev_steer_output = 0.0f;
    s_steer_ff_filtered = 0.0f;
    s_steer_filtered_err = 0.0f;
    s_steer_d_reset_flag = 1u;
    s_speed_integral = 0.0f;
    s_motor_run_counter = 0u;
    s_rules_done = 0u;
    s_rules_done_timer = 0u;
    turn_shield_reset();
    single_edge_reset();
    lost_search_reset();
    s_lost_last_err = 0;
    s_lost_search_dir = 1u;
    reset_speed_planner();
    reset_speed_ff_state();
    route_debug_reset();

    ra_reset();

    for (uint8 i = 0u; i < 7u; i++)
        s_inter_count[i] = 0u;
}

static float steer_pd_calc(int16 pos_err,
                           float kp_scale,
                           float kd_scale,
                           float feedforward,
                           float slew_max)
{
    s_steer_filtered_err = s_steer_filtered_err * ERROR_FILTER_ALPHA +
                           (float)pos_err * (1.0f - ERROR_FILTER_ALPHA);

    float err = s_steer_filtered_err;
    float err_abs = abs_f(err);
    float raw_err = (float)pos_err;

    if (err_abs <= (float)STEER_DEADZONE && abs_f(feedforward) <= 1.0f)
    {
        s_steer_last_pos_err = err;
        s_steer_last_raw_err = raw_err;
        s_prev_steer_output *= 0.5f;
        return 0.0f;
    }

    float p_scale = 1.0f;
    if (err_abs <= (float)STEER_DEADZONE)
    {
        p_scale = 0.0f;
    }
    else if (err_abs < (float)STEER_SOFT_END)
    {
        float t = (err_abs - (float)STEER_DEADZONE) /
                  ((float)STEER_SOFT_END - (float)STEER_DEADZONE);
        p_scale = t * t;
    }

    float p_out = STEER_KP * kp_scale * err * p_scale;
    float d_out = 0.0f;

    if (s_steer_d_reset_flag == 0u)
        d_out = STEER_KD * kd_scale * (raw_err - s_steer_last_raw_err);
    else
        s_steer_d_reset_flag = 0u;

    s_steer_last_pos_err = err;
    s_steer_last_raw_err = raw_err;

    float output = p_out + d_out + feedforward;

    if (output > STEER_MAX) output = STEER_MAX;
    else if (output < -STEER_MAX) output = -STEER_MAX;

    float delta = output - s_prev_steer_output;
    if (delta > slew_max)
        output = s_prev_steer_output + slew_max;
    else if (delta < -slew_max)
        output = s_prev_steer_output - slew_max;

    s_prev_steer_output = output;
    return output;
}

static float speed_pi_calc(float target, float actual, float *integral, int16 pos_err_abs)
{
    float speed_err = target - actual;

    if (pos_err_abs < SPEED_I_SEPARATION)
    {
        *integral += speed_err;

        if (*integral > SPEED_I_MAX) *integral = SPEED_I_MAX;
        else if (*integral < -SPEED_I_MAX) *integral = -SPEED_I_MAX;
    }

    return SPEED_KP * speed_err + SPEED_KI * (*integral);
}

static int16 calc_adapted_speed(int16 base, int16 pos_err_abs)
{
    int16 t1 = sp_err_t1;
    int16 t2 = sp_err_t2;
    int16 r1 = sp_ratio_1;
    int16 r2 = sp_ratio_2;

    if (t2 <= t1)
        t2 = t1 + 1;

    if (pos_err_abs <= t1)
        return (int16)((int32)base * r1 / 100);

    if (pos_err_abs >= t2)
        return (int16)((int32)base * r2 / 100);

    int32 ratio = (int32)r1 + ((int32)(r2 - r1) * (pos_err_abs - t1)) / (t2 - t1);
    return (int16)((int32)base * ratio / 100);
}

static uint8 straight_speed_candidate(int16 pos_err_abs)
{
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < SPEED_STRAIGHT_VALID_ROWS)
        return 0u;
    if (pos_err_abs > SPEED_STRAIGHT_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > SPEED_STRAIGHT_LOOKAHEAD_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > SPEED_STRAIGHT_TREND_MAX)
        return 0u;
    if (g_ra_pre_flag != 0u || g_ra_flag != 0u)
        return 0u;

    return 1u;
}

static int16 speed_ramp_apply(int16 target)
{
    if (target < 0)
        target = 0;

    if (s_base_speed_ramped <= 0)
    {
        s_base_speed_ramped = target;
        return s_base_speed_ramped;
    }

    if (target > s_base_speed_ramped + SPEED_RAMP_UP_STEP)
        s_base_speed_ramped += SPEED_RAMP_UP_STEP;
    else if (target < s_base_speed_ramped - SPEED_RAMP_DOWN_STEP)
        s_base_speed_ramped -= SPEED_RAMP_DOWN_STEP;
    else
        s_base_speed_ramped = target;

    return s_base_speed_ramped;
}

static int16 apply_speed_pct(int16 target, int16 pct)
{
    if (pct < 0) pct = 0;
    if (pct > 120) pct = 120;

    return (int16)((int32)target * pct / 100);
}

static int16 calc_signal_speed_pct(int16 signal, int16 t1, int16 t2, int16 min_pct)
{
    if (t2 <= t1)
        t2 = t1 + 1;

    if (signal <= t1)
        return 100;

    if (signal >= t2)
        return min_pct;

    return (int16)(100 - ((int32)(100 - min_pct) * (signal - t1)) / (t2 - t1));
}

static int16 plan_lookahead_speed(int16 target, int16 pos_err_abs)
{
    int16 la_abs = abs_i16(g_tf.lookahead_error);
    int16 trend_abs = abs_i16(g_tf.error_trend);
    int16 la_pct = calc_signal_speed_pct(la_abs,
                                         SPEED_LOOKAHEAD_SLOW_T1,
                                         SPEED_LOOKAHEAD_SLOW_T2,
                                         SPEED_LOOKAHEAD_MIN_PCT);
    int16 trend_pct = calc_signal_speed_pct(trend_abs,
                                            SPEED_TREND_SLOW_T1,
                                            SPEED_TREND_SLOW_T2,
                                            SPEED_TREND_MIN_PCT);
    int16 pct = (la_pct < trend_pct) ? la_pct : trend_pct;

    if (pos_err_abs > sp_err_t2)
        pct = (pct < sp_ratio_2) ? pct : sp_ratio_2;

    return apply_speed_pct(target, pct);
}

static float limit_normal_steer(float steer, float speed_out)
{
    int16 pct = STEER_DIFF_NORMAL_PCT;

    if (s_straight_active)
        pct = STEER_DIFF_STRAIGHT_PCT;

    if (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER)
        pct = STEER_DIFF_RECOVER_PCT;

    float limit = abs_f(speed_out) * (float)pct * 0.01f;
    limit = clamp_f(limit, STEER_DIFF_MIN_LIMIT, STEER_DIFF_MAX_LIMIT);

    return clamp_f(steer, -limit, limit);
}

static float recover_yaw_correction(void)
{
    float target;
    float yaw_err;
    float correction;
    float fade;

    if (s_ra_state != RA_ST_ACTIVE || s_ra_phase != RA_PH_RECOVER)
        return 0.0f;
    if (s_ra_dir == 0u || !imu_ready || imu_error)
        return 0.0f;

    /* Fade out yaw correction as camera tracking stabilizes. */
    if (s_ra_recover_good_cnt >= RA_RECOVER_CONFIRM_FRAMES)
        return 0.0f;

    fade = 1.0f - (float)s_ra_recover_good_cnt / (float)RA_RECOVER_CONFIRM_FRAMES;

    target = (s_ra_dir == 1u) ?
             RA_RECOVER_YAW_TARGET_DEG :
             -RA_RECOVER_YAW_TARGET_DEG;
    yaw_err = normalize_angle(target - yaw_angle);

    if (abs_f(yaw_err) <= RA_RECOVER_YAW_DEADZONE)
        return 0.0f;

    correction = -yaw_err * RA_RECOVER_YAW_KP * fade;
    return clamp_f(correction, -RA_RECOVER_YAW_MAX, RA_RECOVER_YAW_MAX);
}

static int16 plan_base_speed(int16 target, int16 pos_err_abs, uint8 pre_slow_active)
{
    s_straight_active = 0u;

    if (s_ra_state != RA_ST_NONE || pre_slow_active)
    {
        s_straight_cnt = 0u;
        return speed_ramp_apply(target);
    }

    if (g_sym_component_flag && g_tf.line_lost == 0u)
    {
        s_straight_cnt = 0u;
        s_straight_active = 1u;
        return speed_ramp_apply(target);
    }

    if (g_tf.line_lost != 0u)
    {
        s_straight_cnt = 0u;
        target = (int16)((int32)target * SPEED_LINE_LOST_PCT / 100);
        return speed_ramp_apply(target);
    }

    if (g_tf.valid_row_count < SPEED_VISION_BAD_VALID_ROWS)
    {
        s_straight_cnt = 0u;
        target = (int16)((int32)target * SPEED_VISION_BAD_PCT / 100);
        return speed_ramp_apply(target);
    }

    target = plan_lookahead_speed(target, pos_err_abs);

    if (straight_speed_candidate(pos_err_abs))
    {
        if (s_straight_cnt < 255u)
            s_straight_cnt++;

        if (s_straight_cnt >= SPEED_STRAIGHT_CONFIRM_FRAMES)
        {
            s_straight_active = 1u;
            target = (int16)((int32)target * SPEED_STRAIGHT_BOOST_PCT / 100);
        }
    }
    else
    {
        s_straight_cnt = 0u;
    }

    return speed_ramp_apply(target);
}

static uint8 route_action_from_flag(uint8 flag)
{
    if (flag == 3u)
        return ACT_LEFT;
    if (flag == 4u)
        return ACT_RIGHT;
    if (flag == 1u)
        return ACT_RIGHT;
    if (flag == 2u)
        return ACT_LEFT;

    return ACT_STRAIGHT;
}

static RouteDecision fallback_intersection_decision(uint8 flag)
{
    RouteDecision d = { ACT_STRAIGHT, EDGE_BOTH, 0u, 0u };

    d.action = route_action_from_flag(flag);
    return d;
}

static RouteDecision select_intersection_decision(uint8 flag)
{
    RouteDecision d = fallback_intersection_decision(flag);
    uint8 next_count;

    if (flag >= 7u)
        return d;

    next_count = s_inter_count[flag] + 1u;

    for (uint8 i = 0u; i < USER_RULE_COUNT; i++)
    {
        if (user_rules[i].flag == flag &&
            user_rules[i].count == next_count)
        {
            s_inter_count[flag] = next_count;

            d.action = (user_rules[i].action == ACT_AUTO) ?
                       route_action_from_flag(flag) :
                       user_rules[i].action;
            d.post_edge_side = user_rules[i].post_edge_side;
            d.post_edge_ms = user_rules[i].post_edge_ms;
            d.valid = 1u;

            s_route_pending_valid = 1u;
            s_route_pending_flag = flag;
            s_route_pending_count = next_count;
            s_route_pending_action = d.action;
            return d;
        }
    }

    if (ra_fallback_direct_enabled(flag))
    {
        d.action = route_action_from_flag(flag);
        d.post_edge_side = EDGE_BOTH;
        d.post_edge_ms = 0u;
        d.valid = 1u;
        return d;
    }

    /* 未匹配路线表时：直角1/2保底自动转，普通路口不消耗计数。 */
    return d;
}

static RaResult ra_state_machine_step(int16 pos_err_abs)
{
    RaResult r = { 0u, 0u, 1.0f };

    if ((g_ra_flag == 1u || g_ra_flag == 2u) && s_ra_state == RA_ST_NONE)
    {
        RouteDecision d = select_intersection_decision((uint8)g_ra_flag);
        uint8 action = (d.action == ACT_RIGHT || d.action == ACT_LEFT) ?
                       d.action :
                       route_action_from_flag((uint8)g_ra_flag);

        if (!d.valid)
        {
            g_ra_flag = 0u;
            ra_debug_update();
            return r;
        }

        ra_start((action == ACT_RIGHT) ? 1u : 2u,
                 (uint8)g_ra_flag,
                 0u,
                 d.post_edge_side,
                 d.post_edge_ms);
    }

    if ((g_ra_flag >= 3u && g_ra_flag <= 5u) && s_ra_state == RA_ST_NONE)
    {
        RouteDecision d = select_intersection_decision((uint8)g_ra_flag);

        if (!d.valid)
        {
            g_ra_flag = 0u;
            ra_debug_update();
            return r;
        }

        if (d.action == ACT_RIGHT || d.action == ACT_LEFT)
            ra_start((d.action == ACT_RIGHT) ? 1u : 2u, (uint8)g_ra_flag, 0u,
                     d.post_edge_side, d.post_edge_ms);
        else
            ra_start(0u, (uint8)g_ra_flag, 1u,
                     d.post_edge_side, d.post_edge_ms);
    }

    if (s_ra_state != RA_ST_ACTIVE)
    {
        ra_debug_update();
        return r;
    }

    s_ra_timer++;
    s_ra_phase_cnt++;

    if (g_ip_max_row > s_ra_ip_row)
        s_ra_ip_row = g_ip_max_row;

    if (s_ra_timer > RA_TIMEOUT_FRAMES)
    {
        ra_finish();
        return r;
    }

    if (s_ra_straight)
    {
        if (s_ra_timer >= RA_STRAIGHT_FRAMES)
        {
            ra_finish();
            return r;
        }

        r.speed_scale = (float)ra_slow_pct * 0.01f;
        r.need_pid = 1u;
        ra_debug_update();
        return r;
    }

    if (s_ra_phase == RA_PH_RECOVER)
    {
        uint8 recover_ok = (g_tf.line_lost == 0u &&
                            g_tf.valid_row_count >= RA_RECOVER_VALID_ROWS &&
                            pos_err_abs <= RA_RECOVER_ERROR_MAX &&
                            abs_i16(g_tf.lookahead_error) <= RA_RECOVER_LOOKAHEAD_MAX &&
                            abs_i16(g_tf.error_trend) <= RA_RECOVER_TREND_MAX) ? 1u : 0u;

        s_ra_recover_cnt++;

        if (g_ra_flag != 0u &&
            s_ra_recover_cnt >= RA_RECOVER_NEAR_DETECT_MIN_FRAMES &&
            ra_accept_near_flag((uint8)g_ra_flag))
        {
            uint8 next_flag = (uint8)g_ra_flag;
            ra_finish_ex(next_flag, 0u);
            r.speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f;
            r.need_pid = 1u;
            ra_debug_update();
            return r;
        }

        if (recover_ok) s_ra_recover_good_cnt++;
        else s_ra_recover_good_cnt = 0u;

        if ((s_ra_recover_cnt >= RA_RECOVER_MIN_FRAMES &&
             s_ra_recover_good_cnt >= RA_RECOVER_CONFIRM_FRAMES) ||
            s_ra_recover_cnt >= RA_RECOVER_MAX_FRAMES)
        {
            ra_finish();
            return r;
        }

        r.speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f;
        r.need_pid = 1u;
        ra_debug_update();
        return r;
    }

    if (s_ra_phase == RA_PH_WAIT)
    {
        if (s_ra_ip_row >= (uint8)ra_slow_row || s_ra_phase_cnt >= RA_WAIT_TIMEOUT)
        {
            s_ra_phase = RA_PH_SLOW;
            s_ra_phase_cnt = 0u;
            s_speed_integral = 0.0f;
        }
    }
    else if (s_ra_phase == RA_PH_SLOW)
    {
        if (s_ra_ip_row >= (uint8)ra_turn_row || s_ra_phase_cnt >= RA_SLOW_TIMEOUT)
        {
            s_ra_phase = RA_PH_APPROACH;
            s_ra_approach_cnt = 0u;
            s_ra_phase_cnt = 0u;
            s_speed_integral = 0.0f;
        }
    }
    else if (s_ra_phase == RA_PH_APPROACH)
    {
        s_ra_approach_cnt++;
        if (s_ra_approach_cnt >= (uint16)ra_approach_frames)
            ra_enter_hard();
    }

    if (s_ra_phase == RA_PH_HARD)
    {
        uint8 direct_fast = (s_ra_orig_flag < 3u &&
                             base_speed >= RA_FAST_SPEED_START) ? 1u : 0u;
        uint8 min_hard = (s_ra_orig_flag >= 3u) ?
                         RA_CROSS_HARD_MIN_FRAMES : RA_HARD_MIN_FRAMES;
        uint8 hard_limit = (s_ra_orig_flag >= 3u) ?
                           RA_CROSS_HARD_TIMEOUT :
                           (direct_fast ? RA_FAST_HARD_TIMEOUT : RA_HARD_TIMEOUT);
        uint8 line_ok = (g_tf.line_lost == 0u &&
                         g_tf.valid_row_count >= RA_EXIT_VALID_ROWS &&
                         pos_err_abs <= RA_EXIT_ERROR_MAX) ? 1u : 0u;

        s_ra_hard_cnt++;

        if (line_ok) s_ra_exit_good_cnt++;
        else s_ra_exit_good_cnt = 0u;

        uint8 yaw_done = 0u;
        float hard_yaw_target = (s_ra_orig_flag >= 3u) ?
                                RA_CROSS_HARD_YAW_DEG :
                                (direct_fast ? RA_FAST_DIRECT_YAW_DEG : (float)ra_hard_yaw);
        float yaw_progress = ra_yaw_progress();
        uint8 camera_done = (s_ra_hard_cnt >= min_hard &&
                             s_ra_exit_good_cnt >= RA_EXIT_CONFIRM_FRAMES) ? 1u : 0u;

        if (direct_fast && imu_ready && !imu_error &&
            yaw_progress < RA_FAST_CAMERA_MIN_YAW)
        {
            camera_done = 0u;
        }

        if (imu_ready && !imu_error && hard_yaw_target > 0.0f &&
            s_ra_hard_cnt >= min_hard &&
            yaw_progress >= hard_yaw_target)
        {
            yaw_done = 1u;
        }

        if (hard_limit < min_hard)
            hard_limit = min_hard;

        uint8 hard_timeout = (s_ra_hard_cnt > hard_limit) ? 1u : 0u;

        if (camera_done || yaw_done || hard_timeout)
        {
            ra_enter_recover();
            return r;
        }

        float outer = (float)ra_hard_outer;
        float inner = (float)ra_hard_inner;
        float out_l;
        float out_r;

        if (s_ra_dir == 1u)
        {
            out_l = inner;
            out_r = outer;
        }
        else
        {
            out_l = outer;
            out_r = inner;
        }

        small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r));
        r.should_return = 1u;
        ra_debug_update();
        return r;
    }

    if (s_ra_phase == RA_PH_SLOW || s_ra_phase == RA_PH_APPROACH)
        r.speed_scale = (float)ra_slow_pct * 0.01f;

    r.need_pid = 1u;
    ra_debug_update();
    return r;
}

static void normal_pid_step(int16 pos_err, int16 pos_err_abs)
{
    SteerSchedule sch = steer_schedule_calc(pos_err_abs);
    float ff_raw = STEER_KP * sch.ff_scale * (float)g_tf.lookahead_error;
    float steer_ff = 0.0f;

    ff_raw = clamp_f(ff_raw, -STEER_FF_MAX, STEER_FF_MAX);
    s_steer_ff_filtered = s_steer_ff_filtered * STEER_FF_FILTER_ALPHA +
                          ff_raw * (1.0f - STEER_FF_FILTER_ALPHA);

    if (g_ra_pre_flag == 0u && g_ra_flag == 0u)
        steer_ff = s_steer_ff_filtered;

    float steer = steer_pd_calc(pos_err,
                                sch.kp_scale,
                                sch.kd_scale,
                                steer_ff,
                                sch.slew_max);

#if YAW_COMP_ENABLE
    {
        float yaw_kp_val = (float)yaw_kp / 10.0f;
        float yaw_abs = abs_f(yaw_angle);
        if (yaw_abs > YAW_DEADZONE)
            steer += yaw_kp_val * yaw_angle;
    }
#endif

    int16 trend_abs = abs_i16(g_tf.error_trend);
    int16 speed_err = pos_err_abs;

    if (base_speed > 200)
    {
        float trend_factor = (float)(base_speed - 200) / 800.0f;
        if (trend_factor > 0.5f) trend_factor = 0.5f;
        speed_err = pos_err_abs + (int16)((float)trend_abs * trend_factor);
    }

    float target_speed = (float)calc_adapted_speed(base_speed, speed_err);
    float actual_l = (float)motor_value.receive_left_speed_data;
    float actual_r = (float)motor_value.receive_right_speed_data;
    float avg_actual = (actual_l + actual_r) * 0.5f;
    float accel_ff = 0.0f;
    if (s_speed_ff_ready)
        accel_ff = (target_speed - s_prev_target_speed) * SPEED_ACCEL_FF_GAIN;
    else
        s_speed_ff_ready = 1u;

    accel_ff = clamp_f(accel_ff, -SPEED_ACCEL_FF_LIMIT, SPEED_ACCEL_FF_LIMIT);
    s_prev_target_speed = target_speed;

    float speed_ff = target_speed * SPEED_FF_RATIO + accel_ff;
    float speed_out = speed_ff + speed_pi_calc(target_speed - speed_ff,
                                               avg_actual,
                                               &s_speed_integral,
                                               pos_err_abs);

    float speed_factor = 1.0f + (float)base_speed * (float)steer_speed_k * 0.001f;
    if (speed_factor > SPEED_FACTOR_MAX)
        speed_factor = SPEED_FACTOR_MAX;

    steer *= speed_factor;

    if (s_straight_active)
        steer *= (float)SPEED_STRAIGHT_STEER_PCT * 0.01f;

    if (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER)
    {
        steer *= (float)RA_RECOVER_STEER_PCT * 0.01f;
        steer += recover_yaw_correction();
    }

    steer = limit_normal_steer(steer, speed_out);

    small_driver_set_duty(clamp_duty(speed_out + steer),
                          clamp_duty(speed_out - steer));
}

void line_pid_control(void)
{
    if (motor_enable == 0)
    {
        small_driver_set_duty(0, 0);
        base_speed = 0;
        s_speed_integral = 0.0f;
        s_motor_run_counter = 0u;
        turn_shield_reset();
        single_edge_reset();
        lost_search_reset();
        reset_speed_planner();
        reset_speed_ff_state();
        ra_reset();
        return;
    }

    s_motor_run_counter++;
    if (s_motor_run_counter >= (uint32)motor_run_time * 1000u / 11u)
    {
        motor_enable = 0;
        small_driver_set_duty(0, 0);
        base_speed = 0;
        s_speed_integral = 0.0f;
        s_motor_run_counter = 0u;
        turn_shield_reset();
        single_edge_reset();
        lost_search_reset();
        reset_speed_planner();
        reset_speed_ff_state();
        ra_reset();
        return;
    }

    if (s_rules_done)
    {
        s_rules_done_timer++;
        if (s_rules_done_timer >= RULES_DONE_DELAY)
        {
            motor_enable = 0;
            small_driver_set_duty(0, 0);
            base_speed = 0;
            s_speed_integral = 0.0f;
            s_motor_run_counter = 0u;
            s_rules_done = 0u;
            s_rules_done_timer = 0u;
            turn_shield_reset();
            single_edge_reset();
            lost_search_reset();
            reset_speed_planner();
            reset_speed_ff_state();
            ra_reset();
            return;
        }
    }

    turn_shield_update();

    int16 pos_err = g_tf.error;
    int16 pos_err_abs = abs_i16(pos_err);

    RaResult ra = ra_state_machine_step(pos_err_abs);
    if (ra.should_return)
        return;

    if (lost_search_step(pos_err))
        return;

    int16 target_base_speed = (int16)((float)motor_speed * 8.0f * ra.speed_scale);
    uint8 pre_slow_active = 0u;

    if (s_ra_state == RA_ST_NONE)
    {
        static uint8 s_pre_lock = 0u;
        static uint8 s_pre_timeout = 0u;

        if (s_turn_shield_frames != 0u)
        {
            s_pre_lock = 0u;
            s_pre_timeout = 0u;
        }

        if (g_ra_pre_flag && g_ra_flag == 0u)
        {
            s_pre_lock = 1u;
            s_pre_timeout = 0u;
        }

        if (g_ra_flag != 0u)
            s_pre_lock = 0u;

        if (g_sym_component_flag)
        {
            s_pre_lock = 0u;
            s_pre_timeout = 0u;
        }

        if (s_pre_lock)
        {
            pre_slow_active = 1u;
            target_base_speed = (int16)((int32)target_base_speed * ra_slow_pct / 100);

            if (!g_ra_pre_flag)
            {
                s_pre_timeout++;
                if (s_pre_timeout > 30u)
                {
                    s_pre_lock = 0u;
                    s_pre_timeout = 0u;
                }
            }
        }
    }

    base_speed = plan_base_speed(target_base_speed, pos_err_abs, pre_slow_active);

    single_edge_tick();
    normal_pid_step(pos_err, pos_err_abs);
}
