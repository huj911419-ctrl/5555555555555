#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

extern int16 base_speed;

/* Runtime menu variables, defined in Menu.c. */
extern int16 motor_speed;
extern int16 pid_kp;
extern int16 pid_ki;
extern int16 pid_kd;
extern int16 yaw_kp;

extern int16 ra_hard_inner;
extern int16 ra_hard_outer;
extern int16 ra_hard_yaw;
extern int16 ra_slow_row;
extern int16 ra_slow_pct;
extern int16 ra_turn_row;
extern int16 ra_approach_frames;

extern int16 sp_err_t1;
extern int16 sp_err_t2;
extern int16 sp_ratio_1;
extern int16 sp_ratio_2;
extern int16 steer_speed_k;
extern int16 steer_ff_k;

extern uint8 ra_dbg_state;
extern uint8 ra_dbg_phase;
extern uint8 ra_dbg_dir;
extern uint8 ra_dbg_ip_row;
extern uint16 ra_dbg_timer;
extern uint16 ra_dbg_hard_cnt;
extern uint8 ra_dbg_exit_good_cnt;
extern int16 ra_dbg_yaw10;
extern uint8 route_dbg_step;
extern uint8 route_dbg_total;
extern uint8 route_dbg_flag;
extern uint8 route_dbg_count;
extern uint8 route_dbg_action;

/* Steering PD. */
#define STEER_KP  ((float)pid_kp * 0.8f)
#define STEER_KD  ((float)pid_kd * 0.6f)
#define STEER_MAX 4000.0f
#define STEER_DEADZONE 1
#define STEER_SOFT_END 6
#define STEER_SLEW_MAX 250.0f
#define STEER_GAIN_SPEED_START 180
#define STEER_GAIN_SPEED_END 800
#define STEER_GAIN_CURVE_T1 10
#define STEER_GAIN_CURVE_T2 38
#define STEER_FAST_KP_PCT 75
#define STEER_FAST_KD_PCT 115
#define STEER_CURVE_KP_PCT 125
#define STEER_CURVE_KD_PCT 135
#define STEER_FF_SPEED_START 180
#define STEER_FF_SPEED_END 800
#define STEER_FF_MAX 650.0f
#define STEER_FF_FILTER_ALPHA 0.70f
#define STEER_DIFF_MIN_LIMIT 250.0f
#define STEER_DIFF_MAX_LIMIT 1800.0f
#define STEER_DIFF_NORMAL_PCT 85
#define STEER_DIFF_STRAIGHT_PCT 45
#define STEER_DIFF_RECOVER_PCT 110

/* Speed PI. */
#define SPEED_KP 0.5f
#define SPEED_KI ((float)pid_ki * 0.25f)
#define SPEED_I_MAX 500.0f
#define SPEED_I_SEPARATION 20
#define SPEED_FF_RATIO 0.4f
#define SPEED_ACCEL_FF_GAIN 0.35f
#define SPEED_ACCEL_FF_LIMIT 180.0f
#define SPEED_FACTOR_MAX 5.0f
#define SPEED_STRAIGHT_VALID_ROWS 42u
#define SPEED_STRAIGHT_ERR_MAX 4
#define SPEED_STRAIGHT_LOOKAHEAD_MAX 5
#define SPEED_STRAIGHT_TREND_MAX 5
#define SPEED_STRAIGHT_CONFIRM_FRAMES 6u
#define SPEED_STRAIGHT_BOOST_PCT 105
#define SPEED_STRAIGHT_STEER_PCT 85
#define SPEED_LOOKAHEAD_SLOW_T1 8
#define SPEED_LOOKAHEAD_SLOW_T2 28
#define SPEED_LOOKAHEAD_MIN_PCT 55
#define SPEED_TREND_SLOW_T1 6
#define SPEED_TREND_SLOW_T2 24
#define SPEED_TREND_MIN_PCT 60
#define SPEED_VISION_BAD_VALID_ROWS 22u
#define SPEED_VISION_BAD_PCT 35
#define SPEED_LINE_LOST_PCT 25
#define SPEED_RAMP_UP_STEP 25
#define SPEED_RAMP_DOWN_STEP 180

#define MAX_DUTY 5000.0f
#define ERROR_FILTER_ALPHA 0.60f

/* Keep normal-line yaw compensation disabled by default.
 * IMU is used below only as a hard-turn exit reference. */
#define YAW_COMP_ENABLE 0
#define YAW_DEADZONE 1.0f

/* RA state timing, one frame is 11 ms. */
#define RA_HARD_TIMEOUT          20u
#define RA_FAST_SPEED_START      520
#define RA_FAST_HARD_TIMEOUT     26u
#define RA_CROSS_HARD_TIMEOUT    32u
#define RA_TIMEOUT_FRAMES        150u
#define RA_WAIT_TIMEOUT          12u
#define RA_SLOW_TIMEOUT          24u
#define RA_HARD_MIN_FRAMES       14u
#define RA_CROSS_HARD_MIN_FRAMES 24u
#define RA_EXIT_VALID_ROWS       12u
#define RA_EXIT_ERROR_MAX        12
#define RA_EXIT_CONFIRM_FRAMES   6u
#define RA_RECOVER_MIN_FRAMES    15u
#define RA_RECOVER_MAX_FRAMES    55u
#define RA_RECOVER_SPEED_PCT     50
#define RA_RECOVER_STEER_PCT     70
#define RA_RECOVER_VALID_ROWS    25u
#define RA_RECOVER_ERROR_MAX     12
#define RA_RECOVER_LOOKAHEAD_MAX 14
#define RA_RECOVER_TREND_MAX     16
#define RA_RECOVER_CONFIRM_FRAMES 7u
#define RA_RECOVER_NEAR_DETECT_MIN_FRAMES 4u
#define RA_RECOVER_YAW_TARGET_DEG 90.0f
#define RA_RECOVER_YAW_KP         3.0f
#define RA_RECOVER_YAW_MAX        180.0f
#define RA_RECOVER_YAW_DEADZONE   6.0f
#define RA_FAST_DIRECT_YAW_DEG    82.0f
#define RA_FAST_CAMERA_MIN_YAW    65.0f
#define RA_CROSS_HARD_YAW_DEG     75.0f
#define RA_STRAIGHT_FRAMES       55u
#define TURN_SHIELD_MIN_FRAMES   3u
#define TURN_SHIELD_MAX_FRAMES   10u
#define TURN_SHIELD_DIST_SUM     1500u
#define TURN_SHIELD_NEAR_ALLOW_FRAMES 4u
#define RULES_DONE_DELAY         136u

/* 直角和丢线保底。 */
#define RA_FALLBACK_DIRECT_ENABLE 1u

#define LOST_SEARCH_ENTER_FRAMES 3u
#define LOST_SEARCH_SWITCH_FRAMES 45u
#define LOST_SEARCH_EXIT_VALID_ROWS 15u
#define LOST_SEARCH_DUTY 520.0f
#define LOST_SEARCH_ERR_DEADZONE 4

#define EDGE_BOTH  0u
#define EDGE_LEFT  1u
#define EDGE_RIGHT 2u
#define EDGE_AUTO  3u
#define SINGLE_EDGE_POST_TURN_MS 500u

extern uint8 g_post_edge_side;

void line_pid_init(void);
void line_pid_control(void);
void line_pid_reset_derivative(void);
void start_single_edge(uint8 side, uint16 duration_ms);

#endif /* CODE_PID_H_ */
