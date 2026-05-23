#include "Menu.h"
#include "Track_funsion.h"
#include "Pid.h"
#include "TFT_show_image.h"

#define KEY1 P11_3
#define KEY2 P11_2
#define KEY3 P20_6
#define KEY4 P20_7

#define TURN_RIGHT_LED P20_8
#define SWITCH1 P33_11
#define SWITCH2 P33_12

#define DEBOUNCE_CNT 20
#define RELEASE_CNT 10
#define MAIN_DRAW_DIV_STOP 1u
#define MAIN_DRAW_DIV_RUN  1u
#define TFT_OFF_WHEN_RUNNING 0u

int16 motor_speed = 140;
int16 motor_dir = 1;
int16 motor_enable = 0;
int16 motor_run_time = 60;

int16 cam_exposure = 200;

int16 pid_kp = 7;
int16 pid_ki = 2;
int16 pid_kd = 8;

int16 sp_err_t1 = 4;
int16 sp_err_t2 = 20;
int16 sp_ratio_1 = 100;
int16 sp_ratio_2 = 68;
int16 steer_speed_k = 3;
int16 steer_ff_k = 12;

int16 ra_hard_inner = -150;
int16 ra_hard_outer = 600;
int16 ra_hard_yaw = 68;//转弯角度越大
int16 ra_slow_row = 40;
int16 ra_slow_pct = 38;
int16 ra_turn_row = 80;
int16 ra_approach_frames = 10;//转弯延时函数

int16 yaw_kp = 0;

static MenuItem items_main[] = {
    {"Enable", &motor_enable, 0, 1, 1},
};

static MenuItem items_motor[] = {
    {"Speed", &motor_speed, 0, 600, 20},
    {"Dir", &motor_dir, 0, 1, 1},
};

static MenuItem items_cam[] = {
    {"Bias", &threshold_bias, -50, 50, 8},
    {"Expose", &cam_exposure, 100, 500, 10},
};

static MenuItem items_pid[] = {
    {"Kp", &pid_kp, 0, 100, 1},
    {"Ki", &pid_ki, 0, 50, 1},
    {"Kd", &pid_kd, 0, 50, 1},
};

static MenuItem items_speed[] = {
    {"StrErr", &sp_err_t1, 1, 50, 1},
    {"CrvErr", &sp_err_t2, 10, 80, 1},
    {"StrSpd%", &sp_ratio_1, 20, 100, 5},
    {"CrvSpd%", &sp_ratio_2, 20, 100, 5},
    {"SpdCpl", &steer_speed_k, 0, 50, 1},
    {"LaFF", &steer_ff_k, 0, 50, 1},
};

static MenuItem items_ra[] = {
    {"SlwRow", &ra_slow_row, 30, 110, 5},
    {"SlwPct", &ra_slow_pct, 10, 100, 5},
    {"TrnRow", &ra_turn_row, 50, 115, 5},
    {"AproF", &ra_approach_frames, 5, 100, 5},
    {"Outer", &ra_hard_outer, 500, 5000, 100},
    {"Inner", &ra_hard_inner, -2000, 5000, 100},
    {"Yaw", &ra_hard_yaw, 30, 85, 5},
    {"IpCol", &ip_col_offset, 0, 7, 1},
    {"IpL", &ip_left_col, 0, 47, 1},
    {"IpR", &ip_right_col, 47, 93, 1},
};

static MenuItem items_imu[] = {
    {"YAW Kp", &yaw_kp, 0, 100, 1},
};

static MenuPageDef g_pages[PAGE_MAX] = {
    { .title = "MAIN", .items = items_main, .item_count = 1, .draw = NULL },
    { .title = "MOTOR", .items = items_motor, .item_count = 2, .draw = NULL },
    { .title = "CAM", .items = items_cam, .item_count = 2, .draw = NULL },
    { .title = "PID", .items = items_pid, .item_count = 3, .draw = NULL },
    { .title = "SPEED", .items = items_speed, .item_count = 6, .draw = NULL },
    { .title = "RA", .items = items_ra, .item_count = 10, .draw = NULL },
    { .title = "IMU", .items = items_imu, .item_count = 1, .draw = NULL },
};

MenuPage now_page = PAGE_MAIN;
uint8 menu_cursor = 0u;

static uint8 dip_is_adjust_mode(void)
{
    uint8 sw1 = (gpio_get_level(SWITCH1) == 0) ? 1u : 0u;
    uint8 sw2 = (gpio_get_level(SWITCH2) == 0) ? 1u : 0u;
    return sw1 | sw2;
}

static void menu_apply_adjusted_value(void)
{
    if (now_page == PAGE_CAM && menu_cursor == 1u)
        mt9v03x_set_exposure_time((uint16)cam_exposure);
}

static uint8 key_scan(void)
{
    typedef enum
    {
        KEY_STATE_IDLE = 0,
        KEY_STATE_DEBOUNCE,
        KEY_STATE_HOLD,
        KEY_STATE_RELEASE,
    } KeyState;

    static KeyState state = KEY_STATE_IDLE;
    static uint16 cnt = 0u;
    static uint8 last_key = 0u;

    uint8 cur_key = 0u;

    if (gpio_get_level(KEY1) == 0)
        cur_key = 1u;
    else if (gpio_get_level(KEY2) == 0)
        cur_key = 2u;
    else if (gpio_get_level(KEY3) == 0)
        cur_key = 3u;
    else if (gpio_get_level(KEY4) == 0)
        cur_key = 4u;

    switch (state)
    {
    case KEY_STATE_IDLE:
        if (cur_key != 0u)
        {
            last_key = cur_key;
            cnt = 0u;
            state = KEY_STATE_DEBOUNCE;
        }
        break;

    case KEY_STATE_DEBOUNCE:
        if (cur_key == last_key)
        {
            cnt++;
            if (cnt >= DEBOUNCE_CNT)
            {
                cnt = 0u;
                state = KEY_STATE_HOLD;
                return last_key;
            }
        }
        else
        {
            state = KEY_STATE_IDLE;
            cnt = 0u;
        }
        break;

    case KEY_STATE_HOLD:
        if (cur_key == 0u)
        {
            cnt = 0u;
            state = KEY_STATE_RELEASE;
        }
        break;

    case KEY_STATE_RELEASE:
        if (cur_key == 0u)
        {
            cnt++;
            if (cnt >= RELEASE_CNT)
            {
                cnt = 0u;
                state = KEY_STATE_IDLE;
            }
        }
        else
        {
            state = KEY_STATE_HOLD;
            cnt = 0u;
        }
        break;

    default:
        state = KEY_STATE_IDLE;
        break;
    }

    return 0u;
}

static void cursor_clamp(void)
{
    uint8 max_items = g_pages[now_page].item_count;

    if (max_items == 0u)
        menu_cursor = 0u;
    else if (menu_cursor >= max_items)
        menu_cursor = max_items - 1u;
}

void key_init_all(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(TURN_RIGHT_LED, GPO, GPIO_HIGH, GPI_PULL_UP);
}

void key_process(void)
{
    uint8 key = key_scan();

    if (key == 0u)
        return;

    if (key == 1u)
    {
        now_page = (MenuPage)((now_page + 1u) % PAGE_MAX);
        menu_cursor = 0u;
#if !RACE_MODE
        tft180_clear();
#endif
        return;
    }

    if (key == 2u)
    {
        now_page = (now_page == 0u) ? (MenuPage)(PAGE_MAX - 1u) : (MenuPage)(now_page - 1u);
        menu_cursor = 0u;
#if !RACE_MODE
        tft180_clear();
#endif
        return;
    }

    uint8 adjust_mode = dip_is_adjust_mode();
    MenuPageDef *page = &g_pages[now_page];
    uint8 item_cnt = page->item_count;

    if (item_cnt == 0u)
        return;

    if (!adjust_mode)
    {
        if (key == 3u)
            menu_cursor = (menu_cursor == 0u) ? (item_cnt - 1u) : (menu_cursor - 1u);

        if (key == 4u)
            menu_cursor = (menu_cursor + 1u) % item_cnt;
    }
    else
    {
        cursor_clamp();
        MenuItem *cur = &page->items[menu_cursor];

        if (key == 3u)
        {
            if (*cur->value + cur->step <= cur->max)
                *cur->value += cur->step;
            else
                *cur->value = cur->max;
        }

        if (key == 4u)
        {
            if (*cur->value - cur->step >= cur->min)
                *cur->value -= cur->step;
            else
                *cur->value = cur->min;
        }

        menu_apply_adjusted_value();
    }
}

static uint8 append_str(char *dst, uint8 pos, const char *src)
{
    while (*src)
        dst[pos++] = *src++;

    return pos;
}

static uint8 append_int(char *dst, uint8 pos, int16 val)
{
    int32 v = val;
    char tmp[8];
    uint8 n = 0u;

    if (v < 0)
    {
        dst[pos++] = '-';
        v = -v;
    }

    do
    {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    } while (v > 0);

    for (uint8 i = n; i > 0u; i--)
        dst[pos++] = tmp[i - 1u];

    return pos;
}

static void default_draw(MenuPageDef *page)
{
    char buf[32];
    uint8 adjust_mode = dip_is_adjust_mode();

    if (now_page == PAGE_MAIN)
    {
        const char *short_label[1] = {"EN"};

        for (uint8 i = 0u; i < page->item_count && i < 1u; i++)
        {
            MenuItem *item = &page->items[i];
            uint8 y = 82u + i * 16u;

            tft180_show_string(52, y, (i == menu_cursor) ? ">" : " ");
            tft180_show_string(58, y, (char *)short_label[i]);
            tft180_show_string(70, y, ":");
            tft180_show_int(76, y, *item->value, 1u);
        }

        tft180_show_string(52, 98, "L:");
        tft180_show_int(64, 98, (int32)route_dbg_step, 2);
        tft180_show_string(76, 98, "/");
        tft180_show_int(82, 98, (int32)route_dbg_total, 2);
        tft180_show_string(52, 114, "PF:");
        tft180_show_int(70, 114, (int32)route_dbg_flag, 1);
        tft180_show_string(82, 114, "PA:");
        tft180_show_int(100, 114, (int32)route_dbg_action, 1);
        return;
    }

    tft180_show_string(0, 0, (char *)page->title);
    tft180_show_string(0, 8, adjust_mode ? "[SW:ADJUST]" : "[SW:SELECT]");

    uint8 start_y = 120u - page->item_count * 8u;

    for (uint8 i = 0u; i < page->item_count; i++)
    {
        MenuItem *item = &page->items[i];
        uint8 pos = 0u;

        buf[pos++] = (i == menu_cursor) ? '>' : ' ';
        pos = append_str(buf, pos, item->label);

        while (pos < 11u)
            buf[pos++] = ' ';

        pos = append_int(buf, pos, *item->value);
        buf[pos] = '\0';

        tft180_show_string(70, start_y + i * 8u, buf);
    }

    tft180_show_string(0, 120, "K1/K2:Page K3/K4:+-");
}

void menu_show(void)
{
#if RACE_MODE
    return;
#else
    MenuPageDef *page = &g_pages[now_page];
    static uint8 s_main_draw_cnt = 0u;
    static uint8 s_tft_run_off = 0u;

#if TFT_OFF_WHEN_RUNNING
    if (motor_enable != 0)
    {
        if (s_tft_run_off == 0u)
        {
            tft180_clear();
            s_main_draw_cnt = 0u;
            s_tft_run_off = 1u;
        }
        return;
    }

    if (s_tft_run_off != 0u)
    {
        tft180_clear();
        s_main_draw_cnt = 0u;
        s_tft_run_off = 0u;
    }
#endif

    if (now_page == PAGE_MAIN)
    {
        uint8 draw_div = (motor_enable != 0) ? MAIN_DRAW_DIV_RUN : MAIN_DRAW_DIV_STOP;

        if (s_main_draw_cnt == 0u)
            draw_line();

        s_main_draw_cnt++;
        if (s_main_draw_cnt >= draw_div)
            s_main_draw_cnt = 0u;
    }
    else
    {
        s_main_draw_cnt = 0u;
    }

    default_draw(page);
#endif
}

void turn_right_led_on(void)
{
    gpio_set_level(TURN_RIGHT_LED, GPIO_LOW);
}

void turn_right_led_off(void)
{
    gpio_set_level(TURN_RIGHT_LED, GPIO_HIGH);
}
