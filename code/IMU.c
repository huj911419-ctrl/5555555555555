#include "IMU.h"

/*
 * IMU yaw integration layer.
 *
 * Sensor: IMU660RC over SPI0, using raw gyro data.
 * Output convention:
 *   yaw_angle > 0: right turn
 *   yaw_angle < 0: left turn
 *
 * This module only measures yaw. It does not drive the motors directly.
 */

volatile float yaw_angle = 0.0f;
volatile float yaw_rate  = 0.0f;
volatile uint8 imu_ready = 0u;
volatile uint8 imu_error = 1u;
volatile int16 imu_offset_dps10 = 0;

#define IMU_UPDATE_PERIOD_US     5000u
#define IMU_UPDATE_DT_SEC        0.005f

#define IMU_INIT_RETRY           3u
#define IMU_CALIB_RETRY          3u
#define IMU_CALIB_DISCARD        20u
#define IMU_CALIB_SAMPLES        180u
#define IMU_CALIB_DELAY_MS       5u

#define IMU_Z_SIGN               (-1.0f)
#define IMU_RATE_DEADBAND_DPS    0.35f
#define IMU_RATE_FILTER_ALPHA    0.65f
#define IMU_RATE_LIMIT_DPS       1200.0f
#define IMU_CALIB_VAR_MAX        6.0f
#define IMU_RESET_SKIP_SAMPLES   2u

static float s_gyro_z_offset = 0.0f;
static float s_yaw_rate_filtered = 0.0f;
static uint8 s_reset_skip = 0u;

static float imu_absf(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static float imu_clampf(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static float imu_read_z_rate_dps(void)
{
    imu660rc_get_gyro();
    return IMU_Z_SIGN * imu660rc_gyro_transition(imu660rc_gyro_z);
}

static void imu_clear_state(void)
{
    yaw_angle = 0.0f;
    yaw_rate = 0.0f;
    s_yaw_rate_filtered = 0.0f;
    s_reset_skip = IMU_RESET_SKIP_SAMPLES;
}

static void imu_normalize_yaw(void)
{
    if (yaw_angle > 180.0f)
        yaw_angle -= 360.0f;
    else if (yaw_angle < -180.0f)
        yaw_angle += 360.0f;
}

static uint8 imu_calibrate_zero(void)
{
    float best_offset = 0.0f;
    float best_var = 1000000.0f;
    uint8 best_valid = 0u;

    for (uint8 retry = 0u; retry < IMU_CALIB_RETRY; retry++)
    {
        float sum = 0.0f;
        float sum_sq = 0.0f;

        for (uint16 i = 0u; i < IMU_CALIB_DISCARD; i++)
        {
            (void)imu_read_z_rate_dps();
            system_delay_ms(IMU_CALIB_DELAY_MS);
        }

        for (uint16 i = 0u; i < IMU_CALIB_SAMPLES; i++)
        {
            float v = imu_read_z_rate_dps();
            sum += v;
            sum_sq += v * v;
            system_delay_ms(IMU_CALIB_DELAY_MS);
        }

        {
            float sample_count = (float)IMU_CALIB_SAMPLES;
            float mean = sum / sample_count;
            float variance = sum_sq / sample_count - mean * mean;
            if (variance < 0.0f) variance = 0.0f;

            if (variance < best_var)
            {
                best_var = variance;
                best_offset = mean;
                best_valid = 1u;
            }

            if (variance <= IMU_CALIB_VAR_MAX)
                break;
        }
    }

    if (!best_valid)
        return 1u;

    s_gyro_z_offset = best_offset;
    imu_offset_dps10 = (int16)(best_offset * 10.0f);

    /* Use the best offset even if the car moved during calibration. */
    return (best_var <= IMU_CALIB_VAR_MAX) ? 0u : 1u;
}

void imu_init(void)
{
    imu_ready = 0u;
    imu_error = 1u;
    s_gyro_z_offset = 0.0f;
    imu_offset_dps10 = 0;
    imu_clear_state();

    for (uint8 retry = 0u; retry < IMU_INIT_RETRY; retry++)
    {
        if (imu660rc_init(IMU660RC_QUARTERNION_DISABLE) == 0u)
        {
            system_delay_ms(50);

            /*
             * Calibrate while the car is still. If the variance is too high,
             * keep the best offset and still enable yaw so the TFT can expose
             * the problem immediately.
             */
            imu_error = imu_calibrate_zero();
            imu_clear_state();
            imu_ready = 1u;
            pit_init(CCU60_CH1, IMU_UPDATE_PERIOD_US);
            return;
        }

        system_delay_ms(100);
    }

    imu_ready = 0u;
    imu_error = 1u;
}

void imu_update(void)
{
    float rate;

    if (!imu_ready)
        return;

    rate = imu_read_z_rate_dps() - s_gyro_z_offset;
    rate = imu_clampf(rate, -IMU_RATE_LIMIT_DPS, IMU_RATE_LIMIT_DPS);

    if (imu_absf(rate) < IMU_RATE_DEADBAND_DPS)
        rate = 0.0f;

    if (s_reset_skip > 0u)
    {
        s_reset_skip--;
        s_yaw_rate_filtered = 0.0f;
        yaw_rate = 0.0f;
        return;
    }

    s_yaw_rate_filtered =
        s_yaw_rate_filtered * IMU_RATE_FILTER_ALPHA +
        rate * (1.0f - IMU_RATE_FILTER_ALPHA);

    yaw_rate = s_yaw_rate_filtered;
    yaw_angle += yaw_rate * IMU_UPDATE_DT_SEC;
    imu_normalize_yaw();
}

void imu_reset_yaw(void)
{
    imu_clear_state();
}
