#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

/*
 * Yaw-only IMU interface.
 *
 * yaw_angle: degrees, normalized to [-180, 180]
 * yaw_rate : degrees per second
 * imu_ready: 1 after IMU660RC init succeeds and the 5ms PIT is started
 * imu_error: 1 when init failed or zero calibration variance was high
 */

extern volatile float yaw_angle;
extern volatile float yaw_rate;
extern volatile uint8 imu_ready;
extern volatile uint8 imu_error;
extern volatile int16 imu_offset_dps10;

void imu_init(void);
void imu_update(void);
void imu_reset_yaw(void);

#endif /* CODE_IMU_H_ */
