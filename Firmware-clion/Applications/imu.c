//
// Created by luohx on 2020/10/4.
//

#include "imu.h"

pid_type_def imu_pid;
static const float imu_PID[3] = {5.0f, 0, 0};

void imu_pid_init() {
  PID_init(&imu.pid_, PID_DELTA, imu_PID, 40, 0);
}

float imu_correct_val(mpu6050_buffer *imu, float target_angle) {
  if (imu->on_off_ == true) {
    return PID_cal(&imu_pid, imu->yaw_, target_angle);
  } else
    return 0;
}