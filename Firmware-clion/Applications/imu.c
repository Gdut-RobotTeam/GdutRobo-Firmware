//
// Created by luohx on 2020/10/4.
//

#include "imu.h"
#include "timer_it.h"
#include "bsp_delay.h"

pid_type_def imu_pid;
static const float imu_PID[3] = {3.0f, 0, 0};

void imu_init() {
  float current_yaw = 0, last_yaw = 0;
  uint8_t init_times = 0;

  imu_receive_init();
  PID_init(&imu.pid_, PID_DELTA, imu_PID, 40, 0);
  while (init_times <= 100) {
    current_yaw = imu.yaw_;
    if (current_yaw == last_yaw) {
      delay_ms(10);
      init_times++;
    } else {
      init_times = 0;
    }
    last_yaw = current_yaw;
  }
  imu.init_ = current_yaw <= 180 ? -current_yaw : 360 - current_yaw;
  delay_ms(500);
}

float imu_correct_val(mpu6050_buffer *imu, float target_yaw) {
  static uint32_t last_time;
  const uint32_t *sys_time = get_systime_ptr();
  if (*sys_time - last_time >= 100) {
    imu->pid_.error_[1] = 0;
    imu->pid_.error_[2] = 0;
  }
  last_time = *sys_time;
  float current_yaw = imu->yaw_;
  if (current_yaw - target_yaw >= 180)
    current_yaw -= 360;
  if (imu->on_off_ == true) {
    return PID_cal(&imu->pid_, current_yaw, target_yaw);
  } else
    return 0;
}