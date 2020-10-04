//
// Created by luohx on 2020/9/27.
//

#ifndef FIRMWARE_CLION_COMPONENTS_DEVICES_MPU6050_H_
#define FIRMWARE_CLION_COMPONENTS_DEVICES_MPU6050_H_

#include <stdbool.h>
#include "main.h"
#include "pid.h"

typedef struct {
  float yaw_;
  float roll_;
  float pitch_;
  float ax_;
  float ay_;
  float az_;
  float bias_;
  bool on_off_;
  pid_type_def pid_;
} mpu6050_buffer;

extern void mpu6050_decode(const uint8_t *rx_buffer, mpu6050_buffer *solved_buffer);

#endif //FIRMWARE_CLION_COMPONENTS_DEVICES_MPU6050_H_
