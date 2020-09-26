//
// Created by luohx on 2020/9/24.
//

#ifndef FIRMWARE_CLION_BSP_BSP_IMU_H_
#define FIRMWARE_CLION_BSP_BSP_IMU_H_
#include "main.h"
#include "string.h"

struct Imu {
  float yaw;
  float roll;
  float pitch;
  float ax;
  float ay;
  float az;
  float bias;
};

extern void imu_receive_init();
extern struct Imu imu;

#endif //FIRMWARE_CLION_BSP_BSP_IMU_H_
