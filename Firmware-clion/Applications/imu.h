//
// Created by luohx on 2020/10/4.
//

#ifndef FIRMWARE_CLION_APPLICATIONS_IMU_H_
#define FIRMWARE_CLION_APPLICATIONS_IMU_H_

#include "pid.h"
#include "bsp_imu.h"
extern float imu_correct_val(mpu6050_buffer *imu, float target_angle);

#endif //FIRMWARE_CLION_APPLICATIONS_IMU_H_