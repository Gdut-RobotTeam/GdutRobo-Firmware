//
// Created by luohx on 2020/9/29.
//

#ifndef FIRMWARE_CLION_APPLICATIONS_CHASSIS_H_
#define FIRMWARE_CLION_APPLICATIONS_CHASSIS_H_

#include "tracker.h"
#include "bsp_motor.h"
#include "bsp_imu.h"
#include "pid.h"

#define CHASSIS_MOTOR_NUM 4
#define CHASSIS_RADIUS 1.80

typedef struct {
  float x_;
  float y_;
  float z_;
  bool on_off_;
} odom_t;

typedef struct {
  float vx_;
  float vy_;
  float vw_;
  float yaw_;
  odom_t odom_;
  motor_t motor_[CHASSIS_MOTOR_NUM];
  pid_type_def pid_[CHASSIS_MOTOR_NUM];
} chassis_move_t;

extern void chassis_init();
extern void chassis_ctrl_loop();
extern void chassis_odom_set(bool on_off);
extern const odom_t *chassis_odom_get();
extern void chassis_yaw_set(float yaw_angle);
extern void chassis_speed_set(float vx, float vy, float vw);

#endif //FIRMWARE_CLION_APPLICATIONS_CHASSIS_H_
