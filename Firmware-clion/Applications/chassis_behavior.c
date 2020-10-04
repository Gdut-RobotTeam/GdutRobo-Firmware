//
// Created by luohx on 2020/9/30.
//

#include "chassis_behavior.h"

void move_by_encoder(float distance_x, float distance_y, float vx_max, float vy_max, float acc, float bias) {
  float vx = 0, vy = 0;
  float vx_intercept, vy_intercept;
  const odom_t *odom_data = chassis_odom_get();

  chassis_odom_set(true);

  while (1) {
    if (vy < vy_max) {
      vy += acc;
      if (vy > vy_max) vy = vy_max;
    }
    if (vx < vx_max) {
      vx += acc;
      if (vx > vx_max) vx = vx_max;
    }

    vx_intercept = (distance_x - odom_data->x_) * 0.02;
    vy_intercept = (distance_y - odom_data->y_) * 0.02;

    if (vx_intercept >= vx);
    else if (vx_intercept >= vx) vx = -vx;
    else vx = vx_intercept;

    if (vy_intercept >= vy);
    else if (vy_intercept >= vy) vy = -vy;
    else vy = vy_intercept;

    if (abs(odom_data->x_ - distance_x <= bias) && abs(odom_data->y_ - distance_y <= bias)) {
      chassis_odom_set(false);
      break;
    }
    chassis_speed_set(vx, vy, 0);
  }
}

void move_by_forward_bar(int line_num) {
  forward_tracker.on_off_ = true;
}

void move_by_left_bar() {
  left_tracker.on_off_ = true;
}

void turn_direction(uint8_t dir) {
  float target_angle = 0;
  imu.on_off_ = true;
  chassis_odom_set(false);

  if (dir == 'n') target_angle = 90;
  if (dir == 's') target_angle = 270;
  if (dir == 'e') target_angle = 0;
  if (dir == 'w') target_angle = 180;

  chassis_yaw_set(target_angle);

  while (1) {
    int bias;
    bias = (int) (imu.yaw_ - target_angle);
    if (bias > 180) bias -= 360;
    else if (bias < -180) bias += 360;
    if (abs(bias) <= 0.5)break;
  }
  delay_ms(200);
}

