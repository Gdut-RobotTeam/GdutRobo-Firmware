//
// Created by luohx on 2020/9/30.
//

#include "chassis_behavior.h"
#include "timer_it.h"

void move_by_encoder(const float distance_x,
                     const float distance_y,
                     float vx_max,
                     float vy_max,
                     float acc,
                     float bias) {
  const uint32_t *sys_time = get_systime_ptr();
  uint32_t last_time = *sys_time;
  uint32_t last_time2 = 0;
  float vx = 0, vy = 0;
  float vx_set = 0, vy_set = 0;
  float vx_intercept, vy_intercept;
  const odom_t *odom_data = get_odom_ptr();
  chassis_odom_set(true);
  chassis_speed_set(0, 0, 0);
  forward_tracker.on_off_ = false;
  left_tracker.on_off_ = false;

  while (1) {
    if (*sys_time - last_time >= 40) {
      if (vy <= vy_max) {
        vy += acc;
        if (vy > vy_max) vy = vy_max;
      }
      if (vx <= vx_max) {
        vx += acc;
        if (vx > vx_max) vx = vx_max;
      }
      last_time = *sys_time;
    }

    vx_intercept = (distance_x - odom_data->x_) * 0.05;
    vy_intercept = (distance_y - odom_data->y_) * 0.05;
//    printf("%f,%f\r\n", odom_data->x_, odom_data->y_);

    if (vy_intercept >= vy) vy_set = vy;
    else if (vy_intercept <= -vy) vy_set = -vy;
    else vy_set = vy_intercept;

    if (vx_intercept >= vx)vx_set = vx;
    else if (vx_intercept <= -vx)vx_set = -vx;
    else vx_set = vx_intercept;
//    printf("%f,%f,%f,%f,%f,%f\r\n", vx, vy, vx_intercept, vy_intercept, vx_set, vy_set);
    if ((last_time2 == 0) & (fabsf(odom_data->y_ - distance_y) <= 500) & (fabsf(odom_data->x_ - distance_x) <= 500)) {
      last_time2 = *sys_time;
    }

    if (((fabsf(odom_data->y_ - distance_y) <= bias) & (fabsf(odom_data->x_ - distance_x) <= bias))
        | ((last_time2 != 0) & (*sys_time - last_time2 > 1500))) {
      chassis_odom_set(false);
      chassis_speed_set(0, 0, 0);
      break;
    }
    chassis_speed_set(vx_set, vy_set, 0);
  }
}

void move_by_forward_bar(uint8_t line_num, const int dir, const float vy_max, const float acc, const float theta_y) {
  const odom_t *odom_data = get_odom_ptr();
  const uint32_t *sys_time = get_systime_ptr();
  uint32_t last_time = *sys_time;
  float vy = 0, tmp_vy = 0;

  forward_tracker.on_off_ = true;
  left_tracker.on_off_ = false;
  chassis.odom_.on_off_ = true;

  forward_tracker.sign_ = 0;
  while (1) {
    if (*sys_time - last_time >= 40) {
      if (vy <= vy_max) {
        vy += acc;
        if (vy > vy_max)
          vy = vy_max;
      }
      last_time = *sys_time;
    }

    if (line_num == 0) {
      left_tracker.on_off_ = true;
      if (fabsf(theta_y - odom_data->y_) <= 10) {
        while (1) {
          chassis_speed_set(0, 0, 0);
          if (left_tracker.num_ == 0) {
            chassis_speed_set(0, 40, 0);
          } else if (fabsf(forward_tracker.value_) <= 1 && fabsf(left_tracker.value_) <= 1) {
            left_tracker.on_off_ = false;
            forward_tracker.on_off_ = false;
            break;
          }
        }
        chassis.odom_.on_off_ = false;
        break;
      }
    } else if (line_num == 1) {
      if (fabsf(odom_data->y_) >= 800) {
        if (forward_tracker.sign_ == 1) {
          line_num -= 1;
          chassis_odom_reset('y');
        }
      }
    } else {
      if (forward_tracker.sign_ == 1) {
        if (fabsf(odom_data->y_) >= 500) {
          line_num -= 1;
          forward_tracker.sign_ = 0;
          chassis_odom_reset('y');
        }
      }
    }
    if (line_num != 0)
      tmp_vy = (line_num * 3000 + theta_y) * 0.13;
    else if (line_num == 0)
      tmp_vy = (theta_y - odom_data->y_) * 0.13;
    if (tmp_vy >= vy);
    else if (tmp_vy <= -vy)
      vy = -vy;
    else
      vy = tmp_vy;
    chassis_speed_set(0, vy, 0);
  }
}

void move_by_left_bar(uint8_t line_num, const int dir, const float vx_max, const float acc, const float theta_x) {
  const odom_t *odom_data = get_odom_ptr();
  const uint32_t *sys_time = get_systime_ptr();
  uint32_t last_time = *sys_time;
  float vx = 0, tmp_vx = 0;

  left_tracker.on_off_ = true;
  forward_tracker.on_off_ = false;
  chassis.odom_.on_off_ = true;

  left_tracker.sign_ = 0;
  while (1) {
    if (*sys_time - last_time >= 40) {
      if (vx <= vx_max) {
        vx += acc;
        if (vx > vx_max)
          vx = vx_max;
      }
      last_time = *sys_time;
    }

    if (line_num == 0) {
      forward_tracker.on_off_ = true;
      if (fabsf(theta_x - fabsf(odom_data->x_)) <= 10) {
        while (1) {
          chassis_speed_set(0, 0, 0);
          if (forward_tracker.num_ == 0) {
            chassis_speed_set(-40, 0, 0);
          } else if (fabsf(forward_tracker.value_) <= 1 && fabsf(left_tracker.value_) <= 1) {
            left_tracker.on_off_ = false;
            forward_tracker.on_off_ = false;
            break;
          }
        }
        chassis.odom_.on_off_ = false;
        break;
      }
    } else {
      if (left_tracker.sign_ == 1) {
        if (fabsf(odom_data->x_) >= 800) {
          line_num -= 1;
          chassis_odom_reset('x');
        }
      }
    }
    if (line_num != 0)
      tmp_vx = (line_num * 3000 + theta_x) * 0.13;
    else if (line_num == 0)
      tmp_vx = (theta_x - odom_data->x_) * 0.13;
    if (tmp_vx >= vx);
    else if (tmp_vx <= -vx)
      vx = -vx;
    else
      vx = tmp_vx;
    chassis_speed_set(-vx, 0, 0);
  }
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
    float bias;
    bias = imu.yaw_ - target_angle;
    if (bias > 180) bias -= 360;
    else if (bias < -180) bias += 360;
    if (fabsf(bias) <= 0.5)break;
  }
  delay_ms(200);
}

