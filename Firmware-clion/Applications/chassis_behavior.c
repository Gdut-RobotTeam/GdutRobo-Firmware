//
// Created by luohx on 2020/9/30.
//

#include "chassis_behavior.h"
#include "timer_it.h"
#include "bsp_buzzer.h"

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
  float vx_intercept = 0, vy_intercept = 0;
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
  float vx = 0, tmp_vx = 0, vx_set = 0;

  left_tracker.on_off_ = true;
  forward_tracker.on_off_ = false;
  chassis.odom_.on_off_ = true;

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
    } else if (line_num == 1) {
      if (dir == -1) {
        if (left_tracker.sign_ == 1) {
          if (fabsf(odom_data->x_) >= 800) {
            line_num -= 1;
            chassis_odom_reset('x');
          }
        }
      } else {
        if ((forward_tracker.num_ >= 2) & (forward_tracker.gray_ && 0x03 != 0)) {
          if (fabsf(odom_data->x_) >= 800) {
            line_num -= 1;
            chassis_odom_reset('x');
          }
        }
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
      tmp_vx = dir * (line_num * 3000 + theta_x) * 0.13;
    else if (line_num == 0)
      tmp_vx = (dir * theta_x - odom_data->x_) * 0.13;
    if (tmp_vx >= vx)
      vx_set = vx;
    else if (tmp_vx <= -vx)
      vx_set = -vx;
    else
      vx_set = tmp_vx;
    chassis_speed_set(vx_set, 0, 0);
  }
}

void move_by_relative(int x, int y, int vx_max, int vy_max, int acc, int if_x_is_0, int if_y_is_0) {
  const odom_t *odom_data = get_odom_ptr();
  const uint32_t *sys_time = get_systime_ptr();
  uint32_t last_time = *sys_time;
  int dir_x, dir_y;
  int vx, vy;
  int tmp_vx, tmp_vy;
  int vx_set, vy_set;
  forward_tracker.on_off_ = false;
  left_tracker.on_off_ = false;
  chassis.odom_.on_off_ = true;

  vx = vy = 0;    //得到速度
  vx_set = vy_set = 0;
//
  if (x > 0) dir_x = 1; else if (x < 0) dir_x = -1; else dir_x = if_x_is_0;//方向
  if (y > 0) dir_y = 1; else if (y < 0) dir_y = -1; else dir_y = if_y_is_0;
  while (1) {
    if (*sys_time - last_time >= 40) {
      if (abs(x) > 1) {
        if (vx <= vx_max) {
          vx += acc;
          if (vx > vx_max) vx = vx_max;
        }
      } else {
        if (vx - 65 > acc) vx -= acc;
        else if (65 - vx > acc) vx += acc;
        else vx = 65;
      }
      if (abs(y) > 1) {
        if (vy <= vy_max) {
          vy += acc;
          if (vy > vy_max) vy = vy_max;
        }
      } else {
        if (vy - 65 > acc) vy -= acc;
        else if (65 - vy > acc) vy += acc;
        else vy = 65;
      }
      last_time = *sys_time;
    }

    if (y == 0) {
      if ((left_tracker.num_ != 0) & (left_tracker.num_ < 7)) {
        vy_set = 0;
        left_tracker.on_off_ = true;
        tmp_vy = -left_tracker.value_ * left_tracker.pid_.kp_;
      } else if (left_tracker.num_ >= 7) {
        vy_set = tmp_vy;
      } else {
        tmp_vy = vy * dir_y;
        vy_set = vy * dir_y;
      }
    } else if (abs(y) == 1) {
      if (forward_tracker.sign_ == 1) {
        y -= dir_y;
      }
      vy_set = vy * dir_y;
    } else {
      if (forward_tracker.sign_ == 1) {
        if (fabsf(odom_data->y_) >= 800) {
          y -= dir_y;
          chassis_odom_reset('y');
        }
      }
      vy_set = vy * dir_y;
    }
    if (x == 0) {
      if ((forward_tracker.num_ != 0) & (forward_tracker.num_ < 7)) {
        vx_set = 0;
        forward_tracker.on_off_ = true;
        tmp_vx = forward_tracker.value_ * forward_tracker.pid_.kp_;
      } else if (forward_tracker.num_ >= 7) {
        vx_set = tmp_vx;
      } else {
        tmp_vx = vx * dir_x;
        vx_set = vx * dir_x;
      }
    } else if (x == 1) {
      if (left_tracker.sign_ == 1) {
        x -= dir_x;
      }
      vx_set = vx * dir_x;
    } else {
      if (left_tracker.sign_ == 1) {
        if (fabsf(odom_data->x_) >= 1200) {
          x -= dir_x;
          chassis_odom_reset('x');
        }
      }
      vx_set = vx * dir_x;
    }
    chassis_speed_set(vx_set, vy_set, 0);
    if ((forward_tracker.on_off_ == true) & (left_tracker.on_off_ == true)) {
      if (((forward_tracker.gray_ & 0x30) != 0) & ((forward_tracker.gray_ & 1) == 0)
          & ((forward_tracker.gray_ & 0x200) == 0)
          & ((left_tracker.gray_ & 0x30) != 0) & ((left_tracker.gray_ & 1) == 0) & ((left_tracker.gray_ & 0x200) == 0)
          ) {
        forward_tracker.on_off_ = false;
        left_tracker.on_off_ = false;
        chassis_odom_set(false);
        vx_set = 0;
        vy_set = 0;
        chassis_speed_set(vx_set, vy_set, 0);
        break;
      }
    }
  }
}

void move_from_point_to_point(int x, int y, int v, int acc, int if_x_is_0, int if_y_is_0) {
  int vx, vy;
  if (abs(x) > abs(y)) {
    vx = v;
    vy = v * fabsf((float) y / (float) x);
  } else if (abs(x) < abs(y)) {
    vy = v;
    vx = v * fabsf((float) x / (float) y);
  } else {
    vx = vy = v;
  }
  move_by_relative(x, y, vx, vy, acc, if_x_is_0, if_y_is_0);
}

void turn_direction(int degree) {
  const odom_t *odom_data = get_odom_ptr();
  const uint32_t *sys_time = get_systime_ptr();
  uint32_t last_time = *sys_time;
  uint32_t last_time2 = 0;

  imu.on_off_ = false;
  forward_tracker.on_off_ = false;
  left_tracker.on_off_ = false;
  chassis.odom_.on_off_ = true;

  if (degree > 180)
    degree -= 360;
  if (degree < -180)
    degree += 360;

  int k = fabsf(degree) <= 90 ? 243 : 274;

  float distance_z = degree * CHASSIS_RADIUS * k / 6.665816 / 3.141592;

  float vw = 0, vw_set = 0, vw_intercept = 0;

  while (1) {
    if (*sys_time - last_time >= 40) {
      if (vw <= 50) {
        vw += 5;
        if (vw > 50) vw = 50;
      }
      last_time = *sys_time;
    }

    vw_intercept = (distance_z - odom_data->z_) * 0.5;
    if (vw_intercept >= vw) vw_set = vw;
    else if (vw_intercept <= -vw) vw_set = -vw;
    else vw = vw_intercept;

    if ((last_time2 == 0) & (fabsf(odom_data->z_ - distance_z) <= 500)) {
      last_time2 = *sys_time;
    }

    if (fabsf(odom_data->z_ - distance_z) <= 100 | (last_time2 != 0) & (*sys_time - last_time2 > 2000)) {
      chassis_odom_set(false);
      chassis_speed_set(0, 0, 0);
      break;
    }
    chassis_speed_set(0, 0, vw_set);
  }
}

