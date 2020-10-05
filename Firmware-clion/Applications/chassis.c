//
// Created by luohx on 2020/9/29.
//

#include "chassis.h"
#include "imu.h"

chassis_move_t chassis;
static const float chassis_PID[CHASSIS_MOTOR_NUM][3] = {
    {60, 15, 0},
    {60, 15, 0},
    {60, 15, 0},
    {60, 15, 0}
};

void chassis_init() {
  for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
    chassis.motor_[i].speed_ = 0;
    chassis.motor_[i].encoder_ = 0;
    chassis.vx_ = 0;
    chassis.vy_ = 0;
    chassis.vw_ = 0;
    chassis.yaw_ = 0;
    chassis.odom_.x_ = 0;
    chassis.odom_.y_ = 0;
    chassis.odom_.z_ = 0;
  }
  for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
    PID_init(&chassis.pid_[i], PID_DELTA, chassis_PID[i], 7000, 0);
  }
}

void chassis_speed_set(float vx, float vy, float vw) {
  chassis.vx_ = vx;
  chassis.vy_ = vy;
  chassis.vw_ = vw;
}

void chassis_yaw_set(float yaw_angle) {
  chassis.yaw_ = yaw_angle;
}

void chassis_odom_set(bool on_off) {
  chassis.odom_.on_off_ = on_off;
}

const odom_t *chassis_odom_get() {
  return &chassis.odom_;
}

void chassis_ctrl_loop() {
  chassis.vx_ += tracker_correct_val(&forward_tracker, 0);
  chassis.vy_ += tracker_correct_val(&left_tracker, 0);
  chassis.vy_ += tracker_correct_val(&right_tracker, 0);

  chassis.vw_ += imu_correct_val(&imu, chassis.yaw_);

  chassis.motor_[0].speed_ =
      -0.7071 * chassis.vx_ - 0.7071 * chassis.vy_ + CHASSIS_RADIUS * chassis.vw_;        //top left
  chassis.motor_[1].speed_ =
      0.7071 * chassis.vx_ - 0.7071 * chassis.vy_ + CHASSIS_RADIUS * chassis.vw_;        //bottom left
  chassis.motor_[2].speed_ =
      0.7071 * chassis.vx_ + 0.7071 * chassis.vy_ + CHASSIS_RADIUS * chassis.vw_;        //bottom right
  chassis.motor_[3].speed_ =
      -0.7071 * chassis.vx_ + 0.7071 * chassis.vy_ + CHASSIS_RADIUS * chassis.vw_;        //top right

  chassis.motor_[0].encoder_ = read_encoder(2);
  chassis.motor_[1].encoder_ = read_encoder(3);
  chassis.motor_[2].encoder_ = read_encoder(4);
  chassis.motor_[3].encoder_ = read_encoder(5);
  printf("%d,%d,%d,%d\r\n",
         chassis.motor_[0].encoder_,
         chassis.motor_[1].encoder_,
         chassis.motor_[2].encoder_,
         chassis.motor_[3].encoder_);

  float current_x_speed = (chassis.motor_[1].encoder_ - chassis.motor_[0].encoder_) * 1.4142;
  float current_y_speed = (chassis.motor_[3].encoder_ - chassis.motor_[2].encoder_) * 1.4142;

  if (chassis.odom_.on_off_ == true) {
    chassis.odom_.x_ += current_x_speed;
    chassis.odom_.y_ += current_y_speed;
  } else {
    chassis.odom_.x_ = 0;
    chassis.odom_.y_ = 0;
  }

  for (int i = 0; i < 4; i++) {
//    int output = (int) PID_cal(&chassis.pid_[i], (float) chassis.motor_[i].encoder_, chassis.motor_[i].speed_);
    int output = (int) PID_cal(&chassis.pid_[i], chassis.motor_[i].encoder_, 100);
    motor_set_pwm(i + 1, output);
  }
}