//
// Created by luohx on 2020/9/30.
//

#ifndef FIRMWARE_CLION_APPLICATIONS_CHASSIS_BEHAVIOR_H_
#define FIRMWARE_CLION_APPLICATIONS_CHASSIS_BEHAVIOR_H_

#include <math.h>
#include <stdlib.h>
#include "chassis.h"
#include "bsp_delay.h"

extern void move_by_forward_bar(uint8_t line_num, int dir, float vy_max, float acc, float theta_y);
extern void move_by_encoder(float distance_x, float distance_y, float vx_max, float vy_max, float acc, float bias);
extern void move_by_left_bar(uint8_t line_num, const int dir, const float vx_max, const float acc, const float theta_x);
extern void move_from_point_to_point(int x, int y, int v, int acc, int if_x_is_0, int if_y_is_0);
extern void turn_direction(int degree);

#endif //FIRMWARE_CLION_APPLICATIONS_CHASSIS_BEHAVIOR_H_
