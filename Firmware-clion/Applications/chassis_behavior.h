//
// Created by luohx on 2020/9/30.
//

#ifndef FIRMWARE_CLION_APPLICATIONS_CHASSIS_BEHAVIOR_H_
#define FIRMWARE_CLION_APPLICATIONS_CHASSIS_BEHAVIOR_H_

#include <math.h>
#include <stdlib.h>
#include "chassis.h"
#include "bsp_delay.h"

extern void move_by_encoder(float distance_x, float distance_y, float vx_max, float vy_max, float acc, float bias);

#endif //FIRMWARE_CLION_APPLICATIONS_CHASSIS_BEHAVIOR_H_
