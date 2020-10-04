//
// Created by luohx on 2020/10/4.
//

#ifndef FIRMWARE_CLION_APPLICATIONS_TRACKER_H_
#define FIRMWARE_CLION_APPLICATIONS_TRACKER_H_

#include "main.h"
#include "bsp_uart4.h"
#include "stdbool.h"

void tracker_init();
extern bool get_tracker_switch_state(const line_grays *tracker);
extern float tracker_correct_val(line_grays *tracker, float target_pos_val);

#endif //FIRMWARE_CLION_APPLICATIONS_TRACKER_H_
