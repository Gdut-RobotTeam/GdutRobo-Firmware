//
// Created by luohx on 2020/10/4.
//

#include "tracker.h"
#include "bsp_delay.h"
#include "pid.h"

extern UART_HandleTypeDef huart4;
static const float tracker_PID[3] = {8.0f, 0, 0};

void tracker_init() {
  PID_init(&forward_tracker.pid_, PID_DELTA, tracker_PID, 40, 100);
  PID_init(&left_tracker.pid_, PID_DELTA, tracker_PID, 40, 100);
  PID_init(&right_tracker.pid_, PID_DELTA, tracker_PID, 40, 100);
}

static bool get_tracker_switch_state(const line_grays *tracker) {
  return tracker->on_off_;
}

float tracker_correct_val(line_grays *tracker, float target_pos_val) {
  if (get_tracker_switch_state(tracker)) {
    if (tracker->num_ == 0u | tracker->num_ >= 7u)
      return 0;
    else
      return PID_cal(&tracker->pid_, tracker->value_, target_pos_val);
  } else
    return 0;
}