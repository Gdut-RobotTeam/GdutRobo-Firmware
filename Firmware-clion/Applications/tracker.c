//
// Created by luohx on 2020/10/4.
//

#include "tracker.h"
#include "pid.h"

extern UART_HandleTypeDef huart4;
pid_type_def tracker_pid;
static const float tracker_PID[3] = {7.0f, 0, 0};

void tracker_init() {
  PID_init(&forward_tracker.pid_, PID_DELTA, tracker_PID, 40, 100);
  PID_init(&left_tracker.pid_, PID_DELTA, tracker_PID, 40, 100);
  PID_init(&right_tracker.pid_, PID_DELTA, tracker_PID, 40, 100);
}

bool get_tracker_switch_state(const line_grays *tracker) {
  return tracker->on_off_;
}

void tracker_inquiry() {
  static uint8_t tracker_id = 0x01;
  tracking_bar_inquiry(&huart4, tracker_id);
  tracker_id++;
  if (tracker_id > 0x03)
    tracker_id = 0x01;
}

float tracker_correct_val(line_grays *tracker, float target_pos_val) {
  if (get_tracker_switch_state(tracker)) {
    if (tracker->num_ == 0 | tracker->num_ >= 7)
      return 0;
    else
      return PID_cal(&tracker->pid_, tracker->value_, target_pos_val);
  } else
    return 0;
}