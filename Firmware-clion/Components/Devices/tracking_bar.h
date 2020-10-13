//
// Created by luohx on 2020/9/28.
//

#ifndef FIRMWARE_CLION_COMPONENTS_DEVICES_TRACKING_BAR_H_
#define FIRMWARE_CLION_COMPONENTS_DEVICES_TRACKING_BAR_H_

#include <stdbool.h>
#include "main.h"
#include "pid.h"

typedef struct {
  uint16_t gray_;
  float value_;
  uint8_t num_;
  uint8_t sign_;
  uint8_t rec_sign_;
  bool on_off_;
  pid_type_def pid_;
} line_grays;

extern uint8_t tracking_bar_verify(const uint8_t *data);
extern void tracking_bar_decode(const uint8_t *rx_buffer, line_grays *solved_buffer, const float *weight);
extern void tracking_bar_inquiry(UART_HandleTypeDef *huart, uint8_t id);

#endif //FIRMWARE_CLION_COMPONENTS_DEVICES_TRACKING_BAR_H_
