//
// Created by luohx on 2020/9/29.
//

#include "timer_it.h"

extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart4;

void timer6_init() {
  HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static uint8_t ticks;
  static uint8_t tracking_bar_id = 0x01;

  if (htim == (&htim6)) {
    if (ticks >= 20) {
      ticks = 0;
      tracking_bar_inquiry(&huart4, tracking_bar_id);
      tracking_bar_id++;
      if (tracking_bar_id > 0x03)
        tracking_bar_id = 0x01;
      chassis_ctrl_loop();
    }
    ticks++;
  }
}