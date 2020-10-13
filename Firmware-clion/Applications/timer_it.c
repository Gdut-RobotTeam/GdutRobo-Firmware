//
// Created by luohx on 2020/9/29.
//

#include "timer_it.h"
#include "bsp_uart4.h"

extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart4;
uint32_t sys_time = 0;

void timer6_init() {
  HAL_TIM_Base_Start_IT(&htim6);
}

const uint32_t *get_systime_ptr() {
  return &sys_time;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static uint8_t times = 1;
  static uint8_t tracking_counter;

  if (htim == (&htim6)) {
    sys_time++;
    if (times == 20) {
      chassis_ctrl_loop();
      times = 0;
    }
    if (tracking_counter == 5) {
      tracking_bar_inquiry(&huart4, inquiry_id);
      tracking_counter = 0;
    }
    tracking_counter++;
    times++;
  }
}
