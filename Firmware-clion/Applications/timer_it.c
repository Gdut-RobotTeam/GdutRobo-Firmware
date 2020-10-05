//
// Created by luohx on 2020/9/29.
//

#include "timer_it.h"

extern TIM_HandleTypeDef htim6;

void timer6_init() {
  HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == (&htim6)) {
    tracker_inquiry();
    chassis_ctrl_loop();
  }
}
