//
// Created by luohx on 2020/9/25.
//
#include "bsp_buzzer.h"

void buzzer_on(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

void buzzer_off(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}