//
// Created by luohx on 2020/9/25.
//
#include "bsp_buzzer.h"

void buzzer_on() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

void buzzer_off() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

void buzzer_toggle() {
  static uint8_t flag = 0;
  if (flag == 0) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    flag = 1;
  }
  if (flag == 1) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    flag = 0;
  }
}