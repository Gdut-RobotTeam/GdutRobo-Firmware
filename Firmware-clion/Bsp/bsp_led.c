//
// Created by luohx on 2020/9/25.
//

#include "bsp_led.h"

void led_on(void) {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void led_off(void) {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void led_twinkle(void) {
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
