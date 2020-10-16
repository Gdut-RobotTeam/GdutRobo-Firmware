//
// Created by luohx on 2020/9/28.
//

#include "tracking_bar.h"
#include "bsp_delay.h"

uint8_t tracking_bar_verify(const uint8_t *data) {
  uint8_t i = 0;
  register uint16_t verify_value = 0;

  for (i = 0; i < 6; i++)
    verify_value += data[i];

  return (uint8_t) (verify_value & 0XFF);
}

void tracking_bar_decode(const uint8_t *rx_buffer, line_grays *solved_buffer, const float *weight) {
  uint8_t i, a;
  float tmp = 0;

  solved_buffer->gray_ = (rx_buffer[5]) | (rx_buffer[4] << 4) | (rx_buffer[3] << 8);
  solved_buffer->num_ = 0;

  for (i = 0; i < 10; i++) {
    a = (solved_buffer->gray_ >> i) & 0X01;
    solved_buffer->num_ += a;
    tmp += a * weight[i];
  }

  if (solved_buffer->num_ != 0)
    solved_buffer->value_ = tmp / solved_buffer->num_;
  else
    solved_buffer->value_ = 0;
  if (solved_buffer->num_ >= 5)
    solved_buffer->sign_ = 1;
  else
    solved_buffer->sign_ = 0;
}

void tracking_bar_inquiry(UART_HandleTypeDef *huart, uint8_t id) {
  uint8_t get_line_cmd[] = {0xff, 0xff, id, 0x02, 0x08, (0x0a + id)};
  HAL_UART_Transmit(huart, get_line_cmd, 6, 0XFF);
}
