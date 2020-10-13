//
// Created by luohx on 2020/9/28.
//

#include "bsp_uart4.h"
#include "bsp_led.h"

#define BUFFER_SIZE  10
extern UART_HandleTypeDef huart4;

line_grays forward_tracker;
line_grays left_tracker;
line_grays right_tracker;
const float forward_tracker_weight[10] = {-5, -4, -3, -2, -0.5, 0.5, 2, 3, 4, 5};
const float left_tracker_weight[10] = {-5, -4, -3, -2, -0.5, 0.5, 2, 3, 4, 5};
const float right_tracker_weight[10] = {-5, -4, -3, -2, -0.5, 0.5, 2, 3, 4, 5};

uint8_t uart4_rx_buf[BUFFER_SIZE] = {0};
uint8_t uart4_rx_len = 0;

uint8_t inquiry_id;

void uart4_receive_init() {
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
  inquiry_id = 0x01;
}

void UART4_IRQHandler() {
  uint8_t res;
  uint8_t id = 0x01;
  static uint8_t state;
  if (UART4->SR & (1 << 5)) {
    res = UART4->DR;
    if (state == 2)  //head sign has been received
    {
      uart4_rx_buf[uart4_rx_len] = res;
      if (uart4_rx_len >= 6) {
        if (tracking_bar_verify(uart4_rx_buf) == uart4_rx_buf[6]) {
          if (uart4_rx_buf[0] == 0x01) {
            tracking_bar_decode(uart4_rx_buf, &forward_tracker, forward_tracker_weight);
            inquiry_id = 0x02;
//            printf("\r\nforward bar received!\r\n");
          } else if (uart4_rx_buf[0] == 0x02) {
            tracking_bar_decode(uart4_rx_buf, &left_tracker, left_tracker_weight);
            inquiry_id = 0x01;
//            printf("\r\nleft bar received!\r\n");
          }
//          if (uart4_rx_buf[0] == 0x03) {
//            tracking_bar_decode(uart4_rx_buf, &right_tracker, right_tracker_weight);
//            buzzer_off();
//          }
        }
        state = 0;
      }
      ++uart4_rx_len;
    } else if (res == 0xff && state == 0)
      state = 1;
    else if (state == 1 && res == 0xff) {
      state = 2;
      uart4_rx_len = 0;
    } else
      state = 0;
  }
}
