//
// Created by luohx on 2020/9/24.
//

#include "bsp_imu.h"

#define BUFFER_SIZE 200

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

uint8_t imu_rx_buf[BUFFER_SIZE] = {0};
uint8_t imu_rx_len = 0;

mpu6050_buffer imu;

void imu_receive_init() {
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart3, imu_rx_buf, BUFFER_SIZE);
}

void USART3_IRQHandler() {
  uint32_t flag_idle = 0;

  flag_idle = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE);
  if ((flag_idle != RESET)) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);

    HAL_UART_DMAStop(&huart3);
    uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
    imu_rx_len = BUFFER_SIZE
    -temp;

    mpu6050_decode(imu_rx_buf, &imu);

    imu_rx_len = 0;
    memset(imu_rx_buf, 0, imu_rx_len);
  }
  HAL_UART_Receive_DMA(&huart3, imu_rx_buf, BUFFER_SIZE);
  HAL_UART_IRQHandler(&huart3);
}
