//
// Created by luohx on 2020/9/27.
//

#include "mpu6050.h"
#include "imu.h"

void mpu6050_decode(const uint8_t *rx_buffer, mpu6050_buffer *solved_buffer) {
  if (rx_buffer[0] == 0x55) {
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++)
      sum += rx_buffer[i];
    if (sum == rx_buffer[10]) {
      if (rx_buffer[1] == 0x51) {
        solved_buffer->ax_ = 16 * (short) (rx_buffer[3] << 8 | rx_buffer[2]) / 32768.0;
        solved_buffer->ay_ = 16 * (short) (rx_buffer[5] << 8 | rx_buffer[4]) / 32768.0;
        solved_buffer->az_ = 16 * (short) (rx_buffer[7] << 8 | rx_buffer[6]) / 32768.0;
      }

    }
    sum = 0;
    for (int i = 22; i < 32; i++)
      sum += rx_buffer[i];
    if (sum == rx_buffer[32]) {
      if (rx_buffer[23] == 0x53) {
        solved_buffer->roll_ = 180.0 * (short) (rx_buffer[25] << 8 | rx_buffer[24]) / 32768.0;
        solved_buffer->pitch_ = 180.0 * (short) (rx_buffer[27] << 8 | rx_buffer[26]) / 32768.0;
        solved_buffer->yaw_ = 180.0 * (short) (rx_buffer[29] << 8 | rx_buffer[28]) / 32768.0 + solved_buffer->bias_
            + solved_buffer->init_;
        if (solved_buffer->yaw_ > 360) solved_buffer->yaw_ -= 360;
        else if (solved_buffer->yaw_ < 0) solved_buffer->yaw_ += 360;
//        printf("%f\r\n", solved_buffer->yaw_);
//        printf("solved_buffer :%f  %f  %f\r\n", solved_buffer->roll_, solved_buffer->pitch_, solved_buffer->yaw_);
      }
    }
  }
}