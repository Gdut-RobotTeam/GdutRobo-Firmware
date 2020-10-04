//
// Created by luohx on 2020/9/24.
//

#ifndef FIRMWARE_CLION_SUPPORT_RETARGET_H_
#define FIRMWARE_CLION_SUPPORT_RETARGET_H_

#include "stm32f1xx_hal.h"
#include <sys/stat.h>
#include <stdio.h>

void RetargetInit(UART_HandleTypeDef *huart);

int _isatty(int fd);

int _write(int fd, char *ptr, int len);

int _close(int fd);

int _lseek(int fd, int ptr, int dir);

int _read(int fd, char *ptr, int len);

int _fstat(int fd, struct stat *st);

#endif //FIRMWARE_CLION_SUPPORT_RETARGET_H_
