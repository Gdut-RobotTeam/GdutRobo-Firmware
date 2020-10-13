//
// Created by luohx on 2020/9/28.
//

#ifndef FIRMWARE_CLION_BSP_BSP_UART4_H_
#define FIRMWARE_CLION_BSP_BSP_UART4_H_

#include "main.h"
#include "tracking_bar.h"

extern line_grays forward_tracker;
extern line_grays left_tracker;
extern line_grays right_tracker;
extern uint8_t inquiry_id;

extern void uart4_receive_init();

#endif //FIRMWARE_CLION_BSP_BSP_UART4_H_
