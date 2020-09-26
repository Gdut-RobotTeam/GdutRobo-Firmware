//
// Created by luohx on 2020/9/25.
//

#ifndef FIRMWARE_CLION_BSP_BSP_MOTOR_H_
#define FIRMWARE_CLION_BSP_BSP_MOTOR_H_

#include "main.h"
#define MOTOR_PWM_MAX 7000

void motor_pwm_enable(void);

//motor: motor number
//pid_out: CCR MAX = 7200
//Positive values lead to positive transitions and negative values are reversals
void motor_set_pwm(uint8_t motor, int32_t pid_out);

int read_encoder(uint8_t TIMX);

#endif //FIRMWARE_CLION_BSP_BSP_MOTOR_H_
