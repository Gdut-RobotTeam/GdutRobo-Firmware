#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "struct_typedef.h"

#define MOTOR_PWM_MAX 7000

void Motor_PWM_Enable(void);

void Motor_Set_PWM(uint8_t motor, int32_t pid_out);

int Read_Encoder(uint8_t TIMX);

#endif

