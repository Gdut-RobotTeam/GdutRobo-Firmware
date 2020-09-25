#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "struct_typedef.h"

#define MOTOR_PWM_MAX 7000

//使能定时器PWM通道
void Motor_PWM_Enable(void);

//电机PWM设置函数
//motor: 电机编号
//pid_out: 定时器CCR值，最大为7200，函数内限幅MOTOR_PWM_MAX，正值为正转，负值为反转
void Motor_Set_PWM(uint8_t motor, int32_t pid_out);

int Read_Encoder(uint8_t TIMX);

#endif

