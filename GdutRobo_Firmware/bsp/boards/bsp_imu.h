#ifndef BSP_LED_H
#define BSP_LED_H
#include "struct_typedef.h"
#include "string.h"

#define BUFFER_SIZE  200

struct Imu
{
	float yaw;
	float roll;
	float pitch;
	float ax;
	float ay;
	float az;
	float bias;
};

extern void imu_receive_init(void);
extern struct Imu imu;

#endif
