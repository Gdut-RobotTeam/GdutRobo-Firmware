#include "pid.h"

struct Motors motor1;

void Incremental_PID (struct Motors *motor, struct PID *pid)
{
	motor->bias = motor->encoder - motor->target;
	motor->speed += pid->kp*(motor->bias - motor->last_bias) + 
					pid->ki*motor->bias;
	motor->last_bias = motor->bias;
}
