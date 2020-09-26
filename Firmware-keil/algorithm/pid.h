#ifndef __PID_H
#define __PID_H

#include "struct_typedef.h"

struct Motors {
	float speed;
	int bias;
	int last_bias;
	int integration;
	int encoder;
	int target;
	};

struct PID {
	float kp;
	float ki;
	float kd;
	};


#endif
