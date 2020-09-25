#ifndef __BSP_LINEFOLLOWER_H
#define __BSP_LINEFOLLOWER_H

#include "struct_typedef.h"

struct Line_Grays
{
	uint16_t gray;
	float value;
	uint8_t num;
	uint8_t sign;
};

extern struct Line_Grays left_LFB;
extern struct Line_Grays rigth_LFB;

void LFB_ID1_Send(void);
void LFB_ID2_Send(void);

#endif

