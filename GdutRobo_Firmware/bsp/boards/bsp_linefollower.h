#ifndef __BSP_LINEFOLLOWER_H
#define __BSP_LINEFOLLOWER_H

#include "struct_typedef.h"

//是否使用第二块循迹板进行级联
#define LINE_FOLLOWER_ID2_ENABLE

struct Line_Grays
{
	uint16_t gray;
	float value;
	uint8_t num;
	uint8_t sign;
};

extern struct Line_Grays left_LFB;
extern struct Line_Grays rigth_LFB;

//串口接收循迹板初始化
void LFB_receive_init(void);

//向地址为1的循迹板发送查询指令
void LFB_ID1_Send(void);

#ifdef LINE_FOLLOWER_ID2_ENABLE
//向地址为2的循迹板发送查询指令
//该函数在接收处理地址为1的循迹板的数据后会被自动调用
void LFB_ID2_Send(void);
#endif

#endif

