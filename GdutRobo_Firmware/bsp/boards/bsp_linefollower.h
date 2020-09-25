#ifndef __BSP_LINEFOLLOWER_H
#define __BSP_LINEFOLLOWER_H

#include "struct_typedef.h"

//�Ƿ�ʹ�õڶ���ѭ������м���
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

//���ڽ���ѭ�����ʼ��
void LFB_receive_init(void);

//���ַΪ1��ѭ���巢�Ͳ�ѯָ��
void LFB_ID1_Send(void);

#ifdef LINE_FOLLOWER_ID2_ENABLE
//���ַΪ2��ѭ���巢�Ͳ�ѯָ��
//�ú����ڽ��մ����ַΪ1��ѭ��������ݺ�ᱻ�Զ�����
void LFB_ID2_Send(void);
#endif

#endif

