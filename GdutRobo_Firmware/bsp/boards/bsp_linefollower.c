#include "bsp_linefollower.h"
#include "usart.h"
#include "tim.h"

//测试代码////////////////////////////////////////////////
void LFB_Decode(struct Line_Grays *data, uint8_t *buffer, float *weight);

float weight1[10] = {-5,-4,-3,-2,-0.5,0.5,2,3,4,5};
struct Line_Grays left_LFB;
struct Line_Grays rigth_LFB;
//////////////////////////////////////////////////////////


// 校验和计算
static uint8_t calculate_verify(const uint8_t *data, uint8_t length)
{
	uint8_t i=0;
	register uint16_t verify_value = 0;
	
	for (i=0; i<length; i++)
		verify_value += data[i];
	
	return (uint8_t)(verify_value & 0XFF);
}


void LFB_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);     //开启接收完成中断
}


//向地址为1的循迹板发送查询指令
void LFB_ID1_Send(void)
{
	uint8_t getlinecom[]={0xff,0xff,0x01,0x02,0x08,0x0b};  //ID:0X01
	HAL_UART_Transmit(&huart4, getlinecom, 6, 0XFF);
}

#ifdef LINE_FOLLOWER_ID2_ENABLE
//向地址为2的循迹板发送查询指令
//该函数在接收处理地址为1的循迹板的数据后会被自动调用
void LFB_ID2_Send(void)
{
	uint8_t getlinecom[]={0xff,0xff,0x02,0x02,0x08,0x0c};  //ID:0X02
	HAL_UART_Transmit(&huart4, getlinecom, 6, 0XFF);
}
#endif


void UART4_IRQHandler(void)
{
	uint8_t Res;
	uint8_t id2 = 0;
	static uint8_t RxBuffer[10];
	static uint8_t data_cnt;
	static uint8_t state;
	
	if (UART4->SR & (1<<5))
	{
		Res = UART4->DR;
		if(state==2)  //两个0xff已接收
		{
			RxBuffer[data_cnt] = Res;
			if (data_cnt >= 6)
			{
				if (calculate_verify(RxBuffer, 6) == RxBuffer[6])
				{
					if (RxBuffer[0]==0x01)
					{
						LFB_Decode(&left_LFB, RxBuffer, weight1);
						id2 = 1;  //查询ID2循迹板
					}
					#ifdef LINE_FOLLOWER_ID2_ENABLE
					else if (RxBuffer[0]==0x02)
					{
						LFB_Decode(&rigth_LFB, RxBuffer, weight1);
					}
					#endif
				}
				state = 0;
			}
			++data_cnt;
		}
		else if (Res==0xff && state==0)
			state=1;
		else if (state==1 && Res==0xff)
		{
			state=2;
			data_cnt=0;
		}
		else
			state=0;
		
		#ifdef LINE_FOLLOWER_ID2_ENABLE
		if (state==0 && id2==1)
		{
			LFB_ID2_Send();
			id2 = 0;
		}
		#endif
	}
}


//测试函数
void LFB_Decode(struct Line_Grays *data, uint8_t *buffer, float *weight)
{
	uint8_t i,a;
	float tmp=0;
	data->gray = (buffer[5])|(buffer[4]<<4)|(buffer[3]<<8);
	data->num=0;
	for(i=0;i<10;i++)
	{
		a=(data->gray>>i)&0X01;
		data->num+=a;
		tmp+=a*weight[i];
	}
	if(data->num!=0)
		data->value=tmp/data->num;
	else
		data->value=0;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim6))
    {
        LFB_ID1_Send();
    }
}







