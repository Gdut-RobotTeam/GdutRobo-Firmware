#ifndef BSP_SERVO_IIC_H
#define BSP_SERVO_IIC_H
#include "struct_typedef.h"
#include "bsp_delay.h"

//GPIO simulate IIC
#define SDA_IN()  {GPIOB->CRL&=0XFFFFFFF0;GPIOB->CRL|=(uint32_t)8;} 
#define SDA_OUT() {GPIOB->CRL&=0XFFFFFFF0;GPIOB->CRL|=(uint32_t)3;}  
#define SDA_HIGH() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET)
#define SDA_LOW()  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)
#define SCL_HIGH() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET)
#define SCL_LOW()  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET)
#define READ_SDA() HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)

//IIC operating function			 
void iic_start(void);				
void iic_stop(void);	  			
void iic_send_byte(uint8_t txd);			
uint8_t iic_read_byte(unsigned char ack);
uint8_t iic_wait_ack(void); 				
void iic_ack(void);					
void iic_not_ack(void);				

//pca9685
#define pca_adrr 0x80

#define pca_mode1 0x0
#define pca_pre 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define jdMIN  115 
#define jdMAX  590 
#define jd000  130 //0 degrees ~ (4096)
#define jd180  520 //180 degrees ~ (4096)

void pca_write(uint8_t adrr,uint8_t data);
uint8_t pca_read(uint8_t adrr);
void pca_setfreq(float freq);
void pca_setpwm(uint8_t num, uint32_t on, uint32_t off);
extern void pca_init(float hz,uint8_t angle);
extern void angle_write(uint8_t num,uint8_t angle);//angle:Absolute Angle

#endif

