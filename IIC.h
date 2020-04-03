#ifndef __IIC_H
#define __IIC_H
#include "stm32f10x.h"

//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))// 位带操作
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))  

//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) // 0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) // 0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) // 0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) // 0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) // 0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) // 0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) // 0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) // 0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) // 0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) // 0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) // 0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) // 0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) // 0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) // 0x40011E08 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  // 输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  // 输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  // 输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  // 输入 
   	   		   
//IO方向设置， 先置0，在与操作，保证操作完的值正常
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0x00008000;}// 上下拉输入，此时电平由外部上拉或下拉电阻决定
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0x00003000;}// 通用推挽输出

//IO操作函数	 
#define IIC_SCL    PBout(10) // SCL线
#define IIC_SDA    PBout(11) // SDA线
#define READ_SDA   PBin(11)  // 输入SDA线

//IIC所有操作函数
void IIC_Init(void);// 初始化IIC的IO口				 
void IIC_Start(void);// 发送IIC起始信号
void IIC_Stop(void);// 发送IIC停止信号
void IIC_Send_Byte(u8 txd);// IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);// IIC读取一个字节
u8 IIC_Wait_Ack(void);//IIC等待应答信号
void IIC_Ack(void);// IIC发送ACK信号
void IIC_NAck(void);//IIC发送非应答信号

//void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
//u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
//unsigned char I2C_Readkey(unsigned char I2C_Addr);

//unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
//unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
//unsigned char IICwriteCmd(unsigned char dev, unsigned char cmd);
//u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
//u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);// IIC写指定长度的字节 dev：从设备地址 reg：寄存器地址 length：长度 data：要写的数据
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);// IIC读指定长度的字节 dev：从设备地址 reg：寄存器地址 length：长度 data：要读到的内存地址

#endif

//------------------End of File----------------------------//
