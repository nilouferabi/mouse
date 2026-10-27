/****************************************Copyright (c)****************************************************
**                               
**                                     
**                                 
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           IIC.h
** Last modified Date:  2013/11/20
** Last Version:        V1.00
** Description:         AT24C02针对STM32的驱动程序头文件
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          陈杰鸿
** Created date: 
** Version: 
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
*********************************************************************************************************/
#ifndef _iic_H_
#define _iic_H_
#include "stm32f10x.h"
#include "BitBand.h"
#include "Type.h"
// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_XOUT_H	0x3B	//0
#define	ACCEL_XOUT_L	0x3C	//1
#define	ACCEL_YOUT_H	0x3D	//2
#define	ACCEL_YOUT_L	0x3E	//3     
#define	ACCEL_ZOUT_H	0x3F	//4
#define	ACCEL_ZOUT_L	0x40	//5
#define	TEMP_OUT_H		0x41	//6
#define	TEMP_OUT_L		0x42	//7
#define	GYRO_XOUT_H		0x43	//8
#define	GYRO_XOUT_L		0x44	//9
#define	GYRO_YOUT_H		0x45	//10
#define	GYRO_YOUT_L		0x46	//11
#define	GYRO_ZOUT_H		0x47  //12
#define	GYRO_ZOUT_L		0x48	//13

#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress	0xD0	//IIC写入时的地址字节数据，+1为读取
/*********************************************************************************************************
  ZLG7289端口定义
*********************************************************************************************************/       

#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=8<<28;}
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=3<<28;}

 
#define IIC_SCL    PBout(6) 
#define IIC_SDA    PBout(7)  
#define READ_SDA   PBin(7)  
	 
extern void IIC_Init(void);
extern u32 read_fm24LC16(u32 *array,u32 address,u32 control,u32 len);
extern u32 write_fm24LC16(u32 *array,u32 address,u32 control,u32 len);
uint16 GetData(uint8 REG_Address);
void MPU6050_Init(void);
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
