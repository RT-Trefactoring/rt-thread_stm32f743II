/*
*********************************************************************************************************
*
*	模块名称 : 气压强度传感器BMP085驱动模块
*	文件名称 : bsp_bmp085.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2012-2013, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include <rtthread.h>
#ifndef _BSP_BMP085_H
#define _BSP_BMP085_H

#define BMP085_SLAVE_ADDRESS    0xEC		/* I2C从机地址 */

typedef struct _BMP085_T
{
	/* 用于保存芯片内部EEPROM的校准参数 */

	rt_uint16_t dig_T1; 
	rt_int16_t dig_T2;
	rt_int16_t dig_T3;
	rt_uint16_t dig_P1;
	rt_int16_t dig_P2;
	rt_int16_t dig_P3; 
	rt_int16_t dig_P4; 
	rt_int16_t dig_P5; 
	rt_int16_t dig_P6; 
	rt_int16_t dig_P7; 
	rt_int16_t dig_P8;
	rt_int16_t dig_P9;
	rt_int16_t reserved ;
	rt_int8_t OSS;	/* 过采样值，可有用户自己设定 */

	/* 下面2个单元用于存放计算的真实值 */
	volatile rt_int32_t Temp;	/* 温度值， 单位 0.1摄氏度 */
	volatile rt_int32_t Press;	/* 压力值， 单位 Pa */
}BMP085_T;

extern BMP085_T g_tBMP085;


void i2c_Delay1(void);
void i2c_Start1(void);
void i2c_Stop1(void);
void Bmp_InitI2C(void);
rt_uint8_t i2c_WaitAck1(void);
void i2c_Ack1(void);
void i2c_NAck1(void);
void i2c_SendByte1(rt_uint8_t _ucByte);
rt_uint8_t i2cbmp_CheckDevice(rt_uint8_t _Address);
rt_uint8_t i2c_ReadByte1(void);


void bsp_InitBMP085(void);
void BMP085_ReadTempPress(void);

static void BMP085_WriteReg(rt_uint8_t _ucRegAddr, rt_uint8_t _ucRegValue);
static rt_uint16_t BMP085_Read2Bytes(rt_uint8_t _ucRegAddr);
static long BMP085_Read3Bytes(rt_uint8_t _ucRegAddr);
static void BMP085_WaitConvert(void);
double get_pressure(void);
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
