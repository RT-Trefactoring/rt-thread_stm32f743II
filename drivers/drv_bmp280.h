/*
*********************************************************************************************************
*
*	ģ������ : ��ѹǿ�ȴ�����BMP085����ģ��
*	�ļ����� : bsp_bmp085.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2012-2013, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include <rtthread.h>
#ifndef _BSP_BMP085_H
#define _BSP_BMP085_H

#define BMP085_SLAVE_ADDRESS    0xEC		/* I2C�ӻ���ַ */

typedef struct _BMP085_T
{
	/* ���ڱ���оƬ�ڲ�EEPROM��У׼���� */

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
	rt_int8_t OSS;	/* ������ֵ�������û��Լ��趨 */

	/* ����2����Ԫ���ڴ�ż������ʵֵ */
	volatile rt_int32_t Temp;	/* �¶�ֵ�� ��λ 0.1���϶� */
	volatile rt_int32_t Press;	/* ѹ��ֵ�� ��λ Pa */
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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
