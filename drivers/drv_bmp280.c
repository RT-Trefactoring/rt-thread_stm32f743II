/*******************************************************************************
 *   Filename:      bsp_i2c_gpio.c
 *   Revised:       $Date: 2016-05-11
 *   Revision:      $
 *   Writer:        Roger-WY.
 *
 *   Description:   ��gpioģ��i2c����
 *                  ��ģ�鲻����Ӧ�ò�����֡��������I2C���߻�������������
 *   Notes:         �ڷ���I2C�豸ǰ�����ȵ��� i2c_CheckDevice() ���I2C�豸�Ƿ��������ú���������GPIO
 *
 *
 *   All copyrights reserved to Roger-WY.
 *
 *******************************************************************************/
#include <rtthread.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "board.h"
#include "drv_bmp280.h"

#include "drv_dwt.h"
#define I2C_WR	0		/* д����bit */
#define I2C_RD	1		/* ������bit */


/***********************************************
* ������ ����I2C�������ӵ�GPIO�˿�
*        ��Ҫ����IO��ʱ�ӣ�����XXX_XXX_RCC����
*/
#define  I2C_SCL_PORT1   GPIOA
#define  I2C_SCL_PIN1    GPIO_PIN_3

#define  I2C_SDA_PORT1   GPIOD
#define  I2C_SDA_PIN1    GPIO_PIN_5


/* �����дSCL��SDA�ĺ� */
#define I2C_SCL_1()  GPIOA->BSRRL    |=  (0x01<<3)			/* SCL = 1 */
#define I2C_SCL_0()  GPIOA->BSRRH    |=  (0x01<<3)	/* SCL = 0 */


#define I2C_SDA_1()  GPIOD->BSRRL    |=  (0x01<<5)			/* SDA = 1 */
#define I2C_SDA_0()  GPIOD->BSRRH    |=  (0x01<<5)		/* SDA = 0 */

//#define I2C_SDA_READ()  ((I2C_SDA_PORT1->IDR & (uint8_t)I2C_SDA_PIN1) != 0)	/* ��SDA����״̬ */
//#define I2C_SCL_READ()  ((I2C_SCL_PORT1->IDR & (uint8_t)I2C_SCL_PIN1) != 0)	/* ��SCL����״̬ */


#define I2C_SDA_READ()  ((GPIOD->IDR>>5) & 0x01)	/* ��SDA����״̬ */
#define I2C_SCL_READ()  ((GPIOA->IDR>>3) & 0x01)	/* ��SCL����״̬ */

/*******************************************************************************
 * ��    �ƣ� i2c_Delay
 * ��    �ܣ� I2C����λ�ӳ٣����400KHz
 * ��ڲ����� ��
 * ���ڲ����� ��
 * �������ߣ� Roger-WY
 * �������ڣ� 2016-05-20
 * ��    �ģ�
 * �޸����ڣ�
 * ��    ע��
 *******************************************************************************/
void i2c_Delay1(void)
{
	rt_uint8_t i;
//	/*��ʵ��Ӧ��ѡ��400KHz���ҵ����ʼ��� */
//	for (i = 0; i < 60; i++);//100K 
	bsp_DelayUS(6);
}

/*******************************************************************************
 * ��    �ƣ� i2c_Start
 * ��    �ܣ� CPU����I2C���������ź�
 * ��ڲ����� ��
 * ���ڲ����� ��
 * �������ߣ� Roger-WY
 * �������ڣ� 2016-05-20
 * ��    �ģ�
 * �޸����ڣ�
 * ��    ע�� ��ʼʱ��
 *            SCL ������������\____
 *            SDA ��������\______
 *                  |   |
 *                  START
 *******************************************************************************/
void i2c_Start1(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
	I2C_SDA_1();
	i2c_Delay1();
	I2C_SCL_1();
	i2c_Delay1();

	I2C_SDA_0();
	i2c_Delay1();
	I2C_SCL_0();

	i2c_Delay1();
	
}

/*******************************************************************************
 * ��    �ƣ� i2c_Stop
 * ��    �ܣ� CPU����I2C����ֹͣ�ź�
 * ��ڲ����� ��
 * ���ڲ����� ��
 * �������ߣ� Roger-WY
 * �������ڣ� 2016-05-20
 * ��    �ģ�
 * �޸����ڣ�
 * ��    ע�� ֹͣʱ��
 *            SCL _____/��������������
 *            SDA _________/����������
 *                       |   |
 *                       STOP
 *******************************************************************************/
void i2c_Stop1(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */

	I2C_SCL_0();
	i2c_Delay1();
    I2C_SDA_0();
    i2c_Delay1();

	I2C_SCL_1();
	i2c_Delay1();
	I2C_SDA_1();
	
	i2c_Delay1();
	
}
/*******************************************************************************
 * ��    �ƣ� Bsp_InitI2C
 * ��    �ܣ� ����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
 * ��ڲ����� ��
 * ���ڲ����� ��
 * �������ߣ� Roger-WY
 * �������ڣ� 2016-05-20
 * ��    �ģ�
 * �޸����ڣ�
 * ��    ע��
 *******************************************************************************/
void Bmp_InitI2C(void)
{
   
    GPIO_InitTypeDef GPIO_Initure;

     /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	
	GPIO_Initure.Pin=GPIO_PIN_3;            //PA3
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_OD;  //�������
    GPIO_Initure.Pull=GPIO_NOPULL;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);     //��ʼ��
	
	
    GPIO_Initure.Pin=GPIO_PIN_5;            //PD5
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_OD;  //�������
    GPIO_Initure.Pull=GPIO_NOPULL;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //����
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);     //


    /* ��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ */
	i2c_Stop1();
}

/*******************************************************************************
 * ��    �ƣ� i2c_WaitAck
 * ��    �ܣ� CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
 * ��ڲ����� ��
 * ���ڲ����� ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
 * �������ߣ� Roger-WY
 * �������ڣ� 2016-05-20
 * ��    �ģ�
 * �޸����ڣ�
 * ��    ע��
 *******************************************************************************/
rt_uint8_t i2c_WaitAck1(void)
{
	rt_uint8_t re;
    rt_uint8_t TimeOutCnt = 20;  /* ��ʱ������ */

	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
	i2c_Delay1();
	I2C_SCL_1();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
	i2c_Delay1();

    while(TimeOutCnt -- ) {
        if (I2C_SDA_READ())	{/* CPU��ȡSDA����״̬ */
            re = 1;
        } else {
            re = 0;
        }
    }
	I2C_SCL_0();
	i2c_Delay1();
	return re;
}

/*******************************************************************************
 * ��    �ƣ� i2c_Ack
 * ��    �ܣ� CPU����һ��ACK�ź�
 * ��ڲ����� ��
 * ���ڲ����� ��
 * �������ߣ� Roger-WY
 * �������ڣ� 2016-05-20
 * ��    �ģ�
 * �޸����ڣ�
 * ��    ע��
 *******************************************************************************/
void i2c_Ack1(void)
{
	I2C_SDA_0();	/* CPU����SDA = 0 */
	i2c_Delay1();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay1();
	I2C_SCL_0();
	i2c_Delay1();
	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
}

/*******************************************************************************
 * ��    �ƣ� i2c_NAck
 * ��    �ܣ� CPU����1��NACK�ź�
 * ��ڲ����� ��
 * ���ڲ����� ��
 * �������ߣ� Roger-WY
 * �������ڣ� 2016-05-20
 * ��    �ģ�
 * �޸����ڣ�
 * ��    ע��
 *******************************************************************************/
void i2c_NAck1(void)
{
	I2C_SDA_1();	/* CPU����SDA = 1 */
	i2c_Delay1();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay1();
	I2C_SCL_0();
	i2c_Delay1();
}
/*******************************************************************************
 * ��    �ƣ� i2c_SendByte
 * ��    �ܣ� CPU��I2C�����豸����8bit����
 * ��ڲ����� _ucByte �� �ȴ����͵��ֽ�
 * ���ڲ����� ��
 * �������ߣ� Roger-WY
 * �������ڣ� 2016-05-20
 * ��    �ģ�
 * �޸����ڣ�
 * ��    ע��
 *******************************************************************************/
void i2c_SendByte1(rt_uint8_t _ucByte)
{
	rt_uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
	for (i = 0; i < 8; i++) {
        I2C_SCL_0();
		i2c_Delay1();

		if (_ucByte & 0x80) {
			I2C_SDA_1();
		} else {
			I2C_SDA_0();
		}

        _ucByte <<= 1;	/* ����һ��bit */

		i2c_Delay1();

		I2C_SCL_1();
		i2c_Delay1();
	}
    I2C_SCL_0();
	i2c_Delay1();

}

/*******************************************************************************
 * ��    �ƣ� i2c_CheckDevice
 * ��    �ܣ� ���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
 * ��ڲ�����  _Address���豸��I2C���ߵ�ַ
 * ���ڲ����� ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
 * �������ߣ� Roger-WY
 * �������ڣ� 2016-05-20
 * ��    �ģ�
 * �޸����ڣ�
 * ��    ע��
 *******************************************************************************/
rt_uint8_t i2cbmp_CheckDevice(rt_uint8_t _Address)
{
	rt_uint8_t ucAck;
    Bmp_InitI2C();
	if (I2C_SDA_READ() && I2C_SCL_READ()) {
		i2c_Start1();		/* ���������ź� */
 
		/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
		i2c_SendByte1(_Address | I2C_WR);
		ucAck = i2c_WaitAck1();	/* ����豸��ACKӦ�� */

		i2c_Stop1();			/* ����ֹͣ�ź� */

		return ucAck;
	}
	return 1;	            /* I2C�����쳣 */
}



/*******************************************************************************
 * ��    �ƣ� i2c_ReadByte
 * ��    �ܣ� CPU��I2C�����豸��ȡ8bit����
 * ��ڲ����� ��
 * ���ڲ����� ����������
 * �������ߣ� Roger-WY
 * �������ڣ� 2016-05-20
 * ��    �ģ�
 * �޸����ڣ�
 * ��    ע��
 *******************************************************************************/
rt_uint8_t i2c_ReadByte1(void)
{
	rt_uint8_t i;
	rt_uint8_t value;

	/* ������1��bitΪ���ݵ�bit7 */
	value = 0;
	for (i = 0; i < 8; i++) {
		value <<= 1;
		I2C_SCL_0();
		i2c_Delay1();
        I2C_SCL_1();
		i2c_Delay1();

		if (I2C_SDA_READ()) {
			value++;
		}
	}
    I2C_SCL_0();
	i2c_Delay1();

	return value;
}

/*-------------- end of file ---------------*/
#define BMP280_I2C_ADDR					(0x76)
#define BMP280_DEFAULT_CHIP_ID			(0x58)

#define BMP280_CHIP_ID					(0xD0)  /* Chip ID Register */
#define BMP280_RST_REG					(0xE0)  /* Softreset Register */
#define BMP280_STAT_REG					(0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG			(0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG				(0xF5)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG			(0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG			(0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG		(0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG		(0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG		(0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG		(0xFC)  /* Temperature XLSB Reg */

#define BMP280_SLEEP_MODE				(0x00)
#define BMP280_FORCED_MODE				(0x01)
#define BMP280_NORMAL_MODE				(0x03)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE			(6)

#define BMP280_OVERSAMP_SKIPPED			(0x00)
#define BMP280_OVERSAMP_1X				(0x01)
#define BMP280_OVERSAMP_2X				(0x02)
#define BMP280_OVERSAMP_4X				(0x03)
#define BMP280_OVERSAMP_8X				(0x04)
#define BMP280_OVERSAMP_16X				(0x05)

#define BMP280_PRESSURE_OSR			(BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR		(BMP280_OVERSAMP_8X)
#define BMP280_MODE					(BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_NORMAL_MODE)
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitBMP085
*	����˵��: ��ʼ��BMP085
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitBMP085(void)
{
	rt_uint8_t ctrlmeas_reg,config_reg;
	
	/* ����оƬ�ڲ���У׼������ÿ��оƬ��ͬ������BOSCH����ǰУ׼�õ����ݣ� */
	g_tBMP085.dig_T1 =  (rt_uint16_t)BMP085_Read2Bytes(0x88);
	g_tBMP085.dig_T2 =  (rt_int16_t)BMP085_Read2Bytes(0x8A);
	g_tBMP085.dig_T3 =  (rt_int16_t)BMP085_Read2Bytes(0x8C);
	g_tBMP085.dig_P1 =  (rt_uint16_t)BMP085_Read2Bytes(0x8E);
	g_tBMP085.dig_P2 =  (rt_int16_t)BMP085_Read2Bytes(0x90);
	g_tBMP085.dig_P3 =  (rt_int16_t)BMP085_Read2Bytes(0x92);
	g_tBMP085.dig_P4 =  (rt_int16_t)BMP085_Read2Bytes(0x94);
	g_tBMP085.dig_P5 =  (rt_int16_t)BMP085_Read2Bytes(0x96);
	g_tBMP085.dig_P6 =  (rt_int16_t)BMP085_Read2Bytes(0x98);
	g_tBMP085.dig_P7 =  (rt_int16_t)BMP085_Read2Bytes(0x9A);
	g_tBMP085.dig_P8 =  (rt_int16_t)BMP085_Read2Bytes(0x9C);
	g_tBMP085.dig_P9 =  (rt_int16_t)BMP085_Read2Bytes(0x9E);
	g_tBMP085.reserved =  (rt_int16_t)BMP085_Read2Bytes(0xA0);
	g_tBMP085.OSS =0;	/* ������������0-3 */

	BMP085_WriteReg(0xE0,0xb6);//reset reg
	
//	ctrlmeas_reg = (0x01 << 5) | (0x01<< 2) | 0x01;
//	config_reg = (0x00<< 5 )| (0x01 << 2)|0x00;
	BMP085_WriteReg(BMP280_MODE, ctrlmeas_reg);
	BMP085_WriteReg(5<<2, config_reg);

//	BMP085_WriteReg(0xF4, 0xFF);
//	BMP085_WriteReg(0xF5, 0x14);
	
//	BMP085_WriteReg(0xf4,0xb3);
//	BMP085_WriteReg(0xf5,5<<2);
	rt_thread_delay(RT_TICK_PER_SECOND/5);
}

/*
*********************************************************************************************************
*	�� �� ��: BMP085_WriteReg
*	����˵��: д�Ĵ���
*	��    ��: _ucOpecode : �Ĵ�����ַ
*			  _ucRegData : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void BMP085_WriteReg(rt_uint8_t _ucRegAddr, rt_uint8_t _ucRegValue)
{
    i2c_Start1();							/* ���߿�ʼ�ź� */

    i2c_SendByte1(BMP085_SLAVE_ADDRESS);		/* �����豸��ַ+д�ź� */
	i2c_WaitAck1();

    i2c_SendByte1(_ucRegAddr);				/* ���ͼĴ�����ַ */
	i2c_WaitAck1();

    i2c_SendByte1(_ucRegValue);				/* ���ͼĴ�����ֵ */
	i2c_WaitAck1();

    i2c_Stop1();                   			/* ����ֹͣ�ź� */
}

/*
*********************************************************************************************************
*	�� �� ��: BMP085_Read2Bytes
*	����˵��: ��ȡBMP085�Ĵ�����ֵ������2�ֽڡ������¶ȼĴ���
*	��    ��: _ucRegAddr �Ĵ�����ַ
*	�� �� ֵ: �Ĵ���ֵ
*********************************************************************************************************
*/

static rt_uint16_t BMP085_Read2Bytes(rt_uint8_t _ucRegAddr)
{
	rt_uint8_t ucData1;
	rt_uint8_t ucData2;
	rt_uint16_t usRegValue;

	i2c_Start1();                  			/* ���߿�ʼ�ź� */
	i2c_SendByte1(BMP085_SLAVE_ADDRESS);		/* �����豸��ַ+д�ź� */
	i2c_WaitAck1();
	i2c_SendByte1(_ucRegAddr);				/* ���͵�ַ */
	i2c_WaitAck1();

	i2c_Start1();                  			/* ���߿�ʼ�ź� */
	i2c_SendByte1(BMP085_SLAVE_ADDRESS + 1);/* �����豸��ַ+���ź� */
	i2c_WaitAck1();

	ucData1 = i2c_ReadByte1();       		/* �������ֽ����� */
	i2c_Ack1();

	ucData2 = i2c_ReadByte1();       		/* �������ֽ����� */
	i2c_NAck1();
	i2c_Stop1();                  			/* ����ֹͣ�ź� */

	usRegValue = (ucData1 << 8) + ucData2;

	return usRegValue;
}

/*
*********************************************************************************************************
*	�� �� ��: BMP085_Read3Bytes
*	����˵��: ��ȡBMP085�Ĵ�����ֵ������3�ֽ�  ���ڶ�ѹ���Ĵ���
*	��    ��: _ucRegAddr �Ĵ�����ַ
*	�� �� ֵ: �Ĵ���ֵ
*********************************************************************************************************
*/
static long BMP085_Read3Bytes(rt_uint8_t _ucRegAddr)
{
	rt_uint8_t ucData1;
	rt_uint8_t ucData2;
	rt_uint8_t ucData3;
	long uiRegValue;

	i2c_Start1();                  			/* ���߿�ʼ�ź� */
	i2c_SendByte1(BMP085_SLAVE_ADDRESS);		/* �����豸��ַ+д�ź� */
	i2c_WaitAck1();
	i2c_SendByte1(_ucRegAddr);				/* ���͵�ַ */
	i2c_WaitAck1();

	i2c_Start1();                  			/* ���߿�ʼ�ź� */
	i2c_SendByte1(BMP085_SLAVE_ADDRESS + 1);/* �����豸��ַ+���ź� */
	i2c_WaitAck1();

	ucData1 = i2c_ReadByte1();       		/* �������ֽ����� */
	i2c_Ack1(); 

	ucData2 = i2c_ReadByte1();       		/* �����м��ֽ����� */
	i2c_Ack1();

	ucData3 = i2c_ReadByte1();       		/* ������ͽ����� */
	i2c_NAck1();
	i2c_Stop1();                  			/* ����ֹͣ�ź� */

	uiRegValue = (long)((ucData1 << 12) | (ucData2 << 4) | (ucData3>>4));

	return uiRegValue;
}

/*
*********************************************************************************************************
*	�� �� ��: BMP085_WaitConvert
*	����˵��: �ȴ��ڲ�ת������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void BMP085_WaitConvert(void)
{
	if (g_tBMP085.OSS == 0)
	{
		rt_thread_delay( RT_TICK_PER_SECOND*6/1000);		/* 4.5ms  7.5ms  13.5ms   25.5ms */
	}
	else if (g_tBMP085.OSS == 1)
	{
		rt_thread_delay( RT_TICK_PER_SECOND*9/1000);	
	}
	else if (g_tBMP085.OSS == 2)
	{
		rt_thread_delay( RT_TICK_PER_SECOND*15/1000);	
	
	}
	else if (g_tBMP085.OSS == 3)
	{
		
      rt_thread_delay( RT_TICK_PER_SECOND*27/1000);	
	}
}

long bmp280Convert(void)
{
	long adc_T;
	long adc_P;
	long var1, var2,t_fine,T,p; 
	
	adc_T = BMP085_Read3Bytes(0xFA);    // ��ȡ�¶�
	BMP085_WaitConvert();
	
	adc_P = BMP085_Read3Bytes(0xF7); // ��ȡѹǿ
	BMP085_WaitConvert();
	
	if(adc_P == 0)
	{
		return 0;
	}
	//Temperature
	var1 = (((double)adc_T)/16384.0-((double)g_tBMP085.dig_T1)/1024.0)*((double)g_tBMP085.dig_T2);
	//rt_kprintf("var1 is %d\n",var1);
	var2 = ((((double)adc_T)/131072.0-((double)g_tBMP085.dig_T1)/8192.0)*(((double)adc_T)/131072.0-((double)g_tBMP085.dig_T1)/8192.0))*((double)g_tBMP085.dig_T3);
	//rt_kprintf("var2 is %d\n",var2);
	
	t_fine =(var1+var2);
	//rt_kprintf("t_fine is %d\n",t_fine);
	
	T = (var1+var2)/5120.0;
	T = T;
	rt_kprintf("temperature is %ld\n",(rt_int32_t)T);
	
	var1 = ((double)t_fine/2.0)-64000.0;
	//rt_kprintf("var1 is %d\n",var1);
	
	var2 = var1*var1*((double)g_tBMP085.dig_P6)/32768.0;
	//rt_kprintf("var2 is %d\n",var2);
	
	var2 = var2+var1*((double)g_tBMP085.dig_P5)*2.0;
	//rt_kprintf("var2 is %d\n",var2);
	
	var2 = (var2/4.0)+(((double)g_tBMP085.dig_P4)*65536.0);
	//rt_kprintf("var2 is %d\n",var2);
	
	var1 = (((double)g_tBMP085.dig_P3)*var1*var1/524288.0+((double)g_tBMP085.dig_P2)*var1)/524288.0;
	//rt_kprintf("var1 is %d\n",var1);
	
	var1 = (1.0+var1/32768.0)*((double)g_tBMP085.dig_P1);
	//rt_kprintf("var1 is %d\n",var1);
	
	p = 1048576.0-(double)adc_P;
	//rt_kprintf("p is %d\n",p);

	p = (p-(var2/4096.0))*6250.0/var1;
	//rt_kprintf("p is %d\n",p);
	
	var1 = ((double)g_tBMP085.dig_P9)*p*p/2147483648.0;
	//rt_kprintf("var1 is %d\n",var1);
	
	var2 = p*((double)g_tBMP085.dig_P8)/32768.0;
	//rt_kprintf("var2 is %d\n",var2);
	
	p = p+(var1+var2+((double)g_tBMP085.dig_P7))/16.0;
	 
	rt_kprintf("pressure is %d\n",p);
	return p;
}

double get_pressure(void)
{ 	
	double pressure = 0;
	int i = 0;
	rt_thread_delay( RT_TICK_PER_SECOND);
	
	//while(i < 3)
	//{
		pressure = bmp280Convert();
	//	i++;
	//}

	//rt_kprintf("pressure_end is %d\n",pressure);
	//BMP085_WriteReg(0xf4,0x24);
	return pressure;
} 
