/*******************************************************************************
 *   Filename:      bsp_i2c_gpio.c
 *   Revised:       $Date: 2016-05-11
 *   Revision:      $
 *   Writer:        Roger-WY.
 *
 *   Description:   用gpio模拟i2c总线
 *                  该模块不包括应用层命令帧，仅包括I2C总线基本操作函数。
 *   Notes:         在访问I2C设备前，请先调用 i2c_CheckDevice() 检测I2C设备是否正常，该函数会配置GPIO
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
#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */


/***********************************************
* 描述： 定义I2C总线连接的GPIO端口
*        需要配置IO口时钟，配置XXX_XXX_RCC即可
*/
#define  I2C_SCL_PORT1   GPIOA
#define  I2C_SCL_PIN1    GPIO_PIN_3

#define  I2C_SDA_PORT1   GPIOD
#define  I2C_SDA_PIN1    GPIO_PIN_5


/* 定义读写SCL和SDA的宏 */
#define I2C_SCL_1()  GPIOA->BSRRL    |=  (0x01<<3)			/* SCL = 1 */
#define I2C_SCL_0()  GPIOA->BSRRH    |=  (0x01<<3)	/* SCL = 0 */


#define I2C_SDA_1()  GPIOD->BSRRL    |=  (0x01<<5)			/* SDA = 1 */
#define I2C_SDA_0()  GPIOD->BSRRH    |=  (0x01<<5)		/* SDA = 0 */

//#define I2C_SDA_READ()  ((I2C_SDA_PORT1->IDR & (uint8_t)I2C_SDA_PIN1) != 0)	/* 读SDA口线状态 */
//#define I2C_SCL_READ()  ((I2C_SCL_PORT1->IDR & (uint8_t)I2C_SCL_PIN1) != 0)	/* 读SCL口线状态 */


#define I2C_SDA_READ()  ((GPIOD->IDR>>5) & 0x01)	/* 读SDA口线状态 */
#define I2C_SCL_READ()  ((GPIOA->IDR>>3) & 0x01)	/* 读SCL口线状态 */

/*******************************************************************************
 * 名    称： i2c_Delay
 * 功    能： I2C总线位延迟，最快400KHz
 * 入口参数： 无
 * 出口参数： 无
 * 作　　者： Roger-WY
 * 创建日期： 2016-05-20
 * 修    改：
 * 修改日期：
 * 备    注：
 *******************************************************************************/
void i2c_Delay1(void)
{
	rt_uint8_t i;
//	/*　实际应用选择400KHz左右的速率即可 */
//	for (i = 0; i < 60; i++);//100K 
	bsp_DelayUS(6);
}

/*******************************************************************************
 * 名    称： i2c_Start
 * 功    能： CPU发起I2C总线启动信号
 * 入口参数： 无
 * 出口参数： 无
 * 作　　者： Roger-WY
 * 创建日期： 2016-05-20
 * 修    改：
 * 修改日期：
 * 备    注： 开始时序
 *            SCL ˉˉˉˉˉˉ\____
 *            SDA ˉˉˉˉ\______
 *                  |   |
 *                  START
 *******************************************************************************/
void i2c_Start1(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
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
 * 名    称： i2c_Stop
 * 功    能： CPU发起I2C总线停止信号
 * 入口参数： 无
 * 出口参数： 无
 * 作　　者： Roger-WY
 * 创建日期： 2016-05-20
 * 修    改：
 * 修改日期：
 * 备    注： 停止时序
 *            SCL _____/ˉˉˉˉˉˉˉ
 *            SDA _________/ˉˉˉˉˉ
 *                       |   |
 *                       STOP
 *******************************************************************************/
void i2c_Stop1(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */

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
 * 名    称： Bsp_InitI2C
 * 功    能： 配置I2C总线的GPIO，采用模拟IO的方式实现
 * 入口参数： 无
 * 出口参数： 无
 * 作　　者： Roger-WY
 * 创建日期： 2016-05-20
 * 修    改：
 * 修改日期：
 * 备    注：
 *******************************************************************************/
void Bmp_InitI2C(void)
{
   
    GPIO_InitTypeDef GPIO_Initure;

     /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	
	GPIO_Initure.Pin=GPIO_PIN_3;            //PA3
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_OD;  //推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);     //初始化
	
	
    GPIO_Initure.Pin=GPIO_PIN_5;            //PD5
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_OD;  //推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);     //


    /* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
	i2c_Stop1();
}

/*******************************************************************************
 * 名    称： i2c_WaitAck
 * 功    能： CPU产生一个时钟，并读取器件的ACK应答信号
 * 入口参数： 无
 * 出口参数： 返回0表示正确应答，1表示无器件响应
 * 作　　者： Roger-WY
 * 创建日期： 2016-05-20
 * 修    改：
 * 修改日期：
 * 备    注：
 *******************************************************************************/
rt_uint8_t i2c_WaitAck1(void)
{
	rt_uint8_t re;
    rt_uint8_t TimeOutCnt = 20;  /* 超时计数器 */

	I2C_SDA_1();	/* CPU释放SDA总线 */
	i2c_Delay1();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay1();

    while(TimeOutCnt -- ) {
        if (I2C_SDA_READ())	{/* CPU读取SDA口线状态 */
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
 * 名    称： i2c_Ack
 * 功    能： CPU产生一个ACK信号
 * 入口参数： 无
 * 出口参数： 无
 * 作　　者： Roger-WY
 * 创建日期： 2016-05-20
 * 修    改：
 * 修改日期：
 * 备    注：
 *******************************************************************************/
void i2c_Ack1(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay1();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay1();
	I2C_SCL_0();
	i2c_Delay1();
	I2C_SDA_1();	/* CPU释放SDA总线 */
}

/*******************************************************************************
 * 名    称： i2c_NAck
 * 功    能： CPU产生1个NACK信号
 * 入口参数： 无
 * 出口参数： 无
 * 作　　者： Roger-WY
 * 创建日期： 2016-05-20
 * 修    改：
 * 修改日期：
 * 备    注：
 *******************************************************************************/
void i2c_NAck1(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay1();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay1();
	I2C_SCL_0();
	i2c_Delay1();
}
/*******************************************************************************
 * 名    称： i2c_SendByte
 * 功    能： CPU向I2C总线设备发送8bit数据
 * 入口参数： _ucByte ： 等待发送的字节
 * 出口参数： 无
 * 作　　者： Roger-WY
 * 创建日期： 2016-05-20
 * 修    改：
 * 修改日期：
 * 备    注：
 *******************************************************************************/
void i2c_SendByte1(rt_uint8_t _ucByte)
{
	rt_uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++) {
        I2C_SCL_0();
		i2c_Delay1();

		if (_ucByte & 0x80) {
			I2C_SDA_1();
		} else {
			I2C_SDA_0();
		}

        _ucByte <<= 1;	/* 左移一个bit */

		i2c_Delay1();

		I2C_SCL_1();
		i2c_Delay1();
	}
    I2C_SCL_0();
	i2c_Delay1();

}

/*******************************************************************************
 * 名    称： i2c_CheckDevice
 * 功    能： 检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
 * 入口参数：  _Address：设备的I2C总线地址
 * 出口参数： 返回值 0 表示正确， 返回1表示未探测到
 * 作　　者： Roger-WY
 * 创建日期： 2016-05-20
 * 修    改：
 * 修改日期：
 * 备    注：
 *******************************************************************************/
rt_uint8_t i2cbmp_CheckDevice(rt_uint8_t _Address)
{
	rt_uint8_t ucAck;
    Bmp_InitI2C();
	if (I2C_SDA_READ() && I2C_SCL_READ()) {
		i2c_Start1();		/* 发送启动信号 */
 
		/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
		i2c_SendByte1(_Address | I2C_WR);
		ucAck = i2c_WaitAck1();	/* 检测设备的ACK应答 */

		i2c_Stop1();			/* 发送停止信号 */

		return ucAck;
	}
	return 1;	            /* I2C总线异常 */
}



/*******************************************************************************
 * 名    称： i2c_ReadByte
 * 功    能： CPU从I2C总线设备读取8bit数据
 * 入口参数： 无
 * 出口参数： 读到的数据
 * 作　　者： Roger-WY
 * 创建日期： 2016-05-20
 * 修    改：
 * 修改日期：
 * 备    注：
 *******************************************************************************/
rt_uint8_t i2c_ReadByte1(void)
{
	rt_uint8_t i;
	rt_uint8_t value;

	/* 读到第1个bit为数据的bit7 */
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
*	函 数 名: bsp_InitBMP085
*	功能说明: 初始化BMP085
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitBMP085(void)
{
	rt_uint8_t ctrlmeas_reg,config_reg;
	
	/* 读出芯片内部的校准参数（每个芯片不同，这是BOSCH出厂前校准好的数据） */
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
	g_tBMP085.OSS =0;	/* 过采样参数，0-3 */

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
*	函 数 名: BMP085_WriteReg
*	功能说明: 写寄存器
*	形    参: _ucOpecode : 寄存器地址
*			  _ucRegData : 寄存器数据
*	返 回 值: 无
*********************************************************************************************************
*/
static void BMP085_WriteReg(rt_uint8_t _ucRegAddr, rt_uint8_t _ucRegValue)
{
    i2c_Start1();							/* 总线开始信号 */

    i2c_SendByte1(BMP085_SLAVE_ADDRESS);		/* 发送设备地址+写信号 */
	i2c_WaitAck1();

    i2c_SendByte1(_ucRegAddr);				/* 发送寄存器地址 */
	i2c_WaitAck1();

    i2c_SendByte1(_ucRegValue);				/* 发送寄存器数值 */
	i2c_WaitAck1();

    i2c_Stop1();                   			/* 总线停止信号 */
}

/*
*********************************************************************************************************
*	函 数 名: BMP085_Read2Bytes
*	功能说明: 读取BMP085寄存器数值，连续2字节。用于温度寄存器
*	形    参: _ucRegAddr 寄存器地址
*	返 回 值: 寄存器值
*********************************************************************************************************
*/

static rt_uint16_t BMP085_Read2Bytes(rt_uint8_t _ucRegAddr)
{
	rt_uint8_t ucData1;
	rt_uint8_t ucData2;
	rt_uint16_t usRegValue;

	i2c_Start1();                  			/* 总线开始信号 */
	i2c_SendByte1(BMP085_SLAVE_ADDRESS);		/* 发送设备地址+写信号 */
	i2c_WaitAck1();
	i2c_SendByte1(_ucRegAddr);				/* 发送地址 */
	i2c_WaitAck1();

	i2c_Start1();                  			/* 总线开始信号 */
	i2c_SendByte1(BMP085_SLAVE_ADDRESS + 1);/* 发送设备地址+读信号 */
	i2c_WaitAck1();

	ucData1 = i2c_ReadByte1();       		/* 读出高字节数据 */
	i2c_Ack1();

	ucData2 = i2c_ReadByte1();       		/* 读出低字节数据 */
	i2c_NAck1();
	i2c_Stop1();                  			/* 总线停止信号 */

	usRegValue = (ucData1 << 8) + ucData2;

	return usRegValue;
}

/*
*********************************************************************************************************
*	函 数 名: BMP085_Read3Bytes
*	功能说明: 读取BMP085寄存器数值，连续3字节  用于读压力寄存器
*	形    参: _ucRegAddr 寄存器地址
*	返 回 值: 寄存器值
*********************************************************************************************************
*/
static long BMP085_Read3Bytes(rt_uint8_t _ucRegAddr)
{
	rt_uint8_t ucData1;
	rt_uint8_t ucData2;
	rt_uint8_t ucData3;
	long uiRegValue;

	i2c_Start1();                  			/* 总线开始信号 */
	i2c_SendByte1(BMP085_SLAVE_ADDRESS);		/* 发送设备地址+写信号 */
	i2c_WaitAck1();
	i2c_SendByte1(_ucRegAddr);				/* 发送地址 */
	i2c_WaitAck1();

	i2c_Start1();                  			/* 总线开始信号 */
	i2c_SendByte1(BMP085_SLAVE_ADDRESS + 1);/* 发送设备地址+读信号 */
	i2c_WaitAck1();

	ucData1 = i2c_ReadByte1();       		/* 读出高字节数据 */
	i2c_Ack1(); 

	ucData2 = i2c_ReadByte1();       		/* 读出中间字节数据 */
	i2c_Ack1();

	ucData3 = i2c_ReadByte1();       		/* 读出最低节数据 */
	i2c_NAck1();
	i2c_Stop1();                  			/* 总线停止信号 */

	uiRegValue = (long)((ucData1 << 12) | (ucData2 << 4) | (ucData3>>4));

	return uiRegValue;
}

/*
*********************************************************************************************************
*	函 数 名: BMP085_WaitConvert
*	功能说明: 等待内部转换结束
*	形    参: 无
*	返 回 值: 无
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
	
	adc_T = BMP085_Read3Bytes(0xFA);    // 读取温度
	BMP085_WaitConvert();
	
	adc_P = BMP085_Read3Bytes(0xF7); // 读取压强
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
