///*
// * File      : drv_iic.c
// * This file is part of RT-Thread RTOS
// * COPYRIGHT (C) 2017 RT-Thread Develop Team
// *
// * The license and distribution terms for this file may be
// * found in the file LICENSE in this distribution or at
// * http://www.rt-thread.org/license/LICENSE
// *
// * Change Logs:
// * Date           Author       Notes

// */

//#include <board.h>
//#include <rthw.h>
//#include <finsh.h>
//#include <rtdevice.h>

////#define DEBUG

//#ifdef DEBUG
//#define DEBUG_PRINTF(...)   rt_kprintf(__VA_ARGS__)
//#else
//#define DEBUG_PRINTF(...)   
//#endif

//#define I2C_BUS_NAME	"i2c2"

//#define MAX44009_ADDR 	0x94


//static struct rt_i2c_bus_device * max44009_i2c_bus;

//static rt_err_t max44009_read_reg(rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
//{
//	struct rt_i2c_msg msgs[2];

//    msgs[0].addr = MAX44009_ADDR;
//    msgs[0].flags = RT_I2C_WR;
//    msgs[0].buf = &reg;
//    msgs[0].len = 1;

//    msgs[1].addr = MAX44009_ADDR;
//    msgs[1].flags = RT_I2C_RD;
//    msgs[1].buf = buf;
//    msgs[1].len = len;

//    if (rt_i2c_transfer(max44009_i2c_bus, msgs, 2) == 2)
//    {
//        return RT_EOK;
//    }
//    else
//    {
//        return -RT_ERROR;
//    }
//	
//}


//static rt_err_t max44009_write_reg(rt_uint8_t reg, rt_uint8_t data)
//{

//    rt_uint8_t buf[2];

//    buf[0] = reg;
//    buf[1] = data;

//    if (rt_i2c_master_send(max44009_i2c_bus, MAX44009_ADDR, 0, buf, 2) == 2)
//    {
//        return RT_EOK;
//    }
//    else
//    {
//        return -RT_ERROR;
//    }
//	

//}




//static rt_err_t max44009_multi_read(rt_uint8_t reg, rt_uint8_t len,rt_uint8_t *buf)
//{


//	struct rt_i2c_msg msgs[3];

//    msgs[0].addr = MAX44009_ADDR;
//    msgs[0].flags = RT_I2C_WR;
//    msgs[0].buf = &reg;
//    msgs[0].len = 1;

//    msgs[1].addr = MAX44009_ADDR;
//    msgs[1].flags = RT_I2C_RD;
//    msgs[1].buf = buf;
//    msgs[1].len = len;
//	
//	msgs[2].addr = MAX44009_ADDR;
//    msgs[2].flags = RT_I2C_RD;
//    msgs[2].buf = buf;
//    msgs[2].len = len;


//    if (rt_i2c_transfer(max44009_i2c_bus, msgs, 3) == 2)
//    {
//        return RT_EOK;
//    }
//    else
//    {
//        return -RT_ERROR;
//    }

//}
///*---------------------------------------------------------------------
// 功能描述: 传感器寄存器配置函数
// 参数说明: val - 配置参数
// 函数返回: 无
// ---------------------------------------------------------------------*/
//void max44009_regcfg(rt_uint8_t val)
//{
//	  rt_uint8_t config = 0;
//	  
//	  config = (0x00<<7)|(0x00<<6)|(0x00<<3)|(0x00<<0);
//	 //默认800ms采集一次
//	 //配置为自动动模式//配置积分时间800ms (光照精度0.045Lux  最大光照度 188006.4Lux)                        
//	  if (val&0x40)                //采集方式
//	  {
//		config |= 0x40;
//	  }
//	  if (val&0x40)                //采集模式 0-自动模式  1-手动模式
//	  {
//		config |= 0x40;
//	  }
//	  if (val&0x08)                //分流比
//	  {
//		config |= 0x08;
//	  }
//	  if (val&0x04)                //积分时间
//	  {
//		config |= 0x04;
//	  }
//	  if (val&0x02)
//	  {
//		config |= 0x02;
//	  }
//	  if (val&0x01)
//	  {
//		config |= 0x01;
//	  }
//	  
//	  max44009_write_reg(0x02, config);//配置参数
//}

///*---------------------------------------------------------------------
// 功能描述: 光照读取函数
// 参数说明: 无
// 函数返回: 返回光照值
// ---------------------------------------------------------------------*/
//rt_uint32_t max44009_getlux(rt_uint32_t *vLux)
//{
//	  rt_uint8_t   vIndex = 0;                     //指数
//	  rt_uint8_t   vMantissa = 0;                  //尾数
//	  rt_uint8_t   lux[2];
//	  rt_uint8_t   lux1,lux2;
//	  rt_uint16_t  val16 = 0;
//	  float    vflux = 0.0;
//		
////	  lux1=max44009_read_reg(0x03, 1, &lux1);
////	  lux2=max44009_read_reg(0x04, 1, &lux2);
//	  max44009_multi_read(0x03,2,lux);
//	  //指数
//	  vIndex = lux[0]&0xF0;
//	  vIndex >>= 4;
//	  
//	  //尾数
//	  vMantissa   = (lux[0]&0x0F);
//	  vMantissa <<= 4;
//	  vMantissa  |= (lux[1]&0x0F);
//	  
//	  if (vIndex == 0x01)
//	  {
//		vIndex = 0x00;
//	  }
//	  else if (vIndex > 0x0E)
//	  {
//		vIndex = 0x0E;
//	  }
//	  
//	  val16 = (0x01<<vIndex);
//	  
//	  vflux = ( (float)val16 ) * ( (float)vMantissa) * 0.045f;
//	 
//	  (*vLux) = (rt_uint32_t)(vflux* 1);   //小球透明外壳矫正
//		
//	 return *vLux;

//} 

///*---------------------------------------------------------------------
// 功能描述: 初始化光照传感器
// 参数说明: 无
// 函数返回: 无
// ---------------------------------------------------------------------*/
//int rt_max44009_init(void)
//{
//	
//	 //i2c初始化
//	max44009_i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(I2C_BUS_NAME);
//	if (max44009_i2c_bus == RT_NULL)
//	{
//		DEBUG_PRINTF("\ni2c_bus %s for max44009 not found!\n", I2C_BUS_NAME);
//		return -RT_ENOSYS;
//	}

//	if (rt_device_open(&max44009_i2c_bus->parent, RT_NULL) != RT_EOK)
//	{
//		DEBUG_PRINTF("\ni2c_bus %s for max44009 opened failed!\n", I2C_BUS_NAME);
//		return -RT_EEMPTY;
//	}
//	
//	max44009_regcfg(0x00);

//	return RT_EOK;
//}

//INIT_APP_EXPORT(rt_max44009_init);


//***************************************
// B_LUX_V30采集程序
//****************************************
#include <rtthread.h>
#include "board.h"
#include "drv_dwt.h"
//RX scl A3
//tx sda D5
//引脚定义
#define B_LUX_SCL0_O    GPIOA->MODER   |=  (0x01<<6);  GPIOA->PUPDR |= (0x01<<6)       //配置PD6上拉输出 默认推免、2Mhz低速输出
#define B_LUX_SCL0_H    GPIOA->BSRRL    |=  (0x01<<3)																			//PD6置位
#define B_LUX_SCL0_L    GPIOA->BSRRH    |=  (0x01<<3)																		//PD6复位

#define B_LUX_SDA0_O    GPIOD->MODER    |=  (0x01<<10);  GPIOD->PUPDR |= (0x01<<10)       //配置PB3上拉输出 默认推免、2Mhz低速输出
#define B_LUX_SDA0_H    GPIOD->BSRRL    |=  (0x01<<5)																			//PD6置位
#define B_LUX_SDA0_L    GPIOD->BSRRH    |=  (0x01<<5)																		//PD6复位

#define B_LUX_SDA0_I    GPIOD->MODER    &=  ~(0x02<<10);  GPIOD->PUPDR &= ~(0x02<<10)     //悬浮输入
#define B_LUX_SDA0_DAT  ((GPIOD->IDR>>5) & 0x01)
 
//#define B_LUX_INT0_I    GPIOB->MODER    &=  ~(0x02<<14);  GPIOB->PUPDR &= ~(0x02<<14)       	//悬浮输入
//#define B_LUX_INT0_DAT  ( (GPIOB->IDR>>7) & 0x01)     //B7

#define	B_LUX_SlaveAddress	  		0x94                                                  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
#define B_LUX_CONFIGURATION       0x00                                                  //默认模式每800MS采集一次， 自动模式

/*---------------------------------------------------------------------
 功能描述: 延时5微秒  不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Delay5us()
{
   	unsigned int i;
	
//	for (i = 0; i < 30; i++);
//	 rt_thread_delay( (RT_TICK_PER_SECOND*5)/1000000 );
	bsp_DelayUS(5);
}


/*---------------------------------------------------------------------
 功能描述: 延时5毫秒  不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Delay5ms()
{
 // unsigned int i;
	
//	for (i = 0; i < 30000; i++);
	
//	 rt_thread_delay((RT_TICK_PER_SECOND*5)/1000);
	bsp_DelayMS(5);
}

/*---------------------------------------------------------------------
 功能描述: 延时纳秒 不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_delay_nms(rt_uint16_t k)	
{						
  rt_uint16_t i;				
  for(i=0;i<k;i++)
  {			
    B_LUX_Delay5ms();
  }						
}					




/*---------------------------------------------------------------------
 功能描述: 起始信号
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Start()
{
  B_LUX_SDA0_H;                         //拉高数据线
  B_LUX_SCL0_H;                         //拉高时钟线
  B_LUX_Delay5us();                     //延时
  B_LUX_SDA0_L;                         //产生下降沿
  B_LUX_Delay5us();                     //延时
  B_LUX_SCL0_L;                         //拉低时钟线
}

/*---------------------------------------------------------------------
 功能描述: 停止信号
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Stop()
{
  B_LUX_SDA0_L;                         //拉低数据线
  B_LUX_SCL0_H;                         //拉高时钟线
  B_LUX_Delay5us();                     //延时
  B_LUX_SDA0_H;                         //产生上升沿
  B_LUX_Delay5us();                     //延时
  B_LUX_SCL0_L;
  B_LUX_Delay5us();
}

/*---------------------------------------------------------------------
 功能描述: 发送应答信号
 参数说明: ack - 应答信号(0:ACK 1:NAK)
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_SendACK(rt_uint8_t ack)
{
  if (ack&0x01)	B_LUX_SDA0_H;		//写应答信号
  else	B_LUX_SDA0_L;
  
  B_LUX_SCL0_H;                         //拉高时钟线
  B_LUX_Delay5us();                     //延时
  B_LUX_SCL0_L;                         //拉低时钟线
  B_LUX_Delay5us();
  B_LUX_SDA0_H;
  B_LUX_Delay5us();                     //延时
}

/*---------------------------------------------------------------------
 功能描述: 接收应答信号
 参数说明: 无
 函数返回: 返回应答信号
 ---------------------------------------------------------------------*/
rt_uint8_t B_LUX_RecvACK()
{
  rt_uint8_t CY = 0x00;
  B_LUX_SDA0_H;
  
  B_LUX_SDA0_I;
  
  B_LUX_SCL0_H;                         //拉高时钟线
  B_LUX_Delay5us();                     //延时
  
  
  CY |= B_LUX_SDA0_DAT;                 //读应答信号
  
  B_LUX_Delay5us();                     //延时
  
  B_LUX_SCL0_L;                         //拉低时钟线
  
  B_LUX_SDA0_O;
  
  return CY;
}

/*---------------------------------------------------------------------
 功能描述: 向IIC总线发送一个字节数据
 参数说明: dat - 写字节
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_SendByte(rt_uint8_t dat)
{
  rt_uint8_t i;
  
  for (i=0; i<8; i++)         			//8位计数器
  {
    if (dat&0x80)	B_LUX_SDA0_H;
    else	B_LUX_SDA0_L;                   //送数据口
    
    B_LUX_Delay5us();             		//延时
    B_LUX_SCL0_H;                		//拉高时钟线
    B_LUX_Delay5us();             		//延时
    B_LUX_SCL0_L;                		//拉低时钟线
    B_LUX_Delay5us();             		//延时
    dat <<= 1;              			//移出数据的最高位
  }
  
  B_LUX_RecvACK();
}

/*---------------------------------------------------------------------
 功能描述: 从IIC总线接收一个字节数据
 参数说明: 无
 函数返回: 接收字节
 ---------------------------------------------------------------------*/
rt_uint8_t B_LUX_RecvByte()
{
  rt_uint8_t i;
  rt_uint8_t dat = 0;
  B_LUX_SDA0_I;
  
  B_LUX_SDA0_H;                         //使能内部上拉,准备读取数据,
  for (i=0; i<8; i++)         	        //8位计数器
  {
    
    
    B_LUX_SCL0_H;                       //拉高时钟线
    B_LUX_Delay5us();             	//延时
    dat |= B_LUX_SDA0_DAT;              //读数据               
    B_LUX_SCL0_L;                       //拉低时钟线
    B_LUX_Delay5us();             	//延时
    
    if (i<7)
      dat <<= 1;
    	
  }
  B_LUX_SDA0_O;
  
  return dat;
}

/*---------------------------------------------------------------------
 功能描述: 写MAX44009
 参数说明: REG_Address - 寄存器地址
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Single_Write(rt_uint8_t REG_Address, rt_uint8_t REG_data)
{
  B_LUX_Start();                                        //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress);                   //发送设备地址+写信号
  B_LUX_SendByte(REG_Address);                          //内部寄存器地址，请参考中文pdf22页 
  B_LUX_SendByte(REG_data);                             //内部寄存器数据，请参考中文pdf22页 
  B_LUX_Stop();                                         //发送停止信号
  B_LUX_Delay5ms();
}

/*---------------------------------------------------------------------
 功能描述: 读MAX44009内部数据
 参数说明: 无
 函数返回: 无
 ---------------------------------------------------------------------*/
rt_uint8_t B_LUX_read(rt_uint8_t REG_Address)
{  
  rt_uint8_t rval = 0;
  B_LUX_Start();                                        //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress+0);                 //发送设备地址+读信号
  B_LUX_SendByte(REG_Address);                          //内部寄存器地址，请参考中文pdf22页 
  B_LUX_Start();                                        //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress+1);                 //发送设备地址+读信号
  rval = B_LUX_RecvByte();                              //BUF[0]存储0x32地址中的数据
  B_LUX_SendACK(1);                                     //回应ACK
  B_LUX_Stop();                                         //停止信号
  B_LUX_Delay5ms();
  
  return rval;
  
}

/*---------------------------------------------------------------------
 功能描述: 读MAX44009内部数据
 参数说明: 无
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Multi_read(rt_uint8_t REG_Address1, rt_uint8_t REG_Address2, rt_uint8_t *vBuf)
{   	
  //寄存器1
  B_LUX_Start();                                        //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress+0);                 //发送设备地址+读信号
  B_LUX_SendByte(REG_Address1);                         //内部寄存器地址，请参考中文pdf22页 
  B_LUX_Start();                                        //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress+1);                 //发送设备地址+读信号
  vBuf[0] = B_LUX_RecvByte();                           //BUF[0]存储0x32地址中的数据
  B_LUX_SendACK(1);

  //连续寄存器2
  B_LUX_Start();                                        //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress+0);                 //发送设备地址+读信号
  B_LUX_SendByte(REG_Address2);                         //内部寄存器地址，请参考中文pdf22页 
  B_LUX_Start();                                        //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress+1);                 //发送设备地址+读信号
  vBuf[1] = B_LUX_RecvByte();                           //BUF[0]存储0x32地址中的数据
  B_LUX_SendACK(1);                                     //回应ACK
  
  
  B_LUX_Stop();                                         //停止信号
  B_LUX_Delay5ms();
}

/*---------------------------------------------------------------------
 功能描述: 传感器寄存器配置函数
 参数说明: val - 配置参数
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_Lux_RegCfg(rt_uint8_t val)
{
  rt_uint8_t valCfg = 0;
  
  valCfg = (0x00<<7)|(0x00<<6)|(0x00<<3)|(0x00<<0);               //默认800ms采集一次   //配置为自动动模式  //配置积分时间800ms (光照精度0.045Lux  最大光照度 188006.4Lux)                      
  
  if (val&0x40)                //采集方式
  {
    valCfg |= 0x40;
  }
  if (val&0x40)                //采集模式 0-自动模式  1-手动模式
  {
    valCfg |= 0x40;
  }
  if (val&0x08)                //分流比
  {
    valCfg |= 0x08;
  }
  if (val&0x04)                //积分时间
  {
    valCfg |= 0x04;
  }
  if (val&0x02)
  {
    valCfg |= 0x02;
  }
  if (val&0x01)
  {
    valCfg |= 0x01;
  }
  
  B_LUX_Single_Write(0x02, valCfg);                               //配置参数
}

/*---------------------------------------------------------------------
 功能描述: 初始化光照传感器
 参数说明: 无
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Init()
{
	 __HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
	 __HAL_RCC_GPIOD_CLK_ENABLE();			//开启GPIOD时钟
	 	
	  B_LUX_SCL0_O;
	  B_LUX_SDA0_O;
	  B_LUX_SCL0_H;
	  B_LUX_SDA0_H;
	  
	  B_LUX_delay_nms(20);	                                          //延时100ms
	  
	  B_Lux_RegCfg(0x00);
  
}

  
/*---------------------------------------------------------------------
 功能描述: 光照读取函数
 参数说明: 无
 函数返回: 返回光照值
 ---------------------------------------------------------------------*/
void B_LUX_GetLux(rt_uint32_t *vLux)
{
  rt_uint8_t   vIndex = 0;                     //指数
  rt_uint8_t   vMantissa = 0;                  //尾数
  rt_uint8_t   vLuxBuf[3];
  rt_uint16_t  val16 = 0;
  float    vflux = 0.0;
  
  B_LUX_Multi_read(0x03, 0x04, vLuxBuf);
  
  //指数
  vIndex = vLuxBuf[0]&0xF0;
  vIndex >>= 4;
  
  //尾数
  vMantissa   = (vLuxBuf[0]&0x0F);
  vMantissa <<= 4;
  vMantissa  |= (vLuxBuf[1]&0x0F);
  
  if (vIndex == 0x01)
  {
    vIndex = 0x00;
  }
  else if (vIndex > 0x0E)
  {
    vIndex = 0x0E;
  }
  
  val16 = (0x01<<vIndex);
  
  vflux = ( (float)val16 ) * ( (float)vMantissa) * 0.045f;
 
	(*vLux) = (rt_uint32_t)(vflux* 1);   //小球透明外壳矫正
	
  rt_thread_delay( RT_TICK_PER_SECOND );
//  B_LUX_delay_nms(1000);   

} 

