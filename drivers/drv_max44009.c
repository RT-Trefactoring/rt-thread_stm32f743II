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
// ��������: �������Ĵ������ú���
// ����˵��: val - ���ò���
// ��������: ��
// ---------------------------------------------------------------------*/
//void max44009_regcfg(rt_uint8_t val)
//{
//	  rt_uint8_t config = 0;
//	  
//	  config = (0x00<<7)|(0x00<<6)|(0x00<<3)|(0x00<<0);
//	 //Ĭ��800ms�ɼ�һ��
//	 //����Ϊ�Զ���ģʽ//���û���ʱ��800ms (���վ���0.045Lux  �����ն� 188006.4Lux)                        
//	  if (val&0x40)                //�ɼ���ʽ
//	  {
//		config |= 0x40;
//	  }
//	  if (val&0x40)                //�ɼ�ģʽ 0-�Զ�ģʽ  1-�ֶ�ģʽ
//	  {
//		config |= 0x40;
//	  }
//	  if (val&0x08)                //������
//	  {
//		config |= 0x08;
//	  }
//	  if (val&0x04)                //����ʱ��
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
//	  max44009_write_reg(0x02, config);//���ò���
//}

///*---------------------------------------------------------------------
// ��������: ���ն�ȡ����
// ����˵��: ��
// ��������: ���ع���ֵ
// ---------------------------------------------------------------------*/
//rt_uint32_t max44009_getlux(rt_uint32_t *vLux)
//{
//	  rt_uint8_t   vIndex = 0;                     //ָ��
//	  rt_uint8_t   vMantissa = 0;                  //β��
//	  rt_uint8_t   lux[2];
//	  rt_uint8_t   lux1,lux2;
//	  rt_uint16_t  val16 = 0;
//	  float    vflux = 0.0;
//		
////	  lux1=max44009_read_reg(0x03, 1, &lux1);
////	  lux2=max44009_read_reg(0x04, 1, &lux2);
//	  max44009_multi_read(0x03,2,lux);
//	  //ָ��
//	  vIndex = lux[0]&0xF0;
//	  vIndex >>= 4;
//	  
//	  //β��
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
//	  (*vLux) = (rt_uint32_t)(vflux* 1);   //С��͸����ǽ���
//		
//	 return *vLux;

//} 

///*---------------------------------------------------------------------
// ��������: ��ʼ�����մ�����
// ����˵��: ��
// ��������: ��
// ---------------------------------------------------------------------*/
//int rt_max44009_init(void)
//{
//	
//	 //i2c��ʼ��
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
// B_LUX_V30�ɼ�����
//****************************************
#include <rtthread.h>
#include "board.h"
#include "drv_dwt.h"
//RX scl A3
//tx sda D5
//���Ŷ���
#define B_LUX_SCL0_O    GPIOA->MODER   |=  (0x01<<6);  GPIOA->PUPDR |= (0x01<<6)       //����PD6������� Ĭ�����⡢2Mhz�������
#define B_LUX_SCL0_H    GPIOA->BSRRL    |=  (0x01<<3)																			//PD6��λ
#define B_LUX_SCL0_L    GPIOA->BSRRH    |=  (0x01<<3)																		//PD6��λ

#define B_LUX_SDA0_O    GPIOD->MODER    |=  (0x01<<10);  GPIOD->PUPDR |= (0x01<<10)       //����PB3������� Ĭ�����⡢2Mhz�������
#define B_LUX_SDA0_H    GPIOD->BSRRL    |=  (0x01<<5)																			//PD6��λ
#define B_LUX_SDA0_L    GPIOD->BSRRH    |=  (0x01<<5)																		//PD6��λ

#define B_LUX_SDA0_I    GPIOD->MODER    &=  ~(0x02<<10);  GPIOD->PUPDR &= ~(0x02<<10)     //��������
#define B_LUX_SDA0_DAT  ((GPIOD->IDR>>5) & 0x01)
 
//#define B_LUX_INT0_I    GPIOB->MODER    &=  ~(0x02<<14);  GPIOB->PUPDR &= ~(0x02<<14)       	//��������
//#define B_LUX_INT0_DAT  ( (GPIOB->IDR>>7) & 0x01)     //B7

#define	B_LUX_SlaveAddress	  		0x94                                                  //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�
#define B_LUX_CONFIGURATION       0x00                                                  //Ĭ��ģʽÿ800MS�ɼ�һ�Σ� �Զ�ģʽ

/*---------------------------------------------------------------------
 ��������: ��ʱ5΢��  ��ͬ�Ĺ�������,��Ҫ�����˺���
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Delay5us()
{
   	unsigned int i;
	
//	for (i = 0; i < 30; i++);
//	 rt_thread_delay( (RT_TICK_PER_SECOND*5)/1000000 );
	bsp_DelayUS(5);
}


/*---------------------------------------------------------------------
 ��������: ��ʱ5����  ��ͬ�Ĺ�������,��Ҫ�����˺���
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Delay5ms()
{
 // unsigned int i;
	
//	for (i = 0; i < 30000; i++);
	
//	 rt_thread_delay((RT_TICK_PER_SECOND*5)/1000);
	bsp_DelayMS(5);
}

/*---------------------------------------------------------------------
 ��������: ��ʱ���� ��ͬ�Ĺ�������,��Ҫ�����˺���
 ����˵��: ��	
 ��������: ��
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
 ��������: ��ʼ�ź�
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Start()
{
  B_LUX_SDA0_H;                         //����������
  B_LUX_SCL0_H;                         //����ʱ����
  B_LUX_Delay5us();                     //��ʱ
  B_LUX_SDA0_L;                         //�����½���
  B_LUX_Delay5us();                     //��ʱ
  B_LUX_SCL0_L;                         //����ʱ����
}

/*---------------------------------------------------------------------
 ��������: ֹͣ�ź�
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Stop()
{
  B_LUX_SDA0_L;                         //����������
  B_LUX_SCL0_H;                         //����ʱ����
  B_LUX_Delay5us();                     //��ʱ
  B_LUX_SDA0_H;                         //����������
  B_LUX_Delay5us();                     //��ʱ
  B_LUX_SCL0_L;
  B_LUX_Delay5us();
}

/*---------------------------------------------------------------------
 ��������: ����Ӧ���ź�
 ����˵��: ack - Ӧ���ź�(0:ACK 1:NAK)
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_SendACK(rt_uint8_t ack)
{
  if (ack&0x01)	B_LUX_SDA0_H;		//дӦ���ź�
  else	B_LUX_SDA0_L;
  
  B_LUX_SCL0_H;                         //����ʱ����
  B_LUX_Delay5us();                     //��ʱ
  B_LUX_SCL0_L;                         //����ʱ����
  B_LUX_Delay5us();
  B_LUX_SDA0_H;
  B_LUX_Delay5us();                     //��ʱ
}

/*---------------------------------------------------------------------
 ��������: ����Ӧ���ź�
 ����˵��: ��
 ��������: ����Ӧ���ź�
 ---------------------------------------------------------------------*/
rt_uint8_t B_LUX_RecvACK()
{
  rt_uint8_t CY = 0x00;
  B_LUX_SDA0_H;
  
  B_LUX_SDA0_I;
  
  B_LUX_SCL0_H;                         //����ʱ����
  B_LUX_Delay5us();                     //��ʱ
  
  
  CY |= B_LUX_SDA0_DAT;                 //��Ӧ���ź�
  
  B_LUX_Delay5us();                     //��ʱ
  
  B_LUX_SCL0_L;                         //����ʱ����
  
  B_LUX_SDA0_O;
  
  return CY;
}

/*---------------------------------------------------------------------
 ��������: ��IIC���߷���һ���ֽ�����
 ����˵��: dat - д�ֽ�
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_SendByte(rt_uint8_t dat)
{
  rt_uint8_t i;
  
  for (i=0; i<8; i++)         			//8λ������
  {
    if (dat&0x80)	B_LUX_SDA0_H;
    else	B_LUX_SDA0_L;                   //�����ݿ�
    
    B_LUX_Delay5us();             		//��ʱ
    B_LUX_SCL0_H;                		//����ʱ����
    B_LUX_Delay5us();             		//��ʱ
    B_LUX_SCL0_L;                		//����ʱ����
    B_LUX_Delay5us();             		//��ʱ
    dat <<= 1;              			//�Ƴ����ݵ����λ
  }
  
  B_LUX_RecvACK();
}

/*---------------------------------------------------------------------
 ��������: ��IIC���߽���һ���ֽ�����
 ����˵��: ��
 ��������: �����ֽ�
 ---------------------------------------------------------------------*/
rt_uint8_t B_LUX_RecvByte()
{
  rt_uint8_t i;
  rt_uint8_t dat = 0;
  B_LUX_SDA0_I;
  
  B_LUX_SDA0_H;                         //ʹ���ڲ�����,׼����ȡ����,
  for (i=0; i<8; i++)         	        //8λ������
  {
    
    
    B_LUX_SCL0_H;                       //����ʱ����
    B_LUX_Delay5us();             	//��ʱ
    dat |= B_LUX_SDA0_DAT;              //������               
    B_LUX_SCL0_L;                       //����ʱ����
    B_LUX_Delay5us();             	//��ʱ
    
    if (i<7)
      dat <<= 1;
    	
  }
  B_LUX_SDA0_O;
  
  return dat;
}

/*---------------------------------------------------------------------
 ��������: дMAX44009
 ����˵��: REG_Address - �Ĵ�����ַ
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Single_Write(rt_uint8_t REG_Address, rt_uint8_t REG_data)
{
  B_LUX_Start();                                        //��ʼ�ź�
  B_LUX_SendByte(B_LUX_SlaveAddress);                   //�����豸��ַ+д�ź�
  B_LUX_SendByte(REG_Address);                          //�ڲ��Ĵ�����ַ����ο�����pdf22ҳ 
  B_LUX_SendByte(REG_data);                             //�ڲ��Ĵ������ݣ���ο�����pdf22ҳ 
  B_LUX_Stop();                                         //����ֹͣ�ź�
  B_LUX_Delay5ms();
}

/*---------------------------------------------------------------------
 ��������: ��MAX44009�ڲ�����
 ����˵��: ��
 ��������: ��
 ---------------------------------------------------------------------*/
rt_uint8_t B_LUX_read(rt_uint8_t REG_Address)
{  
  rt_uint8_t rval = 0;
  B_LUX_Start();                                        //��ʼ�ź�
  B_LUX_SendByte(B_LUX_SlaveAddress+0);                 //�����豸��ַ+���ź�
  B_LUX_SendByte(REG_Address);                          //�ڲ��Ĵ�����ַ����ο�����pdf22ҳ 
  B_LUX_Start();                                        //��ʼ�ź�
  B_LUX_SendByte(B_LUX_SlaveAddress+1);                 //�����豸��ַ+���ź�
  rval = B_LUX_RecvByte();                              //BUF[0]�洢0x32��ַ�е�����
  B_LUX_SendACK(1);                                     //��ӦACK
  B_LUX_Stop();                                         //ֹͣ�ź�
  B_LUX_Delay5ms();
  
  return rval;
  
}

/*---------------------------------------------------------------------
 ��������: ��MAX44009�ڲ�����
 ����˵��: ��
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Multi_read(rt_uint8_t REG_Address1, rt_uint8_t REG_Address2, rt_uint8_t *vBuf)
{   	
  //�Ĵ���1
  B_LUX_Start();                                        //��ʼ�ź�
  B_LUX_SendByte(B_LUX_SlaveAddress+0);                 //�����豸��ַ+���ź�
  B_LUX_SendByte(REG_Address1);                         //�ڲ��Ĵ�����ַ����ο�����pdf22ҳ 
  B_LUX_Start();                                        //��ʼ�ź�
  B_LUX_SendByte(B_LUX_SlaveAddress+1);                 //�����豸��ַ+���ź�
  vBuf[0] = B_LUX_RecvByte();                           //BUF[0]�洢0x32��ַ�е�����
  B_LUX_SendACK(1);

  //�����Ĵ���2
  B_LUX_Start();                                        //��ʼ�ź�
  B_LUX_SendByte(B_LUX_SlaveAddress+0);                 //�����豸��ַ+���ź�
  B_LUX_SendByte(REG_Address2);                         //�ڲ��Ĵ�����ַ����ο�����pdf22ҳ 
  B_LUX_Start();                                        //��ʼ�ź�
  B_LUX_SendByte(B_LUX_SlaveAddress+1);                 //�����豸��ַ+���ź�
  vBuf[1] = B_LUX_RecvByte();                           //BUF[0]�洢0x32��ַ�е�����
  B_LUX_SendACK(1);                                     //��ӦACK
  
  
  B_LUX_Stop();                                         //ֹͣ�ź�
  B_LUX_Delay5ms();
}

/*---------------------------------------------------------------------
 ��������: �������Ĵ������ú���
 ����˵��: val - ���ò���
 ��������: ��
 ---------------------------------------------------------------------*/
void B_Lux_RegCfg(rt_uint8_t val)
{
  rt_uint8_t valCfg = 0;
  
  valCfg = (0x00<<7)|(0x00<<6)|(0x00<<3)|(0x00<<0);               //Ĭ��800ms�ɼ�һ��   //����Ϊ�Զ���ģʽ  //���û���ʱ��800ms (���վ���0.045Lux  �����ն� 188006.4Lux)                      
  
  if (val&0x40)                //�ɼ���ʽ
  {
    valCfg |= 0x40;
  }
  if (val&0x40)                //�ɼ�ģʽ 0-�Զ�ģʽ  1-�ֶ�ģʽ
  {
    valCfg |= 0x40;
  }
  if (val&0x08)                //������
  {
    valCfg |= 0x08;
  }
  if (val&0x04)                //����ʱ��
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
  
  B_LUX_Single_Write(0x02, valCfg);                               //���ò���
}

/*---------------------------------------------------------------------
 ��������: ��ʼ�����մ�����
 ����˵��: ��
 ��������: ��
 ---------------------------------------------------------------------*/
void B_LUX_Init()
{
	 __HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
	 __HAL_RCC_GPIOD_CLK_ENABLE();			//����GPIODʱ��
	 	
	  B_LUX_SCL0_O;
	  B_LUX_SDA0_O;
	  B_LUX_SCL0_H;
	  B_LUX_SDA0_H;
	  
	  B_LUX_delay_nms(20);	                                          //��ʱ100ms
	  
	  B_Lux_RegCfg(0x00);
  
}

  
/*---------------------------------------------------------------------
 ��������: ���ն�ȡ����
 ����˵��: ��
 ��������: ���ع���ֵ
 ---------------------------------------------------------------------*/
void B_LUX_GetLux(rt_uint32_t *vLux)
{
  rt_uint8_t   vIndex = 0;                     //ָ��
  rt_uint8_t   vMantissa = 0;                  //β��
  rt_uint8_t   vLuxBuf[3];
  rt_uint16_t  val16 = 0;
  float    vflux = 0.0;
  
  B_LUX_Multi_read(0x03, 0x04, vLuxBuf);
  
  //ָ��
  vIndex = vLuxBuf[0]&0xF0;
  vIndex >>= 4;
  
  //β��
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
 
	(*vLux) = (rt_uint32_t)(vflux* 1);   //С��͸����ǽ���
	
  rt_thread_delay( RT_TICK_PER_SECOND );
//  B_LUX_delay_nms(1000);   

} 

