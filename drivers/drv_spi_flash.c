/*
 * File      : stm32f20x_40x_spi.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-01-01     aozima       first implementation.
 * 2012-07-27     aozima       fixed variable uninitialized.
 */
#include <board.h>
#include "drv_spi.h"
#include "spi_flash.h"
#include "spi_flash_sfud.h"

#include <rthw.h>
#include <finsh.h>

/*
 * File      : drv_spi.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2017 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-06-05     tanek        first implementation.
 */
 
#include "drv_spi.h"

#include <board.h>
#include <finsh.h>

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINTF(...)   rt_kprintf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)   
#endif

/* private rt-thread spi ops function */
static rt_err_t configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration);
static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message);

static struct rt_spi_ops stm32_spi_ops =
{
    configure,
    xfer
};


static rt_err_t configure(struct rt_spi_device* device,
                          struct rt_spi_configuration* configuration)
{
    struct rt_spi_bus * spi_bus = (struct rt_spi_bus *)device->bus;	
    struct stm32h7_spi *h7_spi = (struct stm32h7_spi *)spi_bus->parent.user_data;
	QSPI_HandleTypeDef * QspiHandle = &h7_spi->QSPI_Handler;

	RT_ASSERT(device != RT_NULL);
	RT_ASSERT(configuration != RT_NULL);

	QspiHandle->Instance=QUADSPI;  
    QspiHandle->Init.ClockPrescaler=1;                     //QPSI分频比，W25Q256最大频率为104M，
                                                            //所以此处应该为2，QSPI频率就为200/(1+1)=100MHZ
    QspiHandle->Init.FifoThreshold=4;                      //FIFO阈值为4个字节
    QspiHandle->Init.SampleShifting=QSPI_SAMPLE_SHIFTING_HALFCYCLE;//采样移位半个周期(DDR模式下,必须设置为0)
    QspiHandle->Init.FlashSize=POSITION_VAL(0X2000000)-1;  //SPI FLASH大小，W25Q256大小为32M字节
    QspiHandle->Init.ChipSelectHighTime=QSPI_CS_HIGH_TIME_5_CYCLE;//片选高电平时间为5个时钟(10*5=55ns),即手册里面的tSHSL参数
    QspiHandle->Init.ClockMode=QSPI_CLOCK_MODE_0;          //模式0
    QspiHandle->Init.FlashID=QSPI_FLASH_ID_1;              //第一片flash
    QspiHandle->Init.DualFlash=QSPI_DUALFLASH_DISABLE;     //禁止双闪存模式

    /* init SPI */
    if (HAL_QSPI_Init(QspiHandle) != HAL_OK)
	{
		return RT_ERROR;
	}
    /* Enable SPI_MASTER */
    __HAL_QSPI_ENABLE(QspiHandle) 
    DEBUG_PRINTF("qspi configuration\n");

    return RT_EOK;
};
//QSPI底层驱动,引脚配置，时钟使能
//此函数会被HAL_QSPI_Init()调用
//hqspi:QSPI句柄
void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_QSPI_CLK_ENABLE();        //使能QSPI时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();       //使能GPIOB时钟
    __HAL_RCC_GPIOF_CLK_ENABLE();       //使能GPIOF时钟
    
    //初始化PB10 片选信号
    GPIO_Initure.Pin=GPIO_PIN_10;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //复用
    GPIO_Initure.Pull=GPIO_PULLUP;              
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  //高速
    GPIO_Initure.Alternate=GPIO_AF9_QUADSPI;   //复用为QSPI
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    //PF8,9
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9;
    GPIO_Initure.Pull=GPIO_NOPULL;              
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;   //高速
	GPIO_Initure.Alternate=GPIO_AF10_QUADSPI;   //复用为QSPI
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);
    
    //PB2
    GPIO_Initure.Pin=GPIO_PIN_2;
    GPIO_Initure.Alternate=GPIO_AF9_QUADSPI;   //复用为QSPI
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    //PF6,7
    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7;
	GPIO_Initure.Alternate=GPIO_AF9_QUADSPI;   //复用为QSPI
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);
}

static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    struct rt_spi_bus * stm32_spi_bus = (struct rt_spi_bus *)device->bus;
    struct stm32h7_spi *h7_spi = (struct stm32h7_spi *)stm32_spi_bus->parent.user_data;
    struct rt_spi_configuration * config = &device->config;
    QUADSPI_TypeDef * QSPI = h7_spi->QSPI_Handler.Instance;
    struct stm32_spi_cs * stm32_spi_cs = device->parent.user_data;
    rt_uint32_t size = message->length;

	RT_ASSERT(device != NULL);
	RT_ASSERT(message != NULL);
	
	QSPI_HandleTypeDef * QspiHandle = &h7_spi->QSPI_Handler;

    /* take CS */
    if(message->cs_take)
    {
        HAL_GPIO_WritePin(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin, GPIO_PIN_RESET);
    }

    {
	
		
        if(config->data_width <= 8)
        {
            const rt_uint8_t * send_ptr = message->send_buf;
            rt_uint8_t * recv_ptr = message->recv_buf;
			
			
			
            while(size--)
            {
                rt_uint8_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }
                
                // Todo: replace register read/write by stm32f4 lib
                //Wait until the transmit buffer is empty
                while ((QSPI->SR & QSPI_FLAG_TC) == RESET);
                // Send the byte
				QSPI->DR = data;

                //Wait until a data is received
                while ((QSPI->SR & QSPI_FLAG_TC) == RESET);
                // Get the received data
                data = QSPI->DR;

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
        else if(config->data_width <= 16)
        {
            const rt_uint16_t * send_ptr = message->send_buf;
            rt_uint16_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint16_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while ((QSPI->SR & QSPI_FLAG_TC) == RESET);
                // Send the byte
                QSPI->DR = data;

                //Wait until a data is received
                while ((QSPI->SR & QSPI_FLAG_TC) == RESET);
                // Get the received data
                data = QSPI->DR;

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
    }

    /* release CS */
    if(message->cs_release)
    {
        //GPIO_SetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
		HAL_GPIO_WritePin(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin, GPIO_PIN_SET);
    }

    return message->length;
};







static struct stm32h7_spi stm32h7_spi1 = 
{
    /* .spi_handle = */
	{
         QUADSPI,
    },
 
};

static struct rt_spi_bus qspi_bus;


/** \brief init and register stm32 spi bus.
 *
 * \param SPI: STM32 SPI, e.g: SPI1,SPI2,SPI3.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return
 *
 */
rt_err_t stm32_spi_bus_register(QUADSPI_TypeDef * SPI,
                            //struct stm32_spi_bus * stm32_spi,
                            const char * spi_bus_name)
{
    struct stm32h7_spi * p_spi_bus;
    struct rt_spi_bus *  spi_bus;
    
    RT_ASSERT(SPI != RT_NULL);
    //RT_ASSERT(stm32_spi != RT_NULL);
    RT_ASSERT(spi_bus_name != RT_NULL);
        

    __HAL_RCC_QSPI_CLK_ENABLE();        //使能QSPI时钟
    spi_bus = &qspi_bus;

    
    spi_bus->parent.user_data = &stm32h7_spi1;
    
    return rt_spi_bus_register(spi_bus, spi_bus_name, &stm32_spi_ops);
}



static int rt_hw_qspi_init(void)
{
    /* register spi bus */
    {
        GPIO_InitTypeDef GPIO_InitStructure;
		rt_err_t result;
	
		__HAL_RCC_GPIOF_CLK_ENABLE();

        GPIO_InitStructure.Alternate  = GPIO_AF5_SPI5;
        GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStructure.Pull  = GPIO_PULLUP;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);

		result = stm32_spi_bus_register(QUADSPI, "qspi");
        if (result != RT_EOK)
		{
			return result;
		}
    }

    /* attach cs */
    {
        static struct rt_spi_device spi_device;
        static struct stm32_spi_cs  spi_cs;
		rt_err_t result;

        GPIO_InitTypeDef GPIO_InitStructure;		
		  //初始化PB10 片选信号
		GPIO_InitStructure.Pin=GPIO_PIN_10;
		GPIO_InitStructure.Mode=GPIO_MODE_AF_PP;          //复用
		GPIO_InitStructure.Pull=GPIO_PULLUP;              
		GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  //高速
		GPIO_InitStructure.Alternate=GPIO_AF9_QUADSPI;   //复用为QSPI
		HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
		
        spi_cs.GPIOx = GPIOB;
        spi_cs.GPIO_Pin = GPIO_PIN_10;
       
        GPIO_InitStructure.Pin = spi_cs.GPIO_Pin;
        HAL_GPIO_WritePin(spi_cs.GPIOx, spi_cs.GPIO_Pin, GPIO_PIN_SET);
        HAL_GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);

        result = rt_spi_bus_attach_device(&spi_device, "qspi", "qspi", (void*)&spi_cs);
		if (result != RT_EOK)
		{
			return result;
		}
    }

	return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_qspi_init);

static int rt_hw_spi_flash_with_sfud_init(void)
{
    if (RT_NULL == rt_sfud_flash_probe("W25Q256", "qspi"))
    {
        return RT_ERROR;
    };

	return RT_EOK;
}

INIT_COMPONENT_EXPORT(rt_hw_spi_flash_with_sfud_init);
